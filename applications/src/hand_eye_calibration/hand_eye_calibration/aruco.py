"""ArUco marker detection and planar pose estimation."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .hand_eye import make_transform


@dataclass
class MarkerObservation:
    corners: np.ndarray          # (4, 2) pixel coordinates
    T_cam_marker: np.ndarray     # 4x4, marker pose in camera optical frame
    reproj_error_px: float
    variant: str


def _dictionary(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary {name!r}")
    dict_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def _detector_parameters():
    params = cv2.aruco.DetectorParameters_create()
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.1
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 4
    params.adaptiveThreshConstant = 7
    return params


def _object_points(size_m: float) -> np.ndarray:
    h = float(size_m) / 2.0
    # ArUco corner order: top-left, top-right, bottom-right, bottom-left.
    # Marker y axis points down, z axis points out of the marker plane.
    return np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float64,
    )


def _camera_params(camera_info) -> tuple[np.ndarray, np.ndarray]:
    K = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    D = np.asarray(camera_info.d, dtype=np.float64)
    if D.size == 0:
        D = np.zeros(5, dtype=np.float64)
    return K, D


def _solve_pnp(obj_points: np.ndarray, corners: np.ndarray, K: np.ndarray, D: np.ndarray):
    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(obj_points, corners, K, D, flags=flags)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE
        )
    if not ok:
        raise RuntimeError("solvePnP failed")
    return rvec, tvec


def _reprojection_error(obj_points, rvec, tvec, K, D, corners) -> float:
    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
    return float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - corners, axis=1)))


def detect_marker(
    image: np.ndarray,
    camera_info,
    dictionary_name: str = "DICT_6X6_250",
    marker_id: int = 0,
    marker_size_m: float = 0.16,
    max_reproj_error_px: float = 3.0,
) -> MarkerObservation:
    """Detect one ArUco marker and estimate its pose in the camera frame.

    Tries several grayscale variants and keeps the detection with the lowest
    reprojection error. Raises ``RuntimeError`` if no usable marker is found.
    """
    if image is None or image.size == 0:
        raise RuntimeError("empty image")
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    K, D = _camera_params(camera_info)
    obj = _object_points(marker_size_m)
    dictionary = _dictionary(dictionary_name)
    params = _detector_parameters()

    variants = [("gray", gray, 1.0)]
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        variants.append(("clahe", clahe, 1.0))
    except cv2.error:
        pass
    variants.append(
        ("upscale", cv2.resize(gray, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_LINEAR), 1.5)
    )

    best: MarkerObservation | None = None
    for name, variant, scale in variants:
        corners, ids, _ = cv2.aruco.detectMarkers(variant, dictionary, parameters=params)
        if ids is None or len(ids) == 0:
            continue
        flat_ids = [int(v) for v in np.asarray(ids).ravel()]
        if marker_id not in flat_ids:
            continue
        c = corners[flat_ids.index(marker_id)][0].astype(np.float64) / scale
        try:
            rvec, tvec = _solve_pnp(obj, c, K, D)
        except RuntimeError:
            continue
        err = _reprojection_error(obj, rvec, tvec, K, D, c)
        # Reject implausible solves (behind camera, huge marker, etc.).
        if tvec.ravel()[2] <= 0.0:
            continue
        if best is None or err < best.reproj_error_px:
            R = cv2.Rodrigues(rvec)[0]
            best = MarkerObservation(
                corners=c,
                T_cam_marker=make_transform(R, tvec.ravel()),
                reproj_error_px=err,
                variant=name,
            )

    if best is None:
        raise RuntimeError("ArUco marker not found")
    if best.reproj_error_px > max_reproj_error_px:
        raise RuntimeError(
            f"marker found but reprojection error {best.reproj_error_px:.2f}px "
            f"> {max_reproj_error_px:.2f}px"
        )
    return best

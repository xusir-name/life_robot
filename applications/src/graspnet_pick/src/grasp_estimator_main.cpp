#include <rclcpp/rclcpp.hpp>

#include <exception>

#include "graspnet_pick/estimator_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<graspnet_pick::GraspEstimator>();
    rclcpp::spin(node);
  } catch (const std::exception &exc) {
    RCLCPP_FATAL(rclcpp::get_logger("grasp_estimator"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

# 单个抓取候选（相机光学坐标系）
# 抓取帧约定：原点 = 两指中点（TCP），z = approach（指向物体），
#              y = 闭合轴，x = y × z，右手系。
std_msgs/Header header

geometry_msgs/Pose pose
float32 score
float32 width_m
sensor_msgs/RegionOfInterest source_box
string backend

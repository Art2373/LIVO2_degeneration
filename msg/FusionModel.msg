# Given fusion model. T_IC maps camera optical coordinates into IMU coordinates.
# The provider supplies parameters; this message contains no estimated quality score.
std_msgs/Header header
uint32 revision
# Unique provider process session; independent of the parameter revision.
string session_id
string source
string reference_sha256
geometry_msgs/Transform imu_from_camera
sensor_msgs/CameraInfo camera_info
# Applied once to original image stamps by the frontend.
float64 camera_time_offset

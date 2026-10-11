#pragma once

// Keep original Avia deployments available; MID360 uses the actual driver2
// generated message type, rather than relying on a coincidentally equal MD5.
#ifdef FASTLIVO_USE_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/CustomMsg.h>
using LivoxMsg = livox_ros_driver2::CustomMsg;
#else
#include <livox_ros_driver/CustomMsg.h>
using LivoxMsg = livox_ros_driver::CustomMsg;
#endif

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "hector_uav_msgs/Altimeter.h"


#include <Eigen/Geometry>
#include <deque>

#include "pose_ekf.h"
#include <algorithm>

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/LocalCartesian.hpp>

std::shared_ptr<GeographicLib::Geoid> geoid;

using namespace std;
using namespace Eigen;

enum SensorType
{
  IMU,
  FIX,
  FIX_VELOCITY,
  MAGNETIC,
  SONAR,
  ALTIMETER,
};

Pose_ekf pose_ekf;
ros::Publisher pose_pub;

nav_msgs::Path path_msg;
ros::Publisher pub_path;

deque< pair<double, sensor_msgs::Imu> > imu_q;
deque< pair<double, geometry_msgs::Vector3Stamped> > mag_q;
deque< pair<double, hector_uav_msgs::Altimeter> >altimeter_q;
deque< pair<double, sensor_msgs::Range> >sonar_height_q;
deque< pair<double, Vector3d> >fix_q;
deque< pair<double, geometry_msgs::Vector3Stamped> >fix_velocity_q;


















/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, double> scale_linear_map;

  std::map<std::string, int> axis_angular_map;
  std::map<std::string, double> scale_angular_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

  if (nh_param->getParam("axis_linear_x", pimpl_->axis_linear_map["x"]))
  {
    nh_param->getParam("scale_linear_x", pimpl_->scale_linear_map["x"]);
  }
  else
  {
    nh_param->param<int>("axis_linear_x", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["x"], 0.5);
  }

  if (nh_param->getParam("axis_linear_y", pimpl_->axis_linear_map["y"]))
  {
    nh_param->getParam("scale_linear_y", pimpl_->scale_linear_map["y"]);
  }
  else
  {
    nh_param->param<int>("axis_linear_y", pimpl_->axis_linear_map["y"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["y"], 0.5);
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map["yaw"]))
  {
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map["yaw"]);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["yaw"], 0.5);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map[it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map[it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map, "x");
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map, "y");
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map, "z");
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map, "yaw");
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map, "pitch");
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map, "roll");

  // if(cmd_vel_msg.linear.x>0)
  //   cmd_vel_msg.linear.x = 0.2;
  // else if(cmd_vel_msg.linear.x<0)
  //   cmd_vel_msg.linear.x = -0.2;
  // else
  //   cmd_vel_msg.linear.x = 0.0;

  // if(cmd_vel_msg.angular.z>0)
  //   cmd_vel_msg.angular.z = 1.0;
  // else if(cmd_vel_msg.angular.z<0)
  //   cmd_vel_msg.angular.z = -1.0;
  // else
  //   cmd_vel_msg.angular.z = 0.0;

  std::cout << "x:" << std::setw(6) << std::fixed << std::setprecision(2) << cmd_vel_msg.linear.x
            << "   y:" << std::setw(6) << std::fixed << std::setprecision(2) << cmd_vel_msg.linear.y
            << "   yaw:" << std::setw(6) << std::fixed << std::setprecision(2) << cmd_vel_msg.angular.z << "\r";
  std::cout.flush();

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  sendCmdVelMsg(joy_msg, "normal");
}

}  // namespace teleop_twist_joy

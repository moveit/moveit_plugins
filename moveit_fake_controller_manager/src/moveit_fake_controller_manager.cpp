/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Ioan A. Sucan nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Dave Coleman, Robert Haschke */

#include "moveit_fake_controllers.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <iterator>

namespace moveit_fake_controller_manager
{
const std::string DEFAULT_TYPE = "interpolation";

class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItFakeControllerManager() : node_handle_("~")
  {
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM("MoveItFakeControllerManager: No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("MoveItFakeControllerManager: controller_list should be specified as an array");
      return;
    }

    pub_ = node_handle_.advertise<sensor_msgs::JointState>("fake_controller_joint_states", 100, false);

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        ROS_ERROR("MoveItFakeControllerManager: Name and joints must be specifed for each controller");
        continue;
      }

      try
      {
        std::string name = std::string(controller_list[i]["name"]);

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM("MoveItFakeControllerManager: The list of joints for controller "
                           << name << " is not specified as an array");
          continue;
        }
        std::vector<std::string> joints;
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          joints.push_back(std::string(controller_list[i]["joints"][j]));

        const std::string &type =
            controller_list[i].hasMember("type") ? std::string(controller_list[i]["type"]) : DEFAULT_TYPE;
        if (type == "last point")
          controllers_[name].reset(new LastPointController(name, joints, pub_));
        else if (type == "via points")
          controllers_[name].reset(new ViaPointController(name, joints, pub_));
        else
          controllers_[name].reset(new InterpolatingController(name, joints, pub_));
      }
      catch (...)
      {
        ROS_ERROR("MoveItFakeControllerManager: Unable to parse controller information");
      }
    }
  }

  virtual ~MoveItFakeControllerManager()
  {
  }

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
      ROS_FATAL_STREAM("No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names)
  {
    for (std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM("Returned " << names.size() << " controllers in list");
  }

  /*
   * Fake controllers are always active
   */
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Fake controllers are always loaded
   */
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on "
               "the param server?", name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate)
  {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;
  std::map<std::string, BaseFakeControllerPtr> controllers_;
};

}  // end namespace moveit_fake_controller_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_fake_controller_manager::MoveItFakeControllerManager,
                       moveit_controller_manager::MoveItControllerManager);

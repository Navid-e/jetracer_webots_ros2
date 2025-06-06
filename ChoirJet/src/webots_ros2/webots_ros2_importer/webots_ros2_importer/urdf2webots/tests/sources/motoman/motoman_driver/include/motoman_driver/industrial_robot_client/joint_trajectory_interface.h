﻿/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_TRAJECTORY_INTERFACE_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_TRAJECTORY_INTERFACE_H

#include <algorithm>
#include <vector>
#include <string>
#include <map>
#include "ros/ros.h"
#include "industrial_msgs/CmdJointTrajectory.h"
#include "motoman_msgs/CmdJointTrajectoryEx.h"
#include "industrial_msgs/StopMotion.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/simple_message.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "motoman_driver/industrial_robot_client/robot_group.h"

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * \brief Message handler that relays joint trajectories to the robot controller
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryInterface
{

public:

  /**
   * \brief Default constructor.
   */
  JointTrajectoryInterface() : default_joint_pos_(0.0), default_vel_ratio_(0.1), default_duration_(10.0) {};
  typedef std::map<int, RobotGroup>::iterator it_type;

  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  virtual bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION, bool version_0 = false);

  /**
   * \brief Initialize robot connection using specified method.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * \return true on success, false otherwise
   */
  virtual bool init(SmplMsgConnection* connection);

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::map<int, RobotGroup> &robot_groups,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  virtual ~JointTrajectoryInterface();

  /**
   * \brief Begin processing messages and publishing topics.
   */
  virtual void run()
  {
    ros::spin();
  }

protected:

  /**
   * \brief Send a stop command to the robot
   */
  virtual void trajectoryStop();

  /**
   * \brief Convert ROS trajectory message into stream of SimpleMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] traj ROS JointTrajectory message
   * \param[out] msgs list of SimpleMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool trajectory_to_msgs(const motoman_msgs::DynamicJointTrajectoryConstPtr &traj, std::vector<SimpleMessage>* msgs);

  /**
   * \brief Convert ROS trajectory message into stream of SimpleMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] traj ROS JointTrajectory message
   * \param[out] msgs list of SimpleMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<SimpleMessage>* msgs);

  /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pt_in trajectory-point, in same order as expected for robot-connection.
   * \param[out] pt_out transformed trajectory-point (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const trajectory_msgs::JointTrajectoryPoint& pt_in, trajectory_msgs::JointTrajectoryPoint* pt_out)
  {
    *pt_out = pt_in;  // by default, no transform is applied
    return true;
  }

  virtual bool transform(const motoman_msgs::DynamicJointsGroup& pt_in, motoman_msgs::DynamicJointsGroup* pt_out)
  {
    *pt_out = pt_in;  // by default, no transform is applied
    return true;
  }

  /**
   * \brief Select specific joints for sending to the robot
   *
   * \param[in] ros_joint_names joint names from ROS command
   * \param[in] ros_pt target pos/vel from ROS command
   * \param[in] rbt_joint_names joint names, in order/count expected by robot connection
   * \param[out] rbt_pt target pos/vel, matching rbt_joint_names
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<std::string>& ros_joint_names, const motoman_msgs::DynamicJointsGroup& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, motoman_msgs::DynamicJointsGroup* rbt_pt);


  /**
   * \brief Select specific joints for sending to the robot
   *
   * \param[in] ros_joint_names joint names from ROS command
   * \param[in] ros_pt target pos/vel from ROS command
   * \param[in] rbt_joint_names joint names, in order/count expected by robot connection
   * \param[out] rbt_pt target pos/vel, matching rbt_joint_names
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<std::string>& ros_joint_names, const trajectory_msgs::JointTrajectoryPoint& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, trajectory_msgs::JointTrajectoryPoint* rbt_pt);


  /**
   * \brief Create SimpleMessage for sending to the robot
   *
   * \param[in] seq sequence # of this point in the overall trajectory
   * \param[in] pt  trajectory point data
   * \param[out] msg message for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool create_message(int seq, const trajectory_msgs::JointTrajectoryPoint &pt, SimpleMessage* msg);

  virtual bool create_message(int seq, const motoman_msgs::DynamicJointsGroup &pt, SimpleMessage* msg);

  virtual bool create_message_ex(int seq, const motoman_msgs::DynamicJointPoint &pt, SimpleMessage* msg);

  /**
   * \brief Reduce the ROS velocity commands (per-joint velocities) to a single scalar for communication to the robot.
   *   If unneeded by the robot server, set to 0 (or any value).
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_velocity computed velocity scalar for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity);

  /**
   * \brief Reduce the ROS velocity commands (per-joint velocities) to a single scalar for communication to the robot.
   *   If unneeded by the robot server, set to 0 (or any value).
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_velocity computed velocity scalar for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_velocity(const motoman_msgs::DynamicJointsGroup& pt, double* rbt_velocity);

  /**
   * \brief Compute the expected move duration for communication to the robot.
   *   If unneeded by the robot server, set to 0 (or any value).
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_duration computed move duration for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_duration(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_duration);

  /**
   * \brief Compute the expected move duration for communication to the robot.
   *   If unneeded by the robot server, set to 0 (or any value).
   *
   * \param[in] pt trajectory point data, in order/count expected by robot connection
   * \param[out] rbt_duration computed move duration for robot message (if needed by robot)
   *
   * \return true on success, false otherwise
   */
  virtual bool calc_duration(const motoman_msgs::DynamicJointsGroup& pt, double* rbt_duration);

  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param messages List of SimpleMessages to send to robot.
   *
   * \return true on success, false otherwise
   */
  virtual bool send_to_robot(const std::vector<SimpleMessage>& messages) = 0;

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform ROS message into SimpleMessage objects and send commands to robot.
   *
   * \param msg DynamicJointTrajectory message from ROS trajectory-planner
   */
  virtual void jointTrajectoryExCB(const motoman_msgs::DynamicJointTrajectoryConstPtr &msg);

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform ROS message into SimpleMessage objects and send commands to robot.
   *
   * \param msg JointTrajectory message from ROS trajectory-planner
   */
  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  /**
   * \brief Callback function registered to ROS stopMotion service
   *   Sends stop-motion command to robot.
   *
   * \param req StopMotion request from service call
   * \param res StopMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool stopMotionCB(industrial_msgs::StopMotion::Request &req,
                            industrial_msgs::StopMotion::Response &res);

  /**
   * \brief Validate that trajectory command meets minimum requirements
   *
   * \param traj incoming trajectory
   * \return true if trajectory is valid, false otherwise
   */
  virtual bool is_valid(const motoman_msgs::DynamicJointTrajectory &traj);

  /**
   * \brief Validate that trajectory command meets minimum requirements
   *
   * \param traj incoming trajectory
   * \return true if trajectory is valid, false otherwise
   */
  virtual bool is_valid(const trajectory_msgs::JointTrajectory &traj);

  /*
   * \brief Callback for JointState topic
   *
   * \param msg JointState message
   */
  virtual void jointStateCB(const sensor_msgs::JointStateConstPtr &msg);

  virtual void jointStateCB(const sensor_msgs::JointStateConstPtr &msg, int robot_id);

  TcpClient default_tcp_connection_;

  ros::NodeHandle node_;
  SmplMsgConnection* connection_;
  ros::Subscriber sub_cur_pos_;  // handle for joint-state topic subscription
  ros::Subscriber sub_joint_trajectory_;  // handle for joint-trajectory topic subscription
  ros::ServiceServer srv_joint_trajectory_;  // handle for joint-trajectory service
  ros::Subscriber sub_joint_trajectory_ex_;  // handle for joint-trajectory topic subscription
  ros::ServiceServer srv_joint_trajectory_ex_;  // handle for joint-trajectory service
  ros::ServiceServer srv_stop_motion_;   // handle for stop_motion service

  std::map<int, ros::ServiceServer> srv_stops_;
  std::map<int, ros::ServiceServer> srv_joints_;
  std::map<int, ros::Subscriber> sub_joint_trajectories_;
  std::map<int, ros::Subscriber> sub_cur_positions_;

  std::vector<std::string> all_joint_names_;
  std::map<int, RobotGroup> robot_groups_;
  bool version_0_;
  double default_joint_pos_;  // default position to use for "dummy joints", if none specified
  double default_vel_ratio_;  // default velocity ratio to use for joint commands, if no velocity or max_vel specified
  double default_duration_;   // default duration to use for joint commands, if no
  std::map<std::string, double> joint_vel_limits_;  // cache of max joint velocities from URDF
  sensor_msgs::JointState cur_joint_pos_;  // cache of last received joint state

  std::map<int, sensor_msgs::JointState> cur_joint_pos_map_;

private:
  /**
   * \brief Callback function registered to ROS CmdJointTrajectory service
   *   Duplicates message-topic functionality, but in service form.
   *
   * \param req CmdJointTrajectory request from service call
   * \param res CmdJointTrajectory response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded
   */
  bool jointTrajectoryCB(industrial_msgs::CmdJointTrajectory::Request &req,
                         industrial_msgs::CmdJointTrajectory::Response &res);


  bool jointTrajectoryExCB(motoman_msgs::CmdJointTrajectoryEx::Request &req,
                           motoman_msgs::CmdJointTrajectoryEx::Response &res);
};

}  // namespace joint_trajectory_interface
} //industrial_robot_client

#endif // MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_TRAJECTORY_INTERFACE_H

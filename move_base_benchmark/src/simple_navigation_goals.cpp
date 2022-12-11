/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <chrono>  // NOLINT

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "ros/ros.h"
#include "tf/tf.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseActionClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");

  // tell the action client that we want to spin a thread by default
  MoveBaseActionClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  ros::NodeHandle private_nh("~");
  std::string global_frame;
  private_nh.param("global_costmap/global_frame", global_frame,
                   std::string("map"));

  double goal_pose_x, goal_pose_y, goal_pose_a;
  if (!private_nh.getParam("goal_pose_x", goal_pose_x)) {
    ROS_ERROR("Goal pose is unavailable");
    return 1;
  }
  if (!private_nh.getParam("goal_pose_y", goal_pose_y)) {
    ROS_ERROR("Goal pose is unavailable");
    return 1;
  }
  if (!private_nh.getParam("goal_pose_a", goal_pose_a)) {
    ROS_ERROR("Goal pose is unavailable");
    return 1;
  }

  // we'll send a goal to the robot to move
  goal.target_pose.header.frame_id = global_frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_pose_x;
  goal.target_pose.pose.position.y = goal_pose_y;
  goal.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(goal_pose_a);

  const auto start_timestamp = std::chrono::system_clock::now();

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The base reached the goal successfully");
  } else {
    ROS_INFO("The base failed to reach the goal for some reason");
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ROS_INFO("Total runtime=%f secs", diff.count());

  return 0;
}

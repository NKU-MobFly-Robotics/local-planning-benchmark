#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <chrono>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  // tell the action client that we want to spin a thread by default
  MoveBaseActionClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  ros::NodeHandle private_nh("~");
  std::string global_frame;
  private_nh.param("global_costmap/global_frame", global_frame, std::string("map"));

  double goal_pose_x, goal_pose_y, goal_pose_a;
  if (!private_nh.getParam("goal_pose_x", goal_pose_x))
  {
    ROS_ERROR("Goal pose is unavailable");
    return -1;
  }
  if (!private_nh.getParam("goal_pose_y", goal_pose_y))
  {
    ROS_ERROR("Goal pose is unavailable");
    return -1;
  }
  if (!private_nh.getParam("goal_pose_a", goal_pose_a))
  {
    ROS_ERROR("Goal pose is unavailable");
    return -1;
  }

  // we'll send a goal to the robot to move
  goal.target_pose.header.frame_id = global_frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_pose_x;
  goal.target_pose.pose.position.y = goal_pose_y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_a);

  const auto start_t = std::chrono::high_resolution_clock::now();

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base reached the goal successfully");
  else
    ROS_INFO("The base failed to reach the goal for some reason");

  const auto end_t = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;
  ROS_INFO("Total runtime=%f secs", timediff.count());

  return 0;
}
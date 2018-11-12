#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_planning/planning_request.h"

#include <actionlib/server/simple_action_server.h>
#include <moveit_planning/planning_actionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/PoseStamped.h"

using namespace std;

geometry_msgs::PoseStamped rpose;
bool first_rpose = false;
void robot_pose_cb(geometry_msgs::PoseStamped p ) {
  rpose = p;
  first_rpose = true;
}

int main (int argc, char **argv) {

  ros::init(argc, argv, "test_plan_and_move");

  ros::NodeHandle n;
  ros::Subscriber robot_pose = n.subscribe("/kuka_kr60/cartesian_pose", 0, robot_pose_cb );

  ros::spinOnce();

  while(!first_rpose) {
    sleep(1);
    ros::spinOnce();
  }

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<moveit_planning::planning_actionAction> ac("plan_and_move", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  moveit_planning::planning_actionGoal goal;
  goal.goal_pose = rpose.pose;
  goal.goal_pose.position.z -= 0.4;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else {
    ROS_INFO("Action did not finish before the time out.");
    exit(0);
  }
  sleep(2);
  cout << "Start new motion!" << endl;

  goal.goal_pose = rpose.pose;
  //goal.goal_pose.position.z += 0.4;
  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

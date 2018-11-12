/*
 * Copyright (C) 2018, Jonathan Cacace.
 * Email id : jonathan.cacace@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 *
*/

#include "planning_server.h"

using namespace std;

//---Get parameters
void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}
//---


planning::planning(std::string name) :
    _as(_nh, name, boost::bind(&planning::executeCB, this, _1), false),
    action_name_(name) {

  load_param(_robot_name, "kuka_kr60", "robot_name");
  load_param(_planning_group_name, "arm", "planning_group_name");
  load_param(_planning_time, 3.0, "planning_time");

  _cpose_pub = _nh.advertise<geometry_msgs::PoseStamped>(_robot_name + "/cartesian_pose", 0 );

  _group = new moveit::planning_interface::MoveGroupInterface ("arm");
  //_group->setPlannerId("RRTkConfigDefault");
  _group->setMaxVelocityScalingFactor(0.1);
	_group->setMaxAccelerationScalingFactor(0.1);
  //_group->setPlanningTime(_planning_time);
  _planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  _first_rpose = false;

}

void planning::executeCB(const moveit_planning::planning_actionGoalConstPtr &goal)  {
  geometry_msgs::Pose pose;

  //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_planning::planning_actionFeedback feedback_;
  moveit_planning::planning_actionResult result_;

  geometry_msgs::Pose target_pose;

  target_pose = goal->goal_pose;

  _group->setPoseTarget(target_pose);

  if( _group->move() ) {

    result_.success.data = true;
    _as.setSucceeded(result_);
  }
  else {
    result_.success.data = false;
  }
  ROS_INFO("%s: Succeeded", action_name_.c_str());

  // set the action state to succeeded
  _as.setSucceeded(result_);
}

void planning::get_cartesian_position() {

  ros::Rate r(5);

  while(ros::ok()) {
    const robot_state::JointModelGroup *joint_model_group =
    _group->getCurrentState()->getJointModelGroup("arm");
    _robot_pose = _group->getCurrentPose();
    _cpose_pub.publish( _robot_pose );

    _first_rpose = true;
    r.sleep();
  }
}

void planning::run() {
  //Waiting for scene initialization
  _as.start();
  sleep(2);
  
  ROS_INFO("Scene initialized! Ready to plan");

  boost::thread get_cartesian_position_t( &planning::get_cartesian_position, this );
  ros::spin();
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "seven_dof_arm_planner");
  planning p_interface("plan_and_move");
  p_interface.run();

  return 0;
}

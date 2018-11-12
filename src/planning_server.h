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

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_planning/planning_request.h"
#include <actionlib/server/simple_action_server.h>
#include <moveit_planning/planning_actionAction.h>

using namespace std;

class planning {
  public:
    planning(std::string name);
    void run();
    void get_cartesian_position();
    bool plan_and_move( moveit_planning::planning_request::Request &req,
                                  moveit_planning::planning_request::Response &res );
    void executeCB(const moveit_planning::planning_actionGoalConstPtr &goal) ;

  private:
    ros::NodeHandle _nh;
    ros::Publisher _cpose_pub;

    moveit::planning_interface::MoveGroupInterface *_group;
    moveit::planning_interface::PlanningSceneInterface *_planning_scene_interface;

    string _robot_name;
    string _planning_group_name;
    ros::ServiceServer _planning_service;

    std::string action_name_;
    actionlib::SimpleActionServer<moveit_planning::planning_actionAction> _as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    double _planning_time;
    geometry_msgs::PoseStamped _robot_pose;
    bool _first_rpose;

};
//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISECATKIN_DEPENDS) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//====================================CATKIN_DEPENDS=============================================================

// Based on interactive_maker_tutorials basic_controls tutorial:
/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *CATKIN_DEPENDS

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITCATKIN_DEPENDS
Y AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <vigir_interactive_marker_pose_server/SetPosition.h>

#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;


ros::Publisher posePublisher_;
geometry_msgs::PoseStamped out_pose_;
geometry_msgs::Pose user_requested_pose_;
std::string p_frame_id_;
std::string p_marker_name_;
std::vector<double> p_init_pose;
std::string p_init_pose_topic;
bool init_pose_received;
ros::Subscriber init_pose_sub;
ros::Subscriber reset_sub;
boost::shared_ptr<tf::TransformListener> tf_listener;
//ros::ServiceServer set_position_service;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

        out_pose_.pose = feedback->pose;
        out_pose_.header = feedback->header;
        posePublisher_.publish(out_pose_);
      break;
  }

  server->applyChanges();
}

void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = 0.3;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

void make6DofMarker( bool fixed, bool setPose=false )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = p_frame_id_;

  if (setPose) {
      int_marker.pose.position.x = user_requested_pose_.position.x;
      int_marker.pose.position.y = user_requested_pose_.position.y;
      int_marker.pose.position.z = user_requested_pose_.position.z;
      int_marker.pose.orientation.x = user_requested_pose_.orientation.x;
      int_marker.pose.orientation.y = user_requested_pose_.orientation.y;
      int_marker.pose.orientation.z = user_requested_pose_.orientation.z;
      int_marker.pose.orientation.w = user_requested_pose_.orientation.w;
  }
  else{
  int_marker.pose.position.x = p_init_pose[0];
  int_marker.pose.position.y = p_init_pose[1];
  int_marker.pose.position.z = p_init_pose[2];
  int_marker.pose.orientation.x = p_init_pose[3];
  int_marker.pose.orientation.y = p_init_pose[4];
  int_marker.pose.orientation.z = p_init_pose[5];
  int_marker.pose.orientation.w = p_init_pose[6];
  }
  int_marker.scale = 0.2;

  int_marker.name = p_marker_name_;
  int_marker.description = p_marker_name_;

  // insert a box
  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void resetMarker(bool set_Pose = false) {
    server.reset( new interactive_markers::InteractiveMarkerServer(p_marker_name_,"",false) );

    ros::Duration(0.1).sleep();

    make6DofMarker( false , set_Pose);

    server->applyChanges();
}

void initPoseCB(const geometry_msgs::PoseStampedConstPtr& pose) {
    tf::StampedTransform transform;
    try {
        tf_listener->waitForTransform(p_frame_id_, pose->header.frame_id , pose->header.stamp, ros::Duration(1.0));
        tf_listener->lookupTransform(p_frame_id_, pose->header.frame_id , pose->header.stamp, transform);
    } catch (tf::TransformException e) {
        ROS_ERROR_STREAM("Transformation exception: " << e.what());
        return ;
    }
    tf::Transform pose_tf;
    pose_tf.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
    pose_tf.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w));
    tf::Transform hand = transform * pose_tf;


    ROS_INFO_ONCE("Received initial position");
    p_init_pose[0] = hand.getOrigin().x();
    p_init_pose[1] = hand.getOrigin().y();
    p_init_pose[2] = hand.getOrigin().z();
    p_init_pose[3] = hand.getRotation().x();
    p_init_pose[4] = hand.getRotation().y();
    p_init_pose[5] = hand.getRotation().z();
    p_init_pose[6] = hand.getRotation().w();

    if (!init_pose_received) {
        init_pose_received = true;
        resetMarker();
    }
}

void resetMarkerCB(const std_msgs::EmptyConstPtr&) {
    resetMarker();
}

bool setMarkerPositionCB( vigir_interactive_marker_pose_server::SetPosition::Request &req, vigir_interactive_marker_pose_server::SetPosition::Response &res) {
    user_requested_pose_ = req.pointIn;
    resetMarker(true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_pose_control");
  ros::NodeHandle n;

  posePublisher_ = n.advertise<geometry_msgs::PoseStamped>("pose", 1, false);

  ros::NodeHandle private_nh_("~");

  private_nh_.param("frame_id", p_frame_id_, std::string("base"));
  private_nh_.param("marker_name", p_marker_name_, std::string("interactive_marker_pose_control"));

  std::string init_pose_str;
  private_nh_.param("init_pose",init_pose_str, std::string(""));
  std::vector<std::string> init_pose_splitted;
  boost::split(init_pose_splitted, init_pose_str, boost::is_any_of(","));
  if (init_pose_splitted.size() != 7) {
      p_init_pose.resize(7,0);
      p_init_pose[6] = 0;
  } else {
     ROS_INFO_STREAM("Setting init position to " << init_pose_str);
     for (unsigned int i = 0; i < init_pose_splitted.size(); i++) {
         try {
            p_init_pose.push_back(boost::lexical_cast<double>(init_pose_splitted[i]));
         } catch (boost::bad_lexical_cast) {
             if (i < 6) {
                p_init_pose[i] = 0;
             } else {
                 p_init_pose[i] = 1;
             }
         }
     }
  }
  init_pose_received = true;
  private_nh_.param("init_pose_topic", p_init_pose_topic, std::string(""));
  if (p_init_pose_topic.compare("") != 0) {
    tf_listener.reset(new tf::TransformListener());
    init_pose_sub = n.subscribe(p_init_pose_topic,1, &initPoseCB);
    ROS_INFO_STREAM("Waiting for initial pose on topic '" << p_init_pose_topic << "'.");
    init_pose_received = false;
  } else {
      resetMarker();
  }

  std::string reset_topic;
  private_nh_.param("reset_topic", reset_topic, std::string("reset_marker"));
  reset_sub = n.subscribe(reset_topic, 1, &resetMarkerCB);

 ros::ServiceServer set_position_service = private_nh_.advertiseService("set_marker_position", setMarkerPositionCB);

  ros::spin();

  server.reset();
}

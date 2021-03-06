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
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>



#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;


ros::Publisher pose_publisher_;
geometry_msgs::PoseStamped out_pose_;
std::string frame_id_;
std::string marker_name_;
Eigen::Affine3d init_pose_;
bool init_pose_received_;
ros::Subscriber init_pose_sub_;
ros::Subscriber reset_sub_;
boost::shared_ptr<tf::TransformListener> tf_listener_;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

        out_pose_.pose = feedback->pose;
        out_pose_.header = feedback->header;
        pose_publisher_.publish(out_pose_);
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

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  //control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

void make6DofMarker( bool fixed )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  tf::poseEigenToMsg(init_pose_, int_marker.pose);
  int_marker.scale = 0.2;

  int_marker.name = marker_name_;
  int_marker.description = marker_name_;

  // insert a box
  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 0.707;
  control.orientation.x = 0.707;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 0.707;
  control.orientation.x = 0;
  control.orientation.y = 0.707;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 0.707;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.707;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void resetMarker() {
    server.reset( new interactive_markers::InteractiveMarkerServer(marker_name_,"",false) );

    ros::Duration(0.1).sleep();

    make6DofMarker( false );

    server->applyChanges();
}

void initPoseCB(const geometry_msgs::PoseStampedConstPtr& pose) {
    tf::StampedTransform marker_to_pose_frame;
    try {
        tf_listener_->waitForTransform(frame_id_, pose->header.frame_id , pose->header.stamp, ros::Duration(1.0));
        tf_listener_->lookupTransform(frame_id_, pose->header.frame_id , pose->header.stamp, marker_to_pose_frame);
    } catch (tf::TransformException e) {
        ROS_ERROR_STREAM("Transformation exception: " << e.what());
        return ;
    }
    tf::Transform pose_tf;
    pose_tf.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
    pose_tf.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w));
    tf::Transform marker_pose = marker_to_pose_frame * pose_tf;


    ROS_INFO_ONCE("Received initial position");
    tf::poseTFToEigen(marker_pose, init_pose_);

    if (!init_pose_received_) {
        init_pose_received_ = true;
        resetMarker();
    }
}

void resetMarkerCB(const std_msgs::EmptyConstPtr&) {
    resetMarker();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_pose_control");
  ros::NodeHandle nh;

  pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, false);

  ros::NodeHandle private_nh_("~");

  private_nh_.param("frame_id", frame_id_, std::string("base"));
  private_nh_.param("marker_name", marker_name_, std::string("interactive_marker_pose_control"));

  std::vector<double> init_pose_vec;
  private_nh_.param<std::vector<double>>("init_pose", init_pose_vec, std::vector<double>(6, 0));
  if (init_pose_vec.size() != 6) {
    ROS_ERROR_STREAM("Init pose must have 6 elements.");
    return 0;
  }

  init_pose_ = Eigen::AngleAxisd(init_pose_vec[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(init_pose_vec[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(init_pose_vec[3], Eigen::Vector3d::UnitX());
  init_pose_.translation() = Eigen::Vector3d(init_pose_vec[0], init_pose_vec[1], init_pose_vec[2]);

  init_pose_received_ = true;
  std::string init_pose_topic;
  private_nh_.param("init_pose_topic", init_pose_topic, std::string(""));
  if (init_pose_topic != "") {
    tf_listener_.reset(new tf::TransformListener());
    init_pose_sub_ = nh.subscribe(init_pose_topic,1 , &initPoseCB);
    ROS_INFO_STREAM("Waiting for initial pose on topic '" << init_pose_topic << "'.");
    init_pose_received_ = false;
  } else {
      resetMarker();
  }

  std::string reset_topic;
  private_nh_.param("reset_topic", reset_topic, std::string("reset_marker"));
  reset_sub_ = nh.subscribe(reset_topic, 1, &resetMarkerCB);

  ros::spin();

  server.reset();
}

/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
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
 * - May 2012: Port to OpenSceneGraph. Mario Prats
 * - modified by Olena Timrova. November 2016
 */
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Group>
#include <osg/Camera>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <marker_array/marker_display.h>
#include <marker_array/frame_manager.h>

using namespace osg_utils;

int main(int argc, char *argv[])
{

   osg::Group *root=new osg::Group();

   osgViewer::Viewer viewer;

   viewer.setSceneData( root );
   viewer.setUpViewInWindow (0, 0, 800, 600);

   //provides interactive control over scene objects
   osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
   tb->setHomePosition( osg::Vec3f(5,5,5), osg::Vec3f(0,0,0), osg::Vec3f(0,0,1) );
   viewer.setCameraManipulator( tb );

   //pressing the ”s” key will display a scene statistic
   viewer.addEventHandler( new osgViewer::StatsHandler );
   viewer.addEventHandler(new osgViewer::WindowSizeHandler);

   //ros initialization with a unique name for the node
   ros::init(argc, argv, "osg_marker_with_texture");

   boost::shared_ptr<FrameManager> frame_manager = FrameManager::instance();
   MarkerDisplay marker_cli(root, *(frame_manager->getTFClient()));

   ros::WallTime last_wall_time = ros::WallTime::now();
   ros::Time last_ros_time = ros::Time::now();

   while( !viewer.done() && ros::ok())
   {
       //ROS_INFO_STREAM("sim loop ");
       ros::spinOnce();
       viewer.frame();
       
       ros::WallTime current_wall_time=ros::WallTime::now();
       ros::Time current_ros_time=ros::Time::now();
       marker_cli.update((current_wall_time-last_wall_time).toSec(), (current_ros_time-last_ros_time).toSec());
       last_wall_time=current_wall_time;
       last_ros_time=current_ros_time;

   }

   return 0;
}

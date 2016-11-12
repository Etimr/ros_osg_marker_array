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

#ifndef MARKER_DISPLAY_H
#define MARKER_DISPLAY_H

#include <map>
#include <set>

#include <osg/Node>
#include <osg/Shape>
#include <osg/Geode>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "shape_marker.h"
#include "marker_base.h"

typedef std::pair<std::string, int> MarkerID;
class MarkerDisplay
{
public:
     
      MarkerDisplay(osg::Node* root, tf::TransformListener &tf_listener);
      virtual ~MarkerDisplay();
      virtual void update(float wall_dt, float ros_dt);
      
      

protected:

      osg::ref_ptr<osg::Node> scene_node_;
      ros::NodeHandle nh;
      ros::Subscriber sub;
     

private:

     tf::MessageFilter<visualization_msgs::Marker>* tf_filter_;
     typedef boost::shared_ptr<osg_markers::MarkerBase> MarkerBasePtr;
     typedef std::vector<visualization_msgs::Marker::ConstPtr> V_MarkerMessage;
     
     //< Marker message queue. Messages are added to this as they are received, 
     //and then processed in our update () function
     V_MarkerMessage marker_queue_;   
     
     typedef std::map<MarkerID, MarkerBasePtr> M_IDToMarker;
     M_IDToMarker markers_;
     boost::mutex queue_mutex_;
     typedef std::set<MarkerBasePtr> S_MarkerBase;
     S_MarkerBase markers_with_expiration_;
     
     void processMessage( const visualization_msgs::Marker::ConstPtr& message );
     void processAdd( const visualization_msgs::Marker::ConstPtr& message );
     void processDelete( const visualization_msgs::Marker::ConstPtr& message );
     void deleteMarker(MarkerID id);
     void messageReceived(const visualization_msgs::MarkerArray::ConstPtr& array);
     void incomingMarker(const visualization_msgs::Marker::ConstPtr& message);

};
#endif /* MARKER_DISPLAY_H */

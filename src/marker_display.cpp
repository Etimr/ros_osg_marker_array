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
#include <marker_array/marker_display.h>
#include <tf/transform_listener.h>
#include <marker_array/triangle_list_marker.h>


MarkerDisplay::MarkerDisplay(osg::Node* root, tf::TransformListener &tf_listener)
{
    scene_node_ = root;
    sub = nh.subscribe( "marker", 100, &MarkerDisplay::messageReceived, this);
    tf_filter_ = new tf::MessageFilter<visualization_msgs::Marker>( tf_listener, "world", 10, nh);
    tf_filter_->registerCallback(boost::bind(&MarkerDisplay::incomingMarker, this, _1));

}

MarkerDisplay::~MarkerDisplay()
{

}

//callback function, adds markers to the tf::MessageFilter
void MarkerDisplay::messageReceived(const visualization_msgs::MarkerArray::ConstPtr& array)
{
    std::vector<visualization_msgs::Marker>::const_iterator it = array->markers.begin();
    std::vector<visualization_msgs::Marker>::const_iterator end = array->markers.end();
    for (; it != end; ++it)
    {
       const visualization_msgs::Marker& marker = *it;
       tf_filter_->add(visualization_msgs::Marker::Ptr(new visualization_msgs::Marker(marker)));
    }

}

//fills marker_queue_ array with markers from tf::MessageFilter
void MarkerDisplay::incomingMarker(const visualization_msgs::Marker::ConstPtr& message)
{
    boost::mutex::scoped_lock lock(queue_mutex_);
    marker_queue_.push_back(message);

}

//deletes markers from the scene
void MarkerDisplay::deleteMarker(MarkerID id)
{
    MarkerBasePtr marker;
    M_IDToMarker::iterator it = markers_.find( id );
    if( it != markers_.end() )
    {
        marker = it->second;
        scene_node_->asGroup()->removeChild(marker->scene_node_);
        markers_with_expiration_.erase(it->second);
        markers_.erase(it);
     }

}

//is called from the simulation loop, processes markers from the marker_queue_ array
void MarkerDisplay::update(float wall_dt, float ros_dt)
{
    V_MarkerMessage local_queue;
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        local_queue.swap( marker_queue_ );
     }

    if ( !local_queue.empty() )
    {
        V_MarkerMessage::iterator message_it = local_queue.begin();
        V_MarkerMessage::iterator message_end = local_queue.end();
        for ( ; message_it != message_end; ++message_it )
        {
           visualization_msgs::Marker::ConstPtr& marker = *message_it;
           processMessage( marker );
         }
     }

    S_MarkerBase::iterator it = markers_with_expiration_.begin();
        S_MarkerBase::iterator end = markers_with_expiration_.end();
        for (; it != end;)
        {
          MarkerBasePtr marker = *it;
          if (marker->expired())
          {
             ++it;
             deleteMarker(marker->getID());
           }
           else
           {
              ++it;
            }
         }
}

void MarkerDisplay::processMessage( const visualization_msgs::Marker::ConstPtr& message )
{
    switch ( message->action )
    {
    case visualization_msgs::Marker::ADD:
        processAdd( message );
        break;

    case visualization_msgs::Marker::DELETE:
        processDelete( message );
        break;

    default:
        ROS_ERROR( "Unknown marker action: %d\n", message->action );
    }
}

//if a marker is already displayed on the screen, replace it with the new one
void MarkerDisplay::processAdd( const visualization_msgs::Marker::ConstPtr& message )
{
    bool create = true;
    MarkerBasePtr marker;

    M_IDToMarker::iterator it = markers_.find( MarkerID(message->ns, message->id) );
      if ( it != markers_.end() )
      {
        marker = it->second;
        markers_with_expiration_.erase(marker);
        processDelete( message );
      }

      if ( create )
      {
          switch ( message->type )
              {
              case visualization_msgs::Marker::CUBE:
              case visualization_msgs::Marker::CYLINDER:
              case visualization_msgs::Marker::SPHERE:
              case visualization_msgs::Marker::ARROW:
                {
                   //marker.reset(new osg_markers::ShapeMarker(scene_node_));
                  //markers_.insert(std::make_pair(MarkerID(message->ns, message->id), marker));
                }
                break;
               case visualization_msgs::Marker::TRIANGLE_LIST:
                {
                  marker.reset(new osg_markers::TriangleListMarker(scene_node_));
                  markers_.insert(std::make_pair(MarkerID(message->ns, message->id), marker));
                }
              break;
              default:
                ROS_ERROR( "Unknown marker type: %d", message->type );
              }
   }

    if(marker)
    {
      marker->setMessage(message);
      if (message->lifetime.toSec() > 0.0001f)
              {
                markers_with_expiration_.insert(marker);
              }
    }

}

void MarkerDisplay::processDelete( const visualization_msgs::Marker::ConstPtr& message )
{

    deleteMarker(MarkerID(message->ns, message->id));

}


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

#ifndef OSG_MARKER_BASE_H
#define OSG_MARKER_BASE_H

#include <visualization_msgs/Marker.h>

#include <ros/time.h>

#include <boost/shared_ptr.hpp>

#include <osg/Node>
#include <osg/Group>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>

namespace osg_markers {

class MarkerBase
{
public:
	typedef visualization_msgs::Marker Marker;
	typedef visualization_msgs::Marker::ConstPtr MarkerConstPtr;
        typedef std::pair<std::string, int> MarkerID;

	MarkerBase(osg::Node* parent_node);

	virtual ~MarkerBase();

	void setMessage(const Marker& message);
	void setMessage(const MarkerConstPtr& message);
        const MarkerConstPtr& getMessage() const { return message_; }
	virtual void setColor( const osg::Vec4d& color );
	    
        osg::ref_ptr<osg::Group> scene_node_;
        bool expired();
        MarkerID getID() { return MarkerID(message_->ns, message_->id); }
    

protected:

    virtual void onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message) = 0;

	osg::ref_ptr<osg::Node> base_node_;
        osg::ref_ptr<osg::PolygonMode> pm; 

        ros::Time expiration_;
	MarkerConstPtr message_;
};

typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;

}

#endif

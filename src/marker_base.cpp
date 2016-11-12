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

#include "marker_array/marker_base.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace osg_markers {

MarkerBase::MarkerBase(osg::Node* parent_node)
{
    base_node_=parent_node;
    scene_node_=new osg::PositionAttitudeTransform();
    scene_node_->setName("MarkerBase Scale PAT");

    //Set a scene to the polygon mode
    //pm = new osg::PolygonMode;
    //pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    //scene_node_->getOrCreateStateSet()->setAttribute(pm.get());

    base_node_->asGroup()->addChild(scene_node_);
    message_.reset();

}

MarkerBase::~MarkerBase()
{
}

void MarkerBase::setMessage(const Marker& message)
{
    MarkerConstPtr message_ptr( new Marker(message) );
	setMessage( message_ptr );
}

void MarkerBase::setMessage(const MarkerConstPtr& message)
{
    MarkerConstPtr old = message_;
    message_ = message;
    expiration_ = ros::Time::now() + message->lifetime;
    onNewMessage(old, message);

}

bool MarkerBase::expired()
{
  return ros::Time::now() >= expiration_;
}


void MarkerBase::setColor( const osg::Vec4d& color )
{
     osg::ref_ptr < osg::Material > material = new osg::Material();
     material->setDiffuse(osg::Material::FRONT_AND_BACK,color);
     scene_node_->getOrCreateStateSet()->setAttribute(material);
}

}

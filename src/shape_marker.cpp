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


#include <ros/assert.h>

#include "marker_array/shape_marker.h"
#include <osg/Shape>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
#include <cmath>

namespace osg_markers 
{

ShapeMarker::ShapeMarker( osg::Node* parent_node ) :
		  MarkerBase(parent_node)
{
}

ShapeMarker::~ShapeMarker()
{
}

void ShapeMarker::onNewMessage( const MarkerConstPtr& old_message,
		const MarkerConstPtr& new_message )
{
	if (shape_.valid() || !old_message || old_message->type != new_message->type)
	{
		shape_.release();

        text = new_message->text;
        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
        texture->setDataVariance(osg::Object::DYNAMIC);
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile( "Images/" +text );

        if (!image)
        {
        std::cout << "Couldn't load texture." << std::endl;
        }
        texture->setImage( image.get() );

		switch (new_message->type)
		{
        case visualization_msgs::Marker::CUBE:
        {
            shape_ = new osg::ShapeDrawable(new osg::Box());
            osg::Geode *geode = new osg::Geode();
            geode->addDrawable(shape_);

            geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get());

			scene_node_->asGroup()->addChild(geode);
		}
		break;

        case visualization_msgs::Marker::CYLINDER:
        case visualization_msgs::Marker::ARROW:
        {
            shape_ = new osg::ShapeDrawable(new osg::Cylinder());
            osg::ref_ptr<osg::Geode> geode = new osg::Geode();
            geode->addDrawable(shape_);

            geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get());

            scene_node_->asGroup()->addChild(geode);
         }
         break;

         case visualization_msgs::Marker::SPHERE:
         {
             shape_ = new osg::ShapeDrawable(new osg::Sphere());
             osg::ref_ptr<osg::Geode> geode = new osg::Geode();
             geode->addDrawable(shape_);

             geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get());

             scene_node_->asGroup()->addChild(geode);
          }
          break;

          default:
          ROS_BREAK();
          break;
          }
	}

}

}

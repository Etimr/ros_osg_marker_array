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
#include "marker_array/triangle_list_marker.h"
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/ReadFile>
#include <math.h>
#include <stdexcept>
namespace osg_markers
{

TriangleListMarker::TriangleListMarker(osg::Node* parent_node)
: MarkerBase(parent_node)
{

}

TriangleListMarker::~TriangleListMarker()
{
}

void TriangleListMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
        ROS_ASSERT(new_message->type == visualization_msgs::Marker::TRIANGLE_LIST);

          // if the names of textures are passed as text fields in the ROS marker
//        text = new_message->text;
//        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
//        texture->setDataVariance(osg::Object::DYNAMIC);
//        osg::ref_ptr<osg::Image> image = osgDB::readImageFile( "Images/" +text );

        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
        texture->setDataVariance(osg::Object::DYNAMIC);
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile( "Images/images_box.jpg" );

        if (!image)
        {
        std::cout << "Couldn't load texture." << std::endl;
        }
        texture->setImage( image.get() );

		geom_ = new osg::Geometry;
        triangle_list_ = new osg::Vec3Array;
        color_list_ = new osg::Vec4Array;
        normals = new osg::Vec3Array;
        texcoords = new osg::Vec2Array;

        geom_->setVertexArray( triangle_list_.get() );
        geom_->setColorArray( color_list_.get() );
        geom_->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom_->setNormalArray(normals.get(), osg::Array::BIND_OVERALL);
        geom_->setTexCoordArray(0, texcoords.get());

        prset_=new osg::DrawArrays( osg::PrimitiveSet::TRIANGLES);
        geom_->addPrimitiveSet(prset_);

        manual_object_geode_ = new osg::Geode;
        manual_object_geode_->addDrawable( geom_.get() );

        manual_object_geode_->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get());
        manual_object_geode_->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
        scene_node_->asGroup()->addChild(manual_object_geode_.get());

        texture->setUnRefImageDataAfterApply( true );

    size_t num_points = new_message->points.size();
    if ((num_points % 3) != 0)
    {
        std::stringstream ss;
        ss << "TriMesh marker has a point count which is not divisible by 3 [" << num_points <<"]";

        ROS_DEBUG("%s", ss.str().c_str());

        return;
    }

        triangle_list_->clear();
        triangle_list_->resize(num_points);
        color_list_->clear();
        color_list_->resize(num_points);
        normals->clear();
        normals->resize(num_points);

        float minX = new_message->points[0].x;
        float maxX = new_message->points[0].x;
        float minY = new_message->points[0].y;
        float maxY = new_message->points[0].y;
        float minZ = new_message->points[0].z;
        float maxZ = new_message->points[0].z;


        for (size_t i = 0; i < num_points; ++i)
        {
            (*triangle_list_)[i] = osg::Vec3(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z);
            (*color_list_)[i]=osg::Vec4(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);
            geom_->setVertexArray(triangle_list_);
            geom_->setColorArray(color_list_);
            geom_->setColorBinding(osg::Geometry::BIND_OVERALL);

            minX = calcMinMax(new_message->points[i].x, minX, "min");
            maxX = calcMinMax(new_message->points[i].x, maxX, "max");

            minY = calcMinMax(new_message->points[i].y, minY, "min");
            maxY = calcMinMax(new_message->points[i].y, maxY, "max");

            minZ = calcMinMax(new_message->points[i].z, minZ, "min");
            maxZ = calcMinMax(new_message->points[i].z, maxZ, "max");

            ((osg::DrawArrays*)prset_)->setFirst(0);
            ((osg::DrawArrays*)prset_)->setCount(num_points);
         }


         for (size_t i = 0; i < num_points; i+=3)
         {
            (*normals)[i] = calcNormal((*triangle_list_)[i], (*triangle_list_)[i+1], (*triangle_list_)[i+2]);
            (*normals)[i+1] = calcNormal((*triangle_list_)[i+1], (*triangle_list_)[i], (*triangle_list_)[i+2]);
            (*normals)[i+2] = calcNormal((*triangle_list_)[i+2], (*triangle_list_)[i], (*triangle_list_)[i+1]);
            geom_->setNormalArray(normals.get(), osg::Array::BIND_OVERALL);
         }

        for (size_t i = 0; i < num_points; ++i)
        {
            osg::Vec3 temp = osg::Vec3(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z);

            if (fabs((*normals)[i][1])> fabs((*normals)[i][0]) && fabs((*normals)[i][1])> fabs((*normals)[i][2]))
               texcoords->push_back(osg::Vec2((temp.x() - minX)/(maxX - minX), (temp.z() - minZ)/(maxZ - minZ)));

            else
               if(fabs((*normals)[i][2])> fabs((*normals)[i][0]) && fabs((*normals)[i][2])> fabs((*normals)[i][1]))
                 texcoords->push_back(osg::Vec2((temp.x() - minX)/(maxX - minX), (temp.y() - minY)/(maxY - minY)));
               else
                 texcoords->push_back(osg::Vec2((temp.y() - minY)/(maxY - minY), (temp.z() - minZ)/(maxZ - minZ)));


            geom_->setTexCoordArray(0, texcoords);
        }
}

osg::Vec3 TriangleListMarker::calcNormal(const osg::Vec3 &point1, const osg::Vec3 &point2, const osg::Vec3 &point3)
{
   osg::Vec3 normal;
   normal = (point3 - point1)^(point2 - point1);
   normal.normalize();
   return normal;
}

float TriangleListMarker::calcMinMax(float newValue, float oldValue, std::string minMax)
{
    if ( !(minMax == "min" || minMax == "max" ))
    {
       throw std::invalid_argument( "received incorrect value, correct values are: min, max" );
    }

    if (minMax == "min")
    {
        if (oldValue > newValue)
        return newValue;
        else return oldValue;
    }
    else
    {
        if (oldValue < newValue)
        return newValue;
        else return oldValue;
    }

}
}


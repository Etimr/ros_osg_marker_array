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

#ifndef OSG_SHAPE_MARKER_H
#define OSG_SHAPE_MARKER_H

#include "marker_base.h"
#include <osg/ShapeDrawable>
#include <visualization_msgs/Marker.h>
#include <osg/Geode>

namespace osg_markers
{

class ShapeMarker : public MarkerBase
{
public:
    ShapeMarker(osg::Node* parent_node);
    ~ShapeMarker();
  //  osg::Geode getNode() { return geode; }

protected:
	virtual void onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message);
       // virtual void onNewMessage(const MarkerConstPtr& message);

	osg::ref_ptr<osg::ShapeDrawable> shape_;
        osg::ref_ptr<osg::Geode> geode;
        std::string text;
        osg::Vec3 calcNormal(const osg::Vec3 &point1, const osg::Vec3 &point2, const osg::Vec3 &point3);
};

}

#endif


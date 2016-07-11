/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "DepthEstimation/DepthMapPixelHypothesis.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "DataStructures/Frame.h"

namespace lsd_slam
{


cv::Vec3b DepthMapPixelHypothesis::getVisualizationColor(int) const
{

	float id;
	id= idepth_smoothed;
	if(id < 0)
		return cv::Vec3b(255,255,255);

	// rainbow between 0 and 4
	float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
	float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
	float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

	uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
	uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
	uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

	return cv::Vec3b(255-rc,255-gc,255-bc);
}

}

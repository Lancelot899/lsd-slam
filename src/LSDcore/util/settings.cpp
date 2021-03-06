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

#include "util/settings.h"
#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>



namespace lsd_slam
{
bool autoRun = true;
bool autoRunWithinFrame = true;

bool dumpMap = false;
bool doFullReConstraintTrack = false;

float KFDistWeight = 4;
float KFUsageWeight = 3;

float minUseGrad = 5;
float cameraPixelNoise2 = 4*4;
float depthSmoothingFactor = 1;

bool allowNegativeIdepths = true;
bool useMotionModel = false;
bool useSubpixelStereo = true;
bool multiThreading = true;
bool useAffineLightningEstimation = true;

bool doSlam = true;
bool doKFReActivation = true;
bool doMapping = true;

int maxLoopClosureCandidates = 10;
int maxOptimizationIterations = 100;
int propagateKeyFrameDepthCount = 0;
float loopclosureStrictness = 1.5;
float relocalizationTH = 0.7;


bool saveKeyframes =  false;
bool saveAllTracked =  false;
bool saveLoopClosureImages =  false;
bool saveAllTrackingStages = false;
bool saveAllTrackingStagesInternal = false;

bool continuousPCOutput = false;


bool fullResetRequested = false;
bool manualTrackingLossIndicated = false;


std::string packagePath = "";


void handleKey(char)
{
}

}

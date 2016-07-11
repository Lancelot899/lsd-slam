#include <sys/types.h>
#include <sys/stat.h>

#include <deque>
#include <map>

#include "opencv2/opencv.hpp"

#include "lsdSystem.h"
#include "IOWrapper/Imgload.h"

#include "LSDcore/DataStructures/Frame.h"
#include "LSDcore/DataStructures/FrameMemory.h"
#include "LSDcore/DataStructures/FramePoseStruct.h"

#include "LSDcore/DepthEstimation/DepthMap.h"
#include "LSDcore/DepthEstimation/DepthMapPixelHypothesis.h"

#include "LSDcore/GlobalMapping/g2oTypeSim3Sophus.h"
#include "LSDcore/GlobalMapping/KeyFrameGraph.h"
#include "LSDcore/GlobalMapping/TrackableKeyFrameSearch.h"
#include "LSDcore/Tracking/TrackingReference.h"

#include "LSDcore/Tracking/SE3Tracker.h"
#include "LSDcore/Tracking/Relocalizer.h"
#include "LSDcore/Tracking/Sim3Tracker.h"
#include "LSDcore/util/globalFuncs.h"


using namespace lsd_slam;


lsdSystem::lsdSystem(int w, int h, Eigen::Matrix3f K, bool enbleSLAM)
	: SLAMEnabled(enbleSLAM), relocalizer(w, h, K)
{
	if(w%16 != 0 || h%16!=0)
	{
		printf("image dimensions must be multiples of 16! Please crop your images / video accordingly.\n");
		assert(false);
	}
	this->width = w;
	this->height = h;
	this->K = K;
	trackingIsGood = true;

	currentKeyFrame = nullptr;
	trackingReferenceFrameSharedPT = nullptr;
	keyFrameGraph = new KeyFrameGraph;
	createNewKeyFrame = false;

	map = new DepthMap(w, h, K);

	newConstraintAdded = false;
	haveUnmergedOptimizationOffset = false;

	tracker = new SE3Tracker(w, h, K);

	for(int level = 4; level < PYRAMID_LEVELS; ++level)
		tracker->settings.maxItsPerLvl[level] = 0;

	trackingReference = new TrackingReference;
	mappingTrackingReference = new TrackingReference;

	if(SLAMEnabled)
	{
		trackableKeyFrameSearch = new TrackableKeyFrameSearch(keyFrameGraph,w,h,K);
		constraintTracker = new Sim3Tracker(w,h,K);
		constraintSE3Tracker = new SE3Tracker(w,h,K);
		newKFTrackingReference = new TrackingReference();
		candidateTrackingReference = new TrackingReference();
	}
	else
	{
		constraintSE3Tracker = 0;
		trackableKeyFrameSearch = 0;
		constraintTracker = 0;
		newKFTrackingReference = 0;
		candidateTrackingReference = 0;
	}

	imgIO = new ImgIO;

	keepRunning = true;
	doFinalOptimization = false;
	lastTrackingClosenessScore = 0;

	threadMapping = boost::thread(&lsdSystem::mappingThreadLoop, this);

	if(SLAMEnabled)
	{
		threadContraintSearch = boost::thread(lsdSystem::constraintSearchThreadLoop, this);
		threadOptimization = boost::thread(&lsdSystem::optimizationThreadLoop, this);
	}

	msTrackFrame = msOptimizationIteration = msFindConstraintsItaration = msFindReferences = 0;
	nTrackFrame = nOptimizationIteration = nFindConstraintsItaration = nFindReferences = 0;
	nAvgFindConstraintsItaration = nAvgFindReferences = nAvgOptimizationIteration = nAvgTrackFrame = 0;
	gettimeofday(&lastHzUpdate, NULL);

}

lsdSystem::~lsdSystem()
{
	keepRunning = false;

	printf("... waiting for all threads of system to exit\n");

	newFrameMappedSignal.notify_all();
	unmappedTrackingFramesSignal.notify_all();
	newKeyFrameCreatedSignal.notify_all();
	newConstraintCreatedSignal.notify_all();

	threadMapping.join();
	threadContraintSearch.join();
	threadOptimization.join();

	printf("DONE waiting for all threads of system to exit\n");

	if(trackableKeyFrameSearch != 0) delete trackableKeyFrameSearch;
	if(constraintSE3Tracker != 0) delete constraintSE3Tracker;
	if(constraintTracker != 0) delete constraintTracker;
	if(newKFTrackingReference != 0) delete newKFTrackingReference;
	if(candidateTrackingReference != 0) delete candidateTrackingReference;

	delete mappingTrackingReference;
	delete map;
	delete trackingReference;
	delete tracker;

	// make sure to reset all shared pointers to all frames before deleting the keyframegraph!
	unmappedTrackingFrames.clear();
	latestFrameTriedForReloc.reset();
	latestTrackedFrame.reset();
	trackingReferenceFrameSharedPT.reset();

	delete keyFrameGraph;

	FrameMemory::getInstance().releaseBuffes();

	imgIO.reset();
}


void lsdSystem::setVisualization(std::shared_ptr<ImgIO> IOWrapper)
{
	this->imgIO = IOWrapper;
}


void lsdSystem::mergeOptimizationOffset()
{
	poseConsistencyMutex.lock();

	bool needPulish = false;

	if(haveUnmergedOptimizationOffset)
	{
		keyFrameGraph->keyframesAllMutex.lock_shared();
		for(unsigned int i = 0; i < keyFrameGraph->keyframesAll.size(); ++i)
		{
			keyFrameGraph->keyframesAll[i]->pose->applyPoseGraphOptResult();
		}
		keyFrameGraph->keyframesAllMutex.unlock_shared();

		haveUnmergedOptimizationOffset = false;
		needPulish = true;
	}

	poseConsistencyMutex.unlock();

	if(needPulish)
		publishKeyframeGraph();
}

void lsdSystem::publishKeyframeGraph()
{
	if(imgIO.get() != nullptr)
		imgIO->publishKeyframeGraph(keyFrameGraph);
}

void lsdSystem::mappingThreadLoop()
{
	printf("starting mapping thread!\n");

	while(keepRunning)
	{
		if(!doMappingIteration())
		{
			boost::unique_lock<boost::mutex> lock(unmappedTrackingFramesMutex);
			unmappedTrackingFramesSignal.timed_wait(lock, boost::posix_time::milliseconds(200));
			lock.unlock();
		}

		newFrameMappedMutex.lock();
		newFrameMappedSignal.notify_all();
		newFrameMappedMutex.unlock();
	}

	printf("exited mapping thread\n");
}

void lsdSystem::finalize()
{
	printf("Finalizing Graph ... finding final constraints!\n");

	lastNumConstraintsAddedOnFullRetrack = 1;
	while (lastNumConstraintsAddedOnFullRetrack)
	{
		doFullReConstraintTrack = true;
		usleep(200000);
	}

	printf("Finalizing Graph ... optimizing!\n");
	doFinalOptimization = true;
	newConstraintMutex.lock();
	newConstraintAdded = true;
	newConstraintCreatedSignal.notify_all();
	newConstraintMutex.unlock();
	while(doFinalOptimization)
	{
		usleep(200000);
	}

	printf("Finalizing Graph ... publish!\n");
	unmappedTrackingFramesMutex.lock();
	unmappedTrackingFramesSignal.notify_all();
	unmappedTrackingFramesMutex.unlock();

	while(doFinalOptimization)
	{
		usleep(200000);
	}

	boost::unique_lock<boost::mutex> lock(newFrameMappedMutex);
	newFrameMappedSignal.wait(lock);
	newFrameMappedSignal.wait(lock);

	usleep(200000);
	printf("Done Finalizing Graph\n");
}


void lsdSystem::constraintSearchThreadLoop()
{
	printf("started constraint search thread!\n");

	boost::unique_lock<boost::mutex> lock(newKeyFrameMutex);
	int failedToRetrack = 0;

	while(keepRunning)
	{
		if(newKeyFrames.size() == 0)
		{
			lock.unlock();
			keyFrameGraph->keyframesForRetrackMutex.lock();
			bool doneSomething = false;
			if(keyFrameGraph->keyframesForRetrack.size() > 10)
			{
				std::deque<Frame*>::iterator toReTrack = keyFrameGraph->keyframesForRetrack.begin()
						+ (rand() % (keyFrameGraph->keyframesForRetrack.size()/3));
				Frame* toReTrackFrame = *toReTrack;
				keyFrameGraph->keyframesForRetrack.erase(toReTrack);
				keyFrameGraph->keyframesForRetrack.push_back(toReTrackFrame);

				keyFrameGraph->keyframesForRetrackMutex.unlock();

				int found = findConstraintsForNewKeyFrames(toReTrackFrame, false, false, 2.0);
				if(found == 0)
					failedToRetrack++;
				else
					failedToRetrack=0;

				if(failedToRetrack < (int)keyFrameGraph->keyframesForRetrack.size() - 5)
					doneSomething = true;
			}
			else
				keyFrameGraph->keyframesForRetrackMutex.lock();

			lock.lock();

			if(!doneSomething)
				newKeyFrameCreatedSignal.timed_wait(lock, boost::posix_time::milliseconds(500));
		}

		else
		{
			Frame *newKF = newKeyFrames.front();
			newKeyFrames.pop_front();
			lock.unlock();

			struct timeval tv_start, tv_end;
			gettimeofday(&tv_start, NULL);

			findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
			failedToRetrack = 0;
			gettimeofday(&tv_end, NULL);
			msFindConstraintsItaration = 0.9*msFindConstraintsItaration +
					0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f +
						 (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);

			nFindConstraintsItaration++;

			FrameMemory::getInstance().pruneActiveFrames();
			lock.lock();
		}

		if(doFullReConstraintTrack)
		{
			lock.unlock();
			printf("Optizing full map!\n");

			int added = 0;
			for(unsigned int i = 0; i < keyFrameGraph->keyframesAll.size(); ++i)
			{
				if(keyFrameGraph->keyframesAll[i]->pose->isInGraph)
					added += findConstraintsForNewKeyFrames(keyFrameGraph->keyframesAll[i],
															false, false, 1.0);
			}

			printf("Done optizing full map! Added %d contraints. \n", added);

			doFullReConstraintTrack = false;

			lastNumConstraintsAddedOnFullRetrack = added;
			lock.lock();
		}


	}
	printf("Exited constraint search thread \n");

}

void lsdSystem::optimizationThreadLoop()
{
	printf("started optimization thread \n");

	while(keepRunning)
	{
		boost::unique_lock<boost::mutex> lock(newConstraintMutex);

		if(!newConstraintAdded)
			newConstraintCreatedSignal.timed_wait(lock, boost::posix_time::milliseconds(2000));
		newConstraintAdded = false;
		lock.unlock();

		if(doFinalOptimization)
		{
			printf("doing final optimization iteration\n");
			optimizationIteration(50, 0.001);
			doFinalOptimization = false;
		}

		while(optimizationIteration(5, 0.02));
	}

	printf("Exited optimization thread\n");
}


void lsdSystem::finishCurrentKeyFrame()
{
	printf("FINALIZING KF %d\n", currentKeyFrame->id());

	map->finalizeKeyFrame();

	if(SLAMEnabled)
	{
		mappingTrackingReference->importFrame(currentKeyFrame.get());
		currentKeyFrame->setPermaRef(mappingTrackingReference);
		mappingTrackingReference->invalidate();

		if(currentKeyFrame->idxInKeyframes < 0)
		{
			keyFrameGraph->keyframesAllMutex.lock();
			currentKeyFrame->idxInKeyframes = keyFrameGraph->keyframesAll.size();
			keyFrameGraph->keyframesAll.push_back(currentKeyFrame.get());
			keyFrameGraph->totalPoints += currentKeyFrame->numPoints;
			keyFrameGraph->totalVertices++;
			keyFrameGraph->keyframesAllMutex.unlock();
		}
	}

	if(imgIO != 0)
		imgIO->publishKeyframe(currentKeyFrame.get());

}

void lsdSystem::discardCurrentKeyFrame()
{
	printf("DISCARDING KF %d\n", currentKeyFrame->id());

	if(currentKeyFrame->idxInKeyframes >= 0)
	{
		printf("WARRING: try to discard a KF that has already been added to the graph... finalizing instead.\n");
		finishCurrentKeyFrame();
		return;
	}

	map->invalidate();

	keyFrameGraph->idToKeyFrameMutex.lock();
	keyFrameGraph->idToKeyFrame.erase(currentKeyFrame->id());
	keyFrameGraph->idToKeyFrameMutex.unlock();
}


void lsdSystem::createNewCurrentKeyFrame(std::shared_ptr<Frame> newKeyFrameCandidate)
{
	printf("CREATE NEW KF %d from %d\n", newKeyFrameCandidate->id(), currentKeyFrame->id());

	if(SLAMEnabled)
	{
		keyFrameGraph->idToKeyFrameMutex.lock();
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(newKeyFrameCandidate->id(), newKeyFrameCandidate));
		keyFrameGraph->idToKeyFrameMutex.unlock();
	}

	map->createKeyFrame(newKeyFrameCandidate.get());

	currentKeyFrameMutex.lock();
	currentKeyFrame = newKeyFrameCandidate;
	currentKeyFrameMutex.unlock();
}

void lsdSystem::changeKeyFrame(bool noCreate, bool force, float maxScore)
{
	Frame* newReferenceKF = 0;
	std::shared_ptr<Frame> newKeyFrameCandidate = latestTrackedFrame;
	if(doKFReActivation && SLAMEnabled)
	{
		struct timeval tv_start, tv_end;
		gettimeofday(&tv_start, nullptr);
		newReferenceKF = trackableKeyFrameSearch->findRePositionCandidate(newKeyFrameCandidate.get(), maxScore);
		gettimeofday(&tv_end, nullptr);
		msFindReferences = 0.9*msFindReferences + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f
													   + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
		nFindReferences++;
	}

	if(newReferenceKF != 0)
		loadNewCurrentKeyFrame(newReferenceKF);
	else
	{
		if(force)
		{
			if(noCreate)
			{
				trackingIsGood = false;
				nextRelocIdx = -1;
				printf("mapping is disabled & moved outside of known map. Starting Relocalizer!\n");
			}
			else
				createNewCurrentKeyFrame(newKeyFrameCandidate);
		}
	}

	createNewKeyFrame = false;

}


bool lsdSystem::updateKeyFrame()
{
	std::shared_ptr<Frame> reference = nullptr;
	std::deque< std::shared_ptr<Frame> > references;

	unmappedTrackingFramesMutex.lock();

	while(unmappedTrackingFrames.size() > 0 &&
		  (!unmappedTrackingFrames.front()->hasTrackingParent() ||
		   unmappedTrackingFrames.front()->getTrackingParent() != currentKeyFrame.get()))
	{
		unmappedTrackingFrames.front()->clear_refPixWasGood();
		unmappedTrackingFrames.pop_front();
	}

	if(unmappedTrackingFrames.size() > 0)
	{
		for(unsigned int i = 0; i < unmappedTrackingFrames.size(); i++)
			references.push_back(unmappedTrackingFrames[i]);

		std::shared_ptr<Frame> popped = unmappedTrackingFrames.front();
		unmappedTrackingFrames.pop_front();
		unmappedTrackingFramesMutex.unlock();

		printf("MAPPING %d on %d to %d (%d frames)\n", currentKeyFrame->id(),
			   references.front()->id(), reference.back()->id(), (int)references.size());

		map->updateKeyframe(reference);

		popped->clear_refPixelWasGood();
		references.clear();
	}

	else
	{
		unmappedTrackingFramesMutex.unlock();
		return false;
	}

	if(imgIO != 0 && currentKeyFrame != 0)
		imgIO->publishKeyframe(currentKeyFrame.get());

	return true;
}


void lsdSystem::addTimingSamples()
{
	map->addTimingSample();
	struct timeval now;
	gettimeofday(&now, nullptr);
	float sPassed = ((now.tv_sec-lastHzUpdate.tv_sec) + (now.tv_usec-lastHzUpdate.tv_usec)/1000000.0f);

}


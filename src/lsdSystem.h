#ifndef __LSDSYSTEM_H__
#define __LSDSYSTEM_H__


#include <eigen/Eigen/Dense>

#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include "util/settings.h"

#include "opencv2/core/core.hpp"

#include "util/SophusUtil.h"

#include "Tracking/Relocalizer.h"


namespace lsd_slam
{

class TrackingReference;
class KeyFrameGraph;
class SE3Tracker;
class Sim3Tracker;
class DepthMap;
class Frame;
class ImgIO;
class TrackableKeyFrameSearch;
class FramePoseStruct;
struct KFConstraintStruct;

typedef Eigen::Matrix<float, 7, 7> Matrix7x7;


class lsdSystem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int width;
    int height;
    Eigen::Matrix3f K;
    const bool SLAMEnabled;

    bool trackingIsGood;

    lsdSystem(int w, int h, Eigen::Matrix3f K, bool enbleSLAM = true);
    lsdSystem(const lsdSystem&) = delete;
    lsdSystem& operator=(const lsdSystem&) = delete;
    ~lsdSystem();

	void randomInit(uchar* image, double timeStamp, int id);
	void gtDepthInit(uchar* image, float* depth, double timeStamp, int id);

	// tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
	void trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp);

	// finalizes the system, i.e. blocks and does all remaining loop-closures etc.
	void finalize();

	/** Does an offline optimization step. */
	void optimizeGraph();

	inline Frame* getCurrentKeyframe() {return currentKeyFrame.get();}	// not thread-safe!

	/** Returns the current pose estimate. */
	SE3 getCurrentPoseEstimate();

	/** Sets the visualization where point clouds and camera poses will be sent to. */
	void setVisualization(std::shared_ptr<ImgIO> IOWrapper);

	void requestDepthMapScreenshot(const std::string& filename);

	bool doMappingIteration();

	int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	bool optimizationIteration(int itsPerTry, float minChange);

	void publishKeyframeGraph();

	std::vector<FramePoseStruct*, Eigen::aligned_allocator<lsd_slam::FramePoseStruct*> > getAllPoses();



	float msTrackFrame, msOptimizationIteration, msFindConstraintsItaration, msFindReferences;
	int nTrackFrame, nOptimizationIteration, nFindConstraintsItaration, nFindReferences;
	float nAvgTrackFrame, nAvgOptimizationIteration, nAvgFindConstraintsItaration, nAvgFindReferences;
	struct timeval lastHzUpdate;

private:
	TrackingReference *trackingReference;    //tracking reference for current keyframe. only used by tracking
	SE3Tracker *tracker;

	DepthMap *map;
	TrackingReference *mappingTrackingReference;

	std::vector<Frame*> KFForReloc;
	int nextRelocIdx;
	std::shared_ptr<Frame> latestFrameTriedForReloc;

	TrackableKeyFrameSearch *trackableKeyFrameSearch;
	Sim3Tracker   *constraintTracker;
	SE3Tracker    *constraintSE3Tracker;
	TrackingReference *newKFTrackingReference;
	TrackingReference *candidateTrackingReference;


	float trackingLastResidual;
	float trackingLastUsage;
	float trackingLastGoodPerBad;
	float trackingLastGoodPerTotal;

	int lastNumConstraintsAddedOnFullRetrack;
	bool doFinalOptimization;
	float lastTrackingClosenessScore;

	boost::condition_variable  newFrameMappedSignal;
	boost::mutex newFrameMappedMutex;

	Relocalizer relocalizer;


	std::shared_ptr<ImgIO> imgIO;
	KeyFrameGraph *keyFrameGraph;


	std::shared_ptr<Frame> latestTrackedFrame;
	bool createNewKeyFrame;


	std::deque<std::shared_ptr<Frame> >  unmappedTrackingFrames;
	boost::mutex unmappedTrackingFramesMutex;
	boost::condition_variable  unmappedTrackingFramesSignal;


	std::deque<Frame*> newKeyFrames;
	boost::mutex newKeyFrameMutex;
	boost::condition_variable newKeyFrameCreatedSignal;


	std::shared_ptr<Frame>  currentKeyFrame;
	std::shared_ptr<Frame>  trackingReferenceFrameSharedPT;
	boost::mutex currentKeyFrameMutex;

	boost::thread threadMapping;
	boost::thread threadContraintSearch;
	boost::thread threadOptimization;
	bool keepRunning;

	bool newConstraintAdded;
	boost::mutex newConstraintMutex;
	boost::condition_variable newConstraintCreatedSignal;
	boost::mutex g2oGraphAccessMutex;


	// optimization merging. SET in Optimization, merged in Mapping.
	bool haveUnmergedOptimizationOffset;

	boost::shared_mutex poseConsistencyMutex;

	void mergeOptimizationOffset();

	void mappingThreadLoop();

	void finishCurrentKeyFrame();
	void discardCurrentKeyFrame();

	void changeKeyFrame(bool noCreate, bool force, float maxScore);
	void createNewCurrentKeyFrame(std::shared_ptr<Frame> newKeyFrameCandidate);
	void loadNewCurrentKeyFrame(Frame *keyFrameToLoad);

	bool updateKeyFrame();
	void addTimingSamples();
	void takeRelocalizeResult();

	void constraintSearchThreadLoop();

	float tryTrackSim3(
			TrackingReference* A, TrackingReference* B,
			int lvlStart, int lvlEnd,
			bool useSSE,
			Sim3 &AtoB, Sim3 &BtoA,
			KFConstraintStruct* e1=0, KFConstraintStruct* e2=0);

	void testConstraint(
			Frame* candidate,
			KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
			Sim3 candidateToFrame_initialEstimate,
			float strictness);

	void optimizationThreadLoop();

};


} //lsd_slam



#endif // __LSDSYSTEM_H__

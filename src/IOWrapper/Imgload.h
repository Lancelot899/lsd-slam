#ifndef __IMGLOAD_H__
#define __IMGLOAD_H__

#include <opencv2/opencv.hpp>

namespace lsd_slam {

class Frame;
class KeyFrameGraph;


class ImgIO {
    void publishKeyframeGraph(KeyFrameGraph *graph);
    void publishKeyframe(Frame *kf);

};



} // lsd_slam


#endif // __IMGLOAD_H__

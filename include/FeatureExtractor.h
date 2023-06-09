#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <vector>
#include <opencv2/opencv.hpp>

#define FEATURE_EXTRACTOR_BASE

namespace Fvins{

class FeatureExtractor;

class FeatureExtractor FEATURE_EXTRACTOR_BASE
{
 public:
  FeatureExtractor(){}
  virtual ~FeatureExtractor() = default;
  virtual bool operator()() {return false;}
  virtual bool operator()(cv::InputArray image, cv::InputArray mask,
                          std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors) {return false;}
  virtual bool operator()(cv::InputArray image, cv::InputArray mask, 
                          std::vector<cv::Point2d> pts) {return false;}

};  

} // namespace Fvins
#endif //FEATURE_EXTRACTOR_H

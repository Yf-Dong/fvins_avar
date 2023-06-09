#ifndef FEATURE_DESCRIPTOR_H
#define FEATURE_DESCRIPTOR_H

#include <vector>
#include <opencv2/opencv.hpp>

#define DESCRIPTOR_BASE

namespace Fvins {

DESCRIPTOR_BASE class FeatureDescriptor 
{
 public:
  FeatureDescriptor() {}
  virtual ~FeatureDescriptor() = default;
  virtual bool operator() () const {return false;}
  virtual bool operator() (const cv::Mat &img, std::vector<cv::KeyPoint> &keypoins, 
                           cv::Mat &descriptors) const {return false;}

};

} //namespave Fvins

#endif // FEATURE_DESCRIPTOR_H
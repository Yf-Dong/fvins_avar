#include "FeatureDescriptor.h"
#include "Avar.h"

namespace Fvins
{

class ORBFeatureDescriptor : public FeatureDescriptor
{
 public:
  ORBFeatureDescriptor();
  bool operator() (const cv::Mat &img, std::vector<cv::KeyPoint> &keypoins, 
                         cv::Mat &descriptors) const override;
 protected:
  std::vector<cv::Point>  mPattern;
  float                   mfFactorPi;
  void computeOrbDescriptor(const cv::KeyPoint &kpt, const cv::Mat &img, uchar *desc) const;
};

ORBFeatureDescriptor::ORBFeatureDescriptor() 
{
  // descriptor pattern;
  mfFactorPi = (float)(CV_PI/180.f);
  const cv::Point* pPattern = (const cv::Point*) avar.getPtr("descriptor_patter");
  const int npoints = 512;
  mPattern.reserve(512);
  std::copy(pPattern, pPattern + npoints, std::back_inserter(mPattern));
}

bool ORBFeatureDescriptor::operator() (const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, 
                                             cv::Mat &descriptors) const
{
  descriptors = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
  for (size_t i = 0; i < keypoints.size(); ++i) {
    computeOrbDescriptor(keypoints[i], img, descriptors.ptr((int)i));
  }
  return true;
} 

void ORBFeatureDescriptor::computeOrbDescriptor(const cv::KeyPoint &kpt, const cv::Mat &img, uchar *desc) const
{

  const cv::Point* pPattern = &mPattern[0];
  float angle = (float)kpt.angle * mfFactorPi;
  float cosa  = (float)cos(angle), sina = (float)sin(angle);
  
  const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
  const int step      = (int)img.step;
  auto getPixel = [&](const int &idx) 
                    {return center[(cvRound(pPattern[idx].y * cosa + pPattern[idx].x * sina)) * step 
                    + cvRound(- pPattern[idx].y * sina + pPattern[idx].x * cosa)];};


  for (int i = 0; i < 32; ++i, pPattern += 16) {
    int t0, t1, val;
    t0 = getPixel(0); t1= getPixel(1);
    val = t0 < t1;
    for (int i = 1;i < 8;++i) {
      t0 = getPixel(i * 2); t1 = getPixel(i * 2 + 1);
      val |= (t0 < t1) << i;
    }
    desc[i] = (uchar)val;
  }
}


// REGISTER_FEATUREDESCRIPTOR(ORBFeatureDescriptor, orb)
REGISTER_PLUGIN(FeatureDescriptor, ORBFeatureDescriptor, orb)
}
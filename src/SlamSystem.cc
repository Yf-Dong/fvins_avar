#include "SlamSystem.h"

SlamSystem::SlamSystem()
{

  // Fvins::FeatureDescriptorPtr ptr = Fvins::DescriptorFactory::create("orb");
  
  mopOrbPtr = Plugin(Fvins::FeatureExtractor)::create("orb");
  cv::Mat img;
  cv::imread("R-C.png").convertTo(img,CV_8UC1);
  cv::imshow("t", img);
  cv::waitKey(0);
  std::vector<cv::KeyPoint> kp;
  cv::Mat desc = cv::Mat();
  mopOrbPtr->operator()(img, cv::Mat(), kp, desc);
  // (*mopOrbPtr)(img, cv::Mat(), kp, desc);
}
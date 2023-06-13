#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include "FeatureExtractor.h"
#include "Avar.hpp"

class SlamSystem
{
 public:
  typedef std::shared_ptr<Fvins::FeatureExtractor> FeatureExtractorPtr;
  SlamSystem();

 private:
  FeatureExtractorPtr mopOrbPtr;
};

#endif
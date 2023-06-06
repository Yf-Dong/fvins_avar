#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include "FeatureExtractor.h"
#include "FeatureDescriptor.h"

class SlamSystem
{
 public:
  SlamSystem();
 private:
  Fvins::FeatureExtractorPtr mopOrbPtr;
};

#endif
#include "FeatureExtractor.h"
#include "FeatureDescriptor.h"
#include <iostream>
#include <thread>
#include <list>
#include "Avar.h"

namespace Fvins 
{

class ExtractorNode
{
 public:
  ExtractorNode() : bNoMore(false) {}
  void divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

  std::vector<cv::KeyPoint> kPts;
  cv::Point2i UL, UR, BL, BR;
  std::list<ExtractorNode>::iterator lit;
  bool bNoMore;
};

void ExtractorNode::divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
{
  const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
  const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

  n1.UL = UL;
  n1.UR = cv::Point2i(UL.x + halfX, UL.y);
  n1.BL = cv::Point2i(UL.x, UL.y + halfY);
  n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
  n1.kPts.reserve(kPts.size());

  n2.UL = n1.UR;
  n2.UR = UR;
  n2.BL = n1.BR;
  n2.BR = cv::Point2i(UR.x, UL.y + halfY);
  n2.kPts.reserve(kPts.size());

  n3.UL = n1.BL;
  n3.UR = n1.BR;
  n3.BL = BL;
  n3.BR = cv::Point2i(n1.BR.x, BL.y);
  n3.kPts.reserve(kPts.size());

  n4.UL = n3.UR;
  n4.UR = n2.BR;
  n4.BL = n3.BR;
  n4.BR = BR;
  n4.kPts.reserve(kPts.size());

  for (size_t i = 0; i < kPts.size(); ++i) {
    const cv::KeyPoint &kp = kPts[i];
    if (kp.pt.x < n1.UR.x) {
      if(kp.pt.y < n1.BL.y) {n1.kPts.push_back(kp);}
      else                  {n3.kPts.push_back(kp);}
    } else if (kp.pt.y < n1.BR.y) {n2.kPts.push_back(kp);}
    else {n4.kPts.push_back(kp);}
  }

  if(n1.kPts.size() == 1) n1.bNoMore = true;
  if(n2.kPts.size() == 1) n2.bNoMore = true;
  if(n3.kPts.size() == 1) n3.bNoMore = true;
  if(n4.kPts.size() == 1) n4.bNoMore = true;

}

class OrbExtractor: public FeatureExtractor
{
 public:
  typedef std::shared_ptr<Fvins::FeatureDescriptor> FeatureDescriptorPtr;

 public:
  OrbExtractor();
  virtual ~OrbExtractor() = default;
  bool operator() ( cv::InputArray image, cv::InputArray mask,std::vector<cv::KeyPoint>& keypoints, 
                    cv::OutputArray descriptors) override;
  void setNFeatures(const int &nfeatures) {mnFeatures = nfeatures;}
 
 protected:
  void initPyramid();
  void initExtractor();
  void computePyramid(cv::Mat image);
  void computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeyPoints);
  void computeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
  float IC_Angle(const cv::Mat& image, const cv::Point2f &pt);
  std::vector<cv::KeyPoint> distributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, 
                                              const int &minX, const int &maxX, const int &minY, const int &maxY,
                                              const int &N, const int &level);

  // void ComputeKeyPointsOctTree(vector<vector<cv::KeyPoint> >& allKeypoints);

  int mnFeatures;
  int mnLevels;
  int miInithFast;
  int minThFast;
  const int miPatchSize;
  const int miHalfPatchSize;
  const int miEgdeTHreshold;
  float mfScaleFactor;
  std::vector<float>      mvScaleFactor;
  std::vector<float>      mvLevelSigma2;
  std::vector<float>      mvInvScaleFactor;
  std::vector<float>      mvInvLevelSigma2;
  std::vector<cv::Mat>    mvImagePyramid;

  std::vector<int>        mnFeaturesPerLevel;
  std::vector<int>        mUmax;

  FeatureDescriptorPtr    mpFeatureDescriptor;
}; 

OrbExtractor::OrbExtractor() :
  mnFeatures          (avar.call<int>   ("ORBextractor.nFeatures")),
  mnLevels            (avar.call<int>   ("ORBextractor.nLevels")),
  miInithFast         (avar.call<int>   ("ORBextractor.iniThFAST")),
  minThFast           (avar.call<int>   ("ORBextractor.minThFAST")),
  mfScaleFactor       (avar.call<float> ("ORBextractor.scaleFactor")),
  miHalfPatchSize     (avar.call<int>   ("ORBextractor.halfPatchSize")),
  miEgdeTHreshold     (avar.call<int>   ("ORBextractor.EdgeThreshold")),
  miPatchSize         (avar.call<int>   ("ORBextractor.patchSize")),
  mpFeatureDescriptor (Plugin(FeatureDescriptor)::create("orb"))
{
  initPyramid();
  initExtractor();
}

void OrbExtractor::initPyramid()
{
  // factor and sigma^2
  mvScaleFactor.resize(mnLevels);
  mvLevelSigma2.resize(mnLevels);
  mvInvScaleFactor.resize(mnLevels);
  mvInvLevelSigma2.resize(mnLevels);
  mvImagePyramid.resize(mnLevels);
  mvScaleFactor[0]    = 1.0f;
  mvLevelSigma2[0]    = 1.0f;
  mvInvScaleFactor[0] = 1.0f;
  mvInvLevelSigma2[0] = 1.0f;

  for (int i=1; i < mnLevels; ++i) {
    mvScaleFactor[i]    = mvScaleFactor[i-1] * mfScaleFactor;
    mvLevelSigma2[i]    = mvScaleFactor[i] * mvScaleFactor[i]; 
    mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
    mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
  }

  //Feature per Level
  mnFeaturesPerLevel.resize(mnLevels);
  float factor = 1.0f / mfScaleFactor;
  float nDesiredFeaturePerScale = mnFeatures* (1 - factor) / (1 - (float)pow((double)factor, (double)mnLevels));
  int sumFeatures = 0;
  for (int level = 0; level < mnLevels-1; level++) {
    mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturePerScale);
    // std::cout << "level: " << level << "  features: " << mnFeaturesPerLevel[level] << std::endl;
    sumFeatures += mnFeaturesPerLevel[level];
    nDesiredFeaturePerScale *= factor;
  }
  mnFeaturesPerLevel[mnLevels-1] = std::max(mnFeatures - sumFeatures, 0);
  
}

void OrbExtractor::initExtractor() 
{ 
  //compute the orientation pattern
  mUmax.resize(miHalfPatchSize + 1);
  float hh = miHalfPatchSize * miHalfPatchSize;
  int vmax = cvCeil(miHalfPatchSize * sqrt(2.f) / 2);
  // FIXME: Is it Better.
  for (int v = 0; v < vmax; ++v) {
    mUmax[v] = cvRound(sqrt(hh - v * v));
    // std::cout << "mUmax1: " << mUmax[v] << std::endl;
  }  
  int v0 = 0;
  for (int v = miHalfPatchSize; v >= vmax; --v) {
    while(mUmax[v0] == mUmax[v0 + 1]) {++v0;}
    mUmax[v] = v0;
    // std::cout << "mUmax2: " << mUmax[v]  << std::endl;
    ++v0;
  }

}

bool OrbExtractor::operator() ( cv::InputArray _image, cv::InputArray _mask,
                                std::vector<cv::KeyPoint>& _keypoints, 
                                cv::OutputArray _descriptors) 
{
  if (_image.empty()) {
    return false;
  }
  cv::Mat image;
  // _image.getMat().convertTo(image, cv::COLOR_RGB2GRAY);
  cv::cvtColor(_image, image, cv::COLOR_RGB2GRAY);
  assert(image.type() == CV_8UC1);
  computePyramid(image);
  std::vector<std::vector<cv::KeyPoint>> allKeyPoints;
  computeKeyPointsOctTree(allKeyPoints);
  
  //FIXME.
  // if (_descriptors.empty()) {
  //   for (int level = 0; level < mnLevels; ++level) {
  //     std::vector<cv::KeyPoint>& keypoints = allKeyPoints[level];
  //     if (level) {
  //       float scale = mvScaleFactor[level];
  //       for (auto &kpt:keypoints) {
  //         kpt.pt *= scale;
  //       }
  //     }
  //     _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
  //   }
  //   return true;
  // }

  //descriptors
  cv::Mat descriptors;
  int nKeypoints = 0;
  for (int level = 0; level < mnLevels; ++level) {
    nKeypoints += (int)allKeyPoints[level].size();
  }
  if (!nKeypoints) {
    _descriptors.release();
  } else {
    // std::cout << "nKeypoints: " << nKeypoints << std::endl;
    _descriptors.create(nKeypoints, 32, CV_8U);
    descriptors = _descriptors.getMat();
  }
  _keypoints.clear();
  _keypoints.reserve(nKeypoints);

  int offset = 0;
  for (int level = 0; level < mnLevels; ++level) {
    std::vector<cv::KeyPoint>& keypoints = allKeyPoints[level];
    int nkeypointsLevel = (int)keypoints.size();

    if (!nkeypointsLevel) continue;
    cv::Mat workingMat = mvImagePyramid[level];
    GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
    cv::Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
    (*mpFeatureDescriptor)(workingMat, keypoints, desc);
    offset += nkeypointsLevel;

    if (level) {
      float scale = mvScaleFactor[level];
      for (auto &kpt:keypoints) {
        kpt.pt *= scale;
      }
    }
    _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());

  }

  cv::Mat img = mvImagePyramid[0];
  for (auto &kpt: _keypoints) {
    cv::circle(img, kpt.pt, 2, cv::Scalar(0,0,255),-1);
  }
  cv::imshow("t", img);
  cv::waitKey(0);
  std::cout << _descriptors.getMat() << std::endl;
  return true;
} 

void OrbExtractor::computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeyPoints) 
{
  allKeyPoints.resize(mnLevels);

  std::vector<std::thread> threads;
  for (int level = 0; level < mnLevels; ++level) {
    // FastExtractor(level);
    cv::Mat &image = mvImagePyramid[level];
    threads.emplace_back(std::thread([level, &image, &allKeyPoints, this]() {
      const float W = 30;
      const int minBorderX = miEgdeTHreshold - 3;
      const int minBorderY = minBorderX;
      const int maxBorderX = image.cols - miEgdeTHreshold + 3;
      const int maxBorderY = image.rows - miEgdeTHreshold + 3;
  
      std::vector<cv::KeyPoint> vToDistributeKeys; 
      vToDistributeKeys.reserve(mnFeatures*10);
  
      const float width  = (maxBorderX - minBorderX);
      const float height = (maxBorderY - minBorderY);
      const int nCols = width / W;
      const int nRows = height / W;
      const int wCell = ceil(width / nCols);
      const int hCell = ceil(height / nRows);
  
      for (int i = 0; i < nRows; ++i) {
        const float iniY =minBorderY+i*hCell;
        float maxY = iniY+hCell+6;
  
        if(iniY >= maxBorderY-3) continue;
        if(maxY >= maxBorderY) maxY=maxBorderY;
        for (int j = 0; j < nCols; ++j) {
          
          const float iniX = minBorderX + j * wCell;
          float maxX = iniX + wCell + 6;
          if(iniX >= maxBorderX - 6) continue;
          if(maxX >= maxBorderX) maxX = maxBorderX;
          
          std::vector<cv::KeyPoint> vKeysCell;
          cv::FAST(image.rowRange(iniY, maxY).colRange(iniX, maxX), vKeysCell, miInithFast, true);
          if (vKeysCell.empty()) {
            cv::FAST(image.rowRange(iniY, maxY).colRange(iniX, maxX), vKeysCell, minThFast, true);
          }
  
          if (!vKeysCell.empty()) {
            for (auto &kpt: vKeysCell) {
              kpt.pt.x += j * wCell;
              kpt.pt.y += i * hCell;
              vToDistributeKeys.emplace_back(kpt);
            }
          }
        }
      }
      
      // allKeyPoints[level] = vToDistributeKeys;

      std::vector<cv::KeyPoint> &keyPoints = allKeyPoints[level];
      keyPoints.reserve(mnFeatures);
      keyPoints = distributeOctTree( vToDistributeKeys, minBorderX, maxBorderX, minBorderY,
                                     maxBorderY, mnFeaturesPerLevel[level], level);
      const int scaledPatchSize = miPatchSize * mvScaleFactor[level];
      for(auto &kpt:keyPoints) {
        kpt.pt.x += minBorderX;
        kpt.pt.y += minBorderY;
        kpt.octave = level;
        kpt.size = scaledPatchSize;
      }

    }));
  }
  for (auto &t: threads) { t.join(); }

  // std::cout << "thread done." << std::endl;
  // for (int i = 0;i < mnLevels; ++i) {
  //   cv::Mat img = mvImagePyramid[i];
  //   for (auto &kpt: allKeyPoints[i]) {
  //     cv::circle(img, kpt.pt, 2, cv::Scalar(0,0,255),-1);
  //   }
  //   cv::imshow("t", img);
  //   cv::waitKey(0);
  // }
  // std::cout << "done" << std::endl;

  for (int level = 0; level < mnLevels; ++level) {
    computeOrientation(mvImagePyramid[level], allKeyPoints[level]);
  }

}

void OrbExtractor::computeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints)
{
  for(std::vector<cv::KeyPoint>::iterator kpt = keypoints.begin(); kpt != keypoints.end(); ++kpt) {
    kpt->angle = IC_Angle(image, kpt->pt);
    // std::cout << "angle: " << kpt->angle << std::endl;
  }
}

float OrbExtractor::IC_Angle(const cv::Mat& image, const cv::Point2f &pt) {
  int m_01 = 0, m_10 = 0;
  const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));
  
  for (int u = -miHalfPatchSize; u <= miHalfPatchSize; ++u) {
    m_10 += u * center[u];
  }

  int step = (int)image.step1();
  for (int v = 1; v <= miHalfPatchSize; ++v) {
    int v_sum = 0;
    const uchar* left_plus  = &center[v * step];
    const uchar* left_minus = &center[- v * step];
    int d = mUmax[v];
    for (int u = -d; u <= d; ++u) {
      int val_plus = left_plus[u], val_minus = left_minus[u];
      m_10 += u * (val_plus + val_minus);
      v_sum += (val_plus - val_minus);
    }
    m_01 += v * v_sum;
  }
  // std::cout << "m_10: " << m_10 << "  m_01: " << m_01 << std::endl;
  return cv::fastAtan2((float)m_01, (float)m_10);
}

void OrbExtractor::computePyramid(cv::Mat image) 
{
  // The border is useless here.
  for (int level = 0; level < mnLevels; ++level) {
    float scale = mvInvScaleFactor[level];
    cv::Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));
    // cv::Size wholeSize(sz.width + miEgdeTHreshold * 2, sz.height + miEgdeTHreshold * 2);
    // cv::Mat temp(wholeSize, image.type());
    // mvImagePyramid[level] = temp(cv::Rect(miEgdeTHreshold, miEgdeTHreshold, sz.width, sz.height));
  
    if (level != 0) {
      cv::resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
      // cv::copyMakeBorder(mvImagePyramid[level], temp, miEgdeTHreshold, miEgdeTHreshold, miEgdeTHreshold,
                        //  miEgdeTHreshold, cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED); 
      // cv::imshow("temp", temp);
      // cv::imshow("pyr", mvImagePyramid[level]);
      // cv::waitKey(0);
    } else {
      mvImagePyramid[level] = image;
      // cv::copyMakeBorder(image, temp, miEgdeTHreshold, miEgdeTHreshold, miEgdeTHreshold,
                        //  miEgdeTHreshold, cv::BORDER_REFLECT_101); 
    }
  }
}

std::vector<cv::KeyPoint> OrbExtractor::distributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, 
                                            const int &minX, const int &maxX, const int &minY, const int &maxY,
                                            const int &nFeatures, const int &level) 
{
  //FIXME: Only X > Y;
  // std::cout << "nFeatures: " << nFeatures << std::endl;
  const int nInit = roundf(static_cast<float> (maxX - minX) / (maxY - minY));
  const float hX = static_cast<float> (maxX - minX) / nInit;
  std::list<ExtractorNode> lNodes;
  std::vector<ExtractorNode *> vpIniNodes;
  vpIniNodes.resize(nInit);

  for (int i = 0; i < nInit; ++i) {
    ExtractorNode ni;
    ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
    ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
    ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
    ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
    ni.kPts.reserve(vToDistributeKeys.size());

    lNodes.push_back(ni);
    vpIniNodes[i] = &lNodes.back();
  }

  for(const auto &kp:vToDistributeKeys) {
    vpIniNodes[kp.pt.x / hX] -> kPts.push_back(kp);
  }
  std::list<ExtractorNode>::iterator lit = lNodes.begin();
  while(lit != lNodes.end()) {
    if (lit->kPts.size() == 1) {
      lit->bNoMore = true;
      ++lit;
    } else if (lit->kPts.empty()) {lit = lNodes.erase(lit);}
    else lit++;
  }
  // std::cout << "init lnode.size(): " << lNodes.size() << std::endl;

  //Init the lNodes, but only the condition that X > Y;
  //**************************************************

  bool bFinish = false;
  int iteration = 0;
  std::vector<std::pair<int, ExtractorNode *>> vSizeAndPointerToNode;
  vSizeAndPointerToNode.reserve(lNodes.size() * 4);

  while (!bFinish) {
    ++iteration;
    int prevSize = lNodes.size();
    lit = lNodes.begin();
    int nToExpand = 0;
    vSizeAndPointerToNode.clear();

    while (lit != lNodes.end()) {
      if (lit->bNoMore) {
        lit++; continue;
      } else {
        ExtractorNode n1, n2, n3, n4;
        lit->divideNode(n1, n2, n3, n4);

        if(n1.kPts.size() > 0) {
          lNodes.push_front(n1);
          if (n1.kPts.size() > 1) {
            ++nToExpand;
            vSizeAndPointerToNode.push_back(std::make_pair(n1.kPts.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
      
        if(n2.kPts.size() > 0) {
          lNodes.push_front(n2);
          if (n2.kPts.size() > 1) {
            ++nToExpand;
            vSizeAndPointerToNode.push_back(std::make_pair(n2.kPts.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }

        if(n3.kPts.size() > 0) {
          lNodes.push_front(n3);
          if (n3.kPts.size() > 1) {
            ++nToExpand;
            vSizeAndPointerToNode.push_back(std::make_pair(n3.kPts.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }

        if(n4.kPts.size() > 0) {
          lNodes.push_front(n4);
          if (n4.kPts.size() > 1) {
            ++nToExpand;
            vSizeAndPointerToNode.push_back(std::make_pair(n4.kPts.size(), &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }

        lit = lNodes.erase(lit);
        continue;
      }
    }

    if ((int)lNodes.size() >= nFeatures || (int)lNodes.size() == prevSize) {
      bFinish = true;
    } 
    //It is better to devide the node have the most key points first.
    // else if (((int)lNodes.size() + nToExpand * 3) > nFeatures) {
    else if ((nToExpand * 3) > nFeatures) {
      while (!bFinish) {
        prevSize = lNodes.size();
        std::vector<std::pair<int, ExtractorNode*>> vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
        vSizeAndPointerToNode.clear();
        
        sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
        for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; --j) {
          ExtractorNode n1, n2, n3, n4;
          vPrevSizeAndPointerToNode[j].second->divideNode(n1,n2,n3,n4);

          if(n1.kPts.size() > 0) {
            lNodes.push_front(n1);
            if (n1.kPts.size() > 1) {
              ++nToExpand;
              vSizeAndPointerToNode.push_back(std::make_pair(n1.kPts.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
        
          if(n2.kPts.size() > 0) {
            lNodes.push_front(n2);
            if (n2.kPts.size() > 1) {
              ++nToExpand;
              vSizeAndPointerToNode.push_back(std::make_pair(n2.kPts.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
  
          if(n3.kPts.size() > 0) {
            lNodes.push_front(n3);
            if (n3.kPts.size() > 1) {
              ++nToExpand;
              vSizeAndPointerToNode.push_back(std::make_pair(n3.kPts.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
  
          if(n4.kPts.size() > 0) {
            lNodes.push_front(n4);
            if (n4.kPts.size() > 1) {
              ++nToExpand;
              vSizeAndPointerToNode.push_back(std::make_pair(n4.kPts.size(), &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          } 

          lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);
          if((int)lNodes.size() >= nFeatures) break;       
        }
        
        if ((int)lNodes.size() >= nFeatures || (int)lNodes.size() == prevSize)
          bFinish = true;
      }
    }

  }

  std::vector<cv::KeyPoint> vResultKeys;
  vResultKeys.reserve(nFeatures);
  for (std::list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); ++ lit) {
    std::vector<cv::KeyPoint> &vNodeKeys = lit->kPts;
    cv::KeyPoint* pKP = &vNodeKeys[0];
    float maxResponse = pKP->response;

    for (size_t k = 1; k < vNodeKeys.size(); ++k) {
      if (vNodeKeys[k].response > maxResponse) {
        pKP = &vNodeKeys[k];
        maxResponse = vNodeKeys[k].response;
      }
    }
    vResultKeys.push_back(*pKP);
  }
  return vResultKeys;
}



// REGISTER_FEATUREEXTRACTOR(OrbExtractor, orb)
REGISTER_PLUGIN(FeatureExtractor, OrbExtractor, orb)

} //namespace Fvins


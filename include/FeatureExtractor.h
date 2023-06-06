#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#ifndef SPtr
#define SPtr std::shared_ptr
#endif
#ifndef UPtr
#define UPtr std::unique_ptr
#endif

#define FEATURE_EXTRACTOR_BASE

#define REGISTER_FEATUREEXTRACTOR(D,E) \
    extern "C" SPtr<Fvins::FeatureExtractor> createExtractor##E() {return SPtr<Fvins::FeatureExtractor>(new D());}\
    class D##E##_Register{ \
      public: D##E##_Register() {\
        MapFunctions& mf = ExtractorFactory::instance().mmFuncs;\
        mf.insert(std::pair<std::string, void*>(#E, (void *)(createExtractor##E)));\
      }\
    } D##E##_instance;

namespace Fvins{

class FeatureExtractor;
typedef SPtr<FeatureExtractor> FeatureExtractorPtr;
typedef SPtr<FeatureExtractor> (*funcCreateFeatureExtractor)();
typedef std::map<std::string, void*> MapFunctions;

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

class ExtractorFactory 
{
 public:
  ExtractorFactory(){}
  ~ExtractorFactory(){}
  static FeatureExtractorPtr create(const std::string& pluginName) ;
  static ExtractorFactory& instance();
  MapFunctions mmFuncs;
};

inline FeatureExtractorPtr ExtractorFactory::create(const std::string& pluginName)
{
  MapFunctions& mf = ExtractorFactory::instance().mmFuncs;
  MapFunctions::iterator iter = mf.find(pluginName);
  if (iter != mf.end()) {
    funcCreateFeatureExtractor func = (funcCreateFeatureExtractor) iter->second;
    return func();
  } else {
    //FIXME
    std::cout << "Can't find the Plugin." << std::endl;
    std::terminate();
    return FeatureExtractorPtr();
  }
}

inline ExtractorFactory& ExtractorFactory::instance()
{
  static UPtr<ExtractorFactory> fac(new ExtractorFactory());
  return *fac;
}

} // namespace Fvins
#endif //FEATURE_EXTRACTOR_H

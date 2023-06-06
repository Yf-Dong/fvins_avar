#ifndef FEATURE_DESCRIPTOR_H
#define FEATURE_DESCRIPTOR_H

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

#define DESCRIPTOR_BASE
#define REGISTER_FEATUREDESCRIPTOR(D,E) \
    extern"C" SPtr<Fvins::FeatureDescriptor> createDescriptor##E() \
    {return SPtr<Fvins::FeatureDescriptor>(new D());}\
    class D##E##Register {\
      public:\
        D##E##Register() {\
          Fvins::MapFunctions& mf = DescriptorFactory::instance().mmFuncs;\
          mf.insert(std::pair<std::string, void*>(#E, (void *)createDescriptor##E));\
        }\
    } D##E##_instance;


namespace Fvins {

class FeatureDescriptor;
typedef std::map<std::string, void*> MapFunctions;
typedef SPtr<FeatureDescriptor> FeatureDescriptorPtr;
typedef SPtr<FeatureDescriptor> (*funcCreateFeatureDescriptor)() ;

DESCRIPTOR_BASE class FeatureDescriptor 
{
 public:
  FeatureDescriptor() {}
  virtual ~FeatureDescriptor() = default;
  virtual bool operator() () const {return false;}
  virtual bool operator() (const cv::Mat &img, std::vector<cv::KeyPoint> &keypoins, 
                           cv::Mat &descriptors) const {return false;}

};

class DescriptorFactory
{
 public:
  DescriptorFactory(){}
  ~DescriptorFactory(){}
  
  static FeatureDescriptorPtr create(const std::string &pluginName) {
    MapFunctions &mf = DescriptorFactory::instance().mmFuncs;
    MapFunctions::iterator iter = mf.find(pluginName);
    if (iter != mf.end()) {
      funcCreateFeatureDescriptor func = funcCreateFeatureDescriptor(iter->second);
      return func();
    } else {
      std::cerr << "No ptr named \"" << pluginName << "\"" << std::endl;
      return FeatureDescriptorPtr();
    }

  }
  
  static DescriptorFactory& instance() {
    static UPtr<DescriptorFactory> fac(new DescriptorFactory());
    return *fac;
  }

  MapFunctions mmFuncs;
};

} //namespave Fvins

#endif // FEATURE_DESCRIPTOR_H
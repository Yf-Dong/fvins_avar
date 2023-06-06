#include "Avar.h"
// #include "FeatureExtractor.h"
#include "SlamSystem.h"
// #include <dlfcn.h>
// #define LIB_LOAD(file)       dlopen(file, RTLD_NOW)

int main(int argc, char ** argv) {
  if (argc < 2) {
    std::cout << "Missing the Conig file path" << std::endl
              << "Like './test /home/yf/Progs/SlamSys/vins-pp/VINS++/config/euroc_mono_imu_config.yaml'" << std::endl
              << "Will using the default Config.yaml" << std::endl;
  return 1;
  } else {
     avar.ParseFile(argv[1]);
  }
  SlamSystem test;
  // auto library = LIB_LOAD("libslam.so");
  Fvins::FeatureExtractorPtr ptr = Fvins::ExtractorFactory::create("orb");
  (*ptr)();
  return 0;
}
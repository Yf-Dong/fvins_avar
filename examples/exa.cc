#include "Avar.h"
#include "Time.h"
#include "FeatureExtractor.h"


int main(int argc, char ** argv) {

  if (argc < 2) {
    std::cout << "Missing the Conig file path" << std::endl
              << "Like './test /home/yf/Progs/SlamSys/vins-pp/VINS++/config/euroc_mono_imu_config.yaml'" << std::endl
              << "Will using the default Config.yaml" << std::endl;
  }
  
  // AVAR::Node root;
  // AVAR::LoadFile(root, argv[1]);

  avar.ParseFile(argv[1]);
  // avar.ParseFile("../config/cam0_mei.yaml");

  // Fvins::Time t = Fvins::Time::now(); 
  // std::cout << t.toSec() << std::endl;
  Eigen::MatrixXd a = avar.callMatrix<double>("Cam0.Extrinsic").toEigen();
  Eigen::MatrixXd b;
  avar.callMatrix<double>("Cam0.Extrinsic").toEigen(b);
// 
  std::cout.precision(20);
  std::cout << a << std::endl;
  std::cout << b << std::endl;
  int c; 
  avar.call<int>("Image1.Height",c);
  // 
  std::cout << c << std::endl;

  bool t;
  avar.call<bool>("Sys.FlowBack", t);
  std::cout << "bool: " << t << std::endl;

  std::string s;
  s = avar.call<std::string>("Cam0.Extrinsic");
  std::cout << s << std::endl;

  std::vector<float> vec;
  avar.call<std::vector<float>>("Cam0.Extrinsic", vec);
  for (auto &num:vec) {
    std::cout <<  num << std::endl;
  }

  // double gamma = avar.call<double>("gamma1");
  // std::cout << "gamma: " << gamma << std::endl;
  
  Fvins::FeatureExtractorPtr ptr = Fvins::ExtractorFactory::create("orb");
  (*ptr)();
  return 0;
}

#include "tstl_depth/tstl_depth.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "TSTL Depth");
  xycar::DepthEstimator depth_estimator;
  std::cout << "main started" << std::endl;
  depth_estimator.run();

  return 0;
}

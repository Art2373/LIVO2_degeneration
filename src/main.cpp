#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fusion_mapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}
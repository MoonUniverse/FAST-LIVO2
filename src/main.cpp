#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LIVMapper>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
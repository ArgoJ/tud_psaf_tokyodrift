#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_ucbridge/uc_node.hpp"

int main(int argc, char * argv[])
{
  // rclcpp
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UcNode>());
    rclcpp::shutdown();
  }
  return 0;
}

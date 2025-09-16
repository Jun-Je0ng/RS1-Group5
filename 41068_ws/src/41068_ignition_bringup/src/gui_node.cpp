#include <rclcpp/rclcpp.hpp>

class GuiNode : public rclcpp::Node {
public:
  GuiNode() : rclcpp::Node("gui_node") {
    RCLCPP_INFO(get_logger(), "GUI node started");
    // TODO: your GUI hooks / pubs-subs here
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GuiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

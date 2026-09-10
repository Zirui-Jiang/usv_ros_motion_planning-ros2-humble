// Original Costmap2DROS integration extracted from simulation_node.cpp.
// Numerical navigation runs in Python; this executable owns the Nav2 lifecycle.
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <lifecycle_msgs/msg/state.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(costmap->get_node_base_interface());
  std::thread spin_thread([&executor]() { executor.spin(); });
  int result = 0;
  try {
    if (costmap->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      costmap->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(costmap->get_logger(), "Costmap lifecycle startup failed");
      result = 1;
      rclcpp::shutdown();
    }
    spin_thread.join();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(costmap->get_logger(), "%s", error.what());
    result = 1;
    rclcpp::shutdown();
    executor.cancel();
    if (spin_thread.joinable()) {spin_thread.join();}
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}

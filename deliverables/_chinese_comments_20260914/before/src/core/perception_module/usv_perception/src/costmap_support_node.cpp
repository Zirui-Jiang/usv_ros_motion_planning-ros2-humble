// Original Costmap2DROS integration extracted from simulation_node.cpp.
// Numerical navigation runs in Python; this executable owns the Nav2 lifecycle.
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace
{
class CostmapSupportNode : public nav2_costmap_2d::Costmap2DROS
{
public:
  using nav2_costmap_2d::Costmap2DROS::Costmap2DROS;

  ~CostmapSupportNode() override
  {
    // A SIGINT during activation can leave Humble in a transition state, for
    // which its preshutdown callback does not deactivate/cleanup the node.
    // Join any partially started update thread before std::thread is destroyed.
    map_update_thread_shutdown_ = true;
    if (map_update_thread_ && map_update_thread_->joinable()) {
      map_update_thread_->join();
    }
    map_update_thread_.reset();

    // Humble declares plugin_loader_ after callback_group_, so the base
    // destructor unloads layer libraries before releasing the group's weak
    // subscription control blocks. ObstacleLayer creates those control blocks
    // in liblayers.so; destroying them after dlclose causes SIGSEGV inside
    // CallbackGroup::~CallbackGroup, even after normal lifecycle cleanup.
    // Quiesce the internal executor and release its group while the base
    // class's plugin loader (and thus those deleters) is still alive.
    executor_thread_.reset();
    executor_.reset();
    // Also cover interrupted activation, where normal cleanup did not get to
    // destroy the layer instances. Their class loader must outlive them.
    layered_costmap_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    callback_group_.reset();
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto costmap = std::make_shared<CostmapSupportNode>(rclcpp::NodeOptions());
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

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "wall_following_cpp_project/wall_follower_controller.hpp"

namespace
{
std::atomic<bool> g_stop_requested{false};

void onSignal(int)
{
  g_stop_requested.store(true);
}
}  // namespace

int main(int argc, char * argv[])
{
  // Own the signal handling so the context stays valid long enough to publish the stop Twist;
  // rclcpp's default handler would shut the context down before we could.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto node = std::make_shared<wall_following::WallFollowerController>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !g_stop_requested.load()) {
    executor.spin_some(std::chrono::milliseconds(50));
  }
  node->stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}

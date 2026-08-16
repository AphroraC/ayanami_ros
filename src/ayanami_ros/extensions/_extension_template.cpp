#include <atomic>

#include <spdlog/spdlog.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami_ros/extension_module_ros.hpp>

namespace ayanami_ros {

namespace {

// 可视化 emit 计数: 若扩展在 navigation->start() 之前完成构造并注册其
// NavigationCallbacks 订阅, start() 里的 publishMapsForVisualization emit 会被
// 本回调收到。用于 T6 加载顺序验证 (M2-P1: 扩展必须先于 navigation start 注册)。
std::atomic<int> g_stub_viz_emit_count{0};

}  // namespace

class StubExtension final : public ExtensionModuleROS {
public:
  StubExtension() {
    spdlog::info("[stub-extension] before-navig: register NavigationCallbacks");
    ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.add(
        [](const std::string&, const ayanami::nav_msgs::OccupancyGrid&) {
          g_stub_viz_emit_count.fetch_add(1);
        });
  }

  std::vector<GenericTopicSubscription> create_subscriptions(
      ros::NodeHandle&) override {
    return {};
  }
};

}  // namespace ayanami_ros

extern "C" ayanami_ros::ExtensionModuleROS* create_extension_module() {
  return new ayanami_ros::StubExtension();
}

extern "C" int stub_extension_viz_emit_count() {
  return ayanami_ros::g_stub_viz_emit_count.load();
}

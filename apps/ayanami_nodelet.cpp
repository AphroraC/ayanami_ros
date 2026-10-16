#include <atomic>
#include <memory>
#include <stdexcept>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

#include <ayanami/utility/logging.hpp>
#include <ayanami_ros/ayanami_ros.hpp>

namespace ayanami_ros {

// ayanami_ros nodelet wrapper (asuka mode): AyanamiROS itself stays
// entry-point agnostic; this class only hands it a private-namespace handle
// bound to a dedicated callback queue and drives its lifecycle.
//
// Callback threading: unlike asuka (whose nodelet spins its own
// AsyncSpinner(1) because AsukaROS has no spin loop), AyanamiROS::loop spins
// the handle's own queue via callAvailable -- the dedicated queue below is
// drained by that single loop thread, so every boundary callback (goal /
// pointcloud / laserscan / odom) is strictly serialized. Extension modules
// advertise on their own root handles (global queue) and are served by the
// host manager's spinner threads, exactly like asuka's viewers. The TF2Listener
// feeds itself from its dedicated thread, unaffected by either queue.
class AyanamiNodelet final : public nodelet::Nodelet {
public:
  AyanamiNodelet() = default;
  ~AyanamiNodelet() override;

  void onInit() override;

private:
  // Process-level single-instance guard: AyanamiROS routes through
  // process-global state (GlobalConfig, the NavigationCallbacks /
  // NavigationInputs slots defined exactly once in libayanami) -- a second
  // nodelet in the same manager would cross-wire slot traffic and config
  // routing. The flag never decrements: those globals survive instance
  // destruction.
  static std::atomic<int>& liveInstances() {
    static std::atomic<int> count{0};
    return count;
  }

  ros::CallbackQueue callback_queue{};
  std::unique_ptr<ros::NodeHandle> node{nullptr};
  std::unique_ptr<AyanamiROS> ayanami_ros{nullptr};
};

AyanamiNodelet::~AyanamiNodelet() {
  // Stop the boundary before its queue dies: ~AyanamiROS joins the loop
  // thread (the queue's only consumer) and stops the navigation module;
  // only then may the NodeHandle and its callback queue go away.
  ayanami_ros.reset();
  node.reset();
}

void AyanamiNodelet::onInit() {
  if (liveInstances().fetch_add(1, std::memory_order_relaxed) != 0) {
    throw std::runtime_error(
        "second AyanamiNodelet instance in this process: GlobalConfig and "
        "the NavigationCallbacks/NavigationInputs slots are single-instance "
        "by design");
  }

  // Private handle on a dedicated queue: subscribeTopics() inside AyanamiROS
  // then lands its goal/pointcloud/odom/laserscan subscriptions on this
  // queue instead of the manager's global one.
  node = std::make_unique<ros::NodeHandle>(getPrivateNodeHandle());
  node->setCallbackQueue(&callback_queue);

  // Startup failures (bad route config, missing module .so, robot_frame
  // mismatch) propagate out of onInit as load errors -- mirroring
  // ayanami_rosnode's "report and exit non-zero" behavior.
  ayanami_ros = std::make_unique<AyanamiROS>(*node);
  ayanami_ros->start();

  SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "Ayanami Nodelet initialized");
}

}  // namespace ayanami_ros

PLUGINLIB_EXPORT_CLASS(ayanami_ros::AyanamiNodelet, nodelet::Nodelet)

#include <ayanami_ros/extension_module_ros.hpp>
namespace ayanami_ros {
class StubExtension final : public ExtensionModuleROS {
public:
  std::vector<GenericTopicSubscription> create_subscriptions(ros::NodeHandle&) override { return {}; }
};
}
extern "C" ayanami_ros::ExtensionModuleROS* create_extension_module() { return new ayanami_ros::StubExtension(); }

#pragma once
#include <ros/ros.h>
#include <ayanami/utility/extension_module.hpp>
#include <vector>
#include <string>

namespace ayanami_ros {
class ExtensionModuleROS : public ayanami::ExtensionModule {
public:
  using Ptr = std::shared_ptr<ExtensionModuleROS>;
  struct GenericTopicSubscription {
    ros::Subscriber sub;
    std::string topic;
  };
  virtual std::vector<GenericTopicSubscription> create_subscriptions(ros::NodeHandle& nh) { return {}; }
  virtual bool needs_wait() const { return false; }
  virtual bool ok() const { return true; }
  virtual void at_exit(const std::string& dump_path) { (void)dump_path; }
};
}

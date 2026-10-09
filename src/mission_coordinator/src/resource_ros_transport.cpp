// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/resource_ros_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#include <factory_interfaces/srv/acquire_resource.hpp>
#include <factory_interfaces/srv/renew_resource.hpp>
#include <factory_interfaces/srv/release_resource.hpp>
#include <factory_interfaces/srv/cancel_resource_wait.hpp>
#include <map>
namespace mission_coordinator {
namespace {
using Acquire = factory_interfaces::srv::AcquireResource;
using Renew = factory_interfaces::srv::RenewResource;
using Release = factory_interfaces::srv::ReleaseResource;
using CancelWait = factory_interfaces::srv::CancelResourceWait;
class RosTransport : public ResourceTransport {
public:
  explicit RosTransport(rclcpp::Node *node) {
    acquire = node->create_client<Acquire>("/factory/resources/acquire");
    renew = node->create_client<Renew>("/factory/resources/renew");
    release = node->create_client<Release>("/factory/resources/release");
    cancel_wait = node->create_client<CancelWait>("/factory/resources/cancel_wait");
    timer = node->create_wall_timer(
        std::chrono::milliseconds(250), [pending = pending_calls] {
          const auto current = std::chrono::steady_clock::now();
          for (auto it = pending->begin(); it != pending->end();) {
            if (current >= it->second.deadline) {
              it->second.remove();
              it = pending->erase(it);
            } else
              ++it;
          }
        });
  }
  ~RosTransport() override {
    timer->cancel();
    for (const auto &call : *pending_calls)
      call.second.remove();
    pending_calls->clear();
  }
  bool available(ResourceOperation operation) const override {
    switch (operation) {
    case ResourceOperation::Acquire:
      return acquire->service_is_ready();
    case ResourceOperation::Renew:
      return renew->service_is_ready();
    case ResourceOperation::Release:
      return release->service_is_ready();
    case ResourceOperation::CancelWait:
      return cancel_wait->service_is_ready();
    }
    return false;
  }
  Cancel send(ResourceOperation operation, const LeaseIdentity &key,
              Reply callback) override {
    switch (operation) {
    case ResourceOperation::Acquire:
      return dispatch<Acquire>(acquire, key, callback, [](const auto &r) {
        const bool waiting =
            !r.granted && r.lease_id.empty() && r.lease_ttl_sec == 0 &&
            (r.reason == "queued" || r.reason == "reconciliation required");
        return ResourceReply{r.granted, r.lease_id, r.lease_ttl_sec, r.reason,
                             false,     false,      waiting};
      });
    case ResourceOperation::Renew:
      return dispatch<Renew>(renew, key, callback, [](const auto &r) {
        return ResourceReply{r.renewed, "", r.lease_ttl_sec, r.reason};
      });
    case ResourceOperation::Release:
      return dispatch<Release>(release, key, callback, [](const auto &r) {
        return ResourceReply{r.released, "", 0, r.reason};
      });
    case ResourceOperation::CancelWait:
      return dispatch<CancelWait>(cancel_wait, key, callback, [](const auto &r) {
        return ResourceReply{r.cancelled,
                             r.lease_id,
                             r.lease_ttl_sec,
                             r.reason,
                             r.reconciliation_required,
                             true};
      });
    }
    throw std::invalid_argument("invalid resource operation");
  }

private:
  template <class Service, class Convert>
  Cancel dispatch(typename rclcpp::Client<Service>::SharedPtr client,
                  const LeaseIdentity &key, Reply callback, Convert convert) {
    auto request = std::make_shared<typename Service::Request>();
    request->robot_id = key.robot_id;
    request->mission_id = key.mission_id;
    request->resource_id = key.resource_id;
    if constexpr (std::is_same_v<Service, Renew> || std::is_same_v<Service, Release>)
      request->lease_id = key.lease_id;
    const auto sequence = ++next_request;
    auto calls = pending_calls;
    auto pending = client->async_send_request(
        request, [callback, convert, calls,
                  sequence](typename rclcpp::Client<Service>::SharedFuture future) {
          calls->erase(sequence);
          ResourceReply reply;
          try {
            reply = convert(*future.get());
          } catch (const std::exception &error) {
            reply = {false, "", 0, error.what()};
          }
          callback(std::move(reply));
        });
    auto remove = [client, id = pending.request_id] {
      client->remove_pending_request(id);
    };
    (*calls)[sequence] = {std::chrono::steady_clock::now() + std::chrono::seconds(10),
                          remove};
    return [calls, sequence, remove] {
      // A canceled acquisition may already have granted centrally. Retain its
      // reply for bounded late exact-lease cleanup; never let it authorize entry.
      if constexpr (!std::is_same_v<Service, Acquire>) {
        remove();
        calls->erase(sequence);
      }
    };
  }
  rclcpp::Client<Acquire>::SharedPtr acquire;
  rclcpp::Client<Renew>::SharedPtr renew;
  rclcpp::Client<Release>::SharedPtr release;
  rclcpp::Client<CancelWait>::SharedPtr cancel_wait;
  struct PendingCall {
    std::chrono::steady_clock::time_point deadline;
    std::function<void()> remove;
  };
  std::shared_ptr<std::map<uint64_t, PendingCall>> pending_calls{
      std::make_shared<std::map<uint64_t, PendingCall>>()};
  uint64_t next_request{0};
  rclcpp::TimerBase::SharedPtr timer;
};
} // namespace
std::shared_ptr<ResourceTransport> make_resource_transport(rclcpp::Node *node) {
  return std::make_shared<RosTransport>(node);
}
} // namespace mission_coordinator

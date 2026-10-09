// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include <memory>
#include <string>

namespace mission_coordinator {
enum class ResourceOperation { Acquire, Renew, Release, CancelWait };
struct LeaseIdentity {
  std::string robot_id, mission_id, resource_id, lease_id;
};
struct ResourceReply {
  bool ok{false};
  std::string lease_id;
  double ttl{0};
  std::string reason;
  bool reconciliation_required{false};
  bool ownership_resolved{false};
  // True only for an authoritative capacity/backpressure service response.
  // A transport failure is never a healthy wait, regardless of its reason text.
  bool authoritative_wait{false};
};
class ResourceTransport {
public:
  using Reply = std::function<void(ResourceReply)>;
  using Cancel = std::function<void()>;
  virtual ~ResourceTransport() = default;
  virtual bool available(ResourceOperation operation) const = 0;
  virtual Cancel send(ResourceOperation operation, const LeaseIdentity &key,
                      Reply callback) = 0;
};
enum class ResourceState { Waiting, Granted, Lost, Released, Cancelled };
struct ResourceNotice {
  ResourceState state;
  LeaseIdentity lease;
  double expires_at;
  std::string reason;
};
// Serialized by the caller's executor. Steady process time is independent of ROS time.
// No method waits for service discovery or a response. The transport is async only.
class ResourceAdapter {
public:
  using Notice = std::function<void(const ResourceNotice &)>;
  ResourceAdapter(std::shared_ptr<ResourceTransport> transport, std::string robot_id,
                  std::function<double()> clock, double wait_timeout = 120.0);
  void acquire(const std::string &resource, const std::string &mission,
               Notice callback);
  void tick();
  bool authorized(const std::string &resource, const std::string &mission) const;
  bool enter(const std::string &resource, const std::string &mission);
  // Called only after physical exit or a confirmed station transaction.
  void exited(const std::string &resource, const std::string &mission);
  void release(const std::string &resource, const std::string &mission,
               std::function<void(bool)> callback);
  // Cancels pending acquisitions and requests cleanup only for proven-safe leases.
  // False means cleanup is pending or ownership/physical uncertainty remains.
  bool release_all(const std::string &mission);
  // True only while bounded cleanup/late acquire resolution can still finish.
  bool cleanup_pending(const std::string &mission) const;

private:
  struct State;
  std::shared_ptr<State> state_;
};
} // namespace mission_coordinator

// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/resource_adapter.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mission_coordinator {
struct ResourceAdapter::State : std::enable_shared_from_this<ResourceAdapter::State> {
  struct Entry {
    LeaseIdentity key;
    Notice callback;
    ResourceState status{ResourceState::Waiting};
    bool occupied{false}, pending{false};
    unsigned attempts{0};
    uint64_t sequence{0};
    double expires{0}, renew_at{0}, next{0}, sent{0};
    ResourceTransport::Cancel cancel;
    std::function<void(bool)> released;
    ResourceOperation operation{ResourceOperation::Acquire};
  };
  std::shared_ptr<ResourceTransport> transport;
  std::string robot;
  std::function<double()> clock;
  std::map<std::pair<std::string, std::string>, std::shared_ptr<Entry>> entries;
  struct Cleanup {
    LeaseIdentity key;
    ResourceOperation operation;
    unsigned attempts{0};
    uint64_t sequence{0};
    bool pending{false}, done{false};
    double sent{0}, next{0};
    ResourceTransport::Cancel cancel;
  };
  std::vector<std::shared_ptr<Cleanup>> cleanup_calls;
  void notify(const std::shared_ptr<Entry> &entry, ResourceState status,
              const std::string &reason) {
    entry->status = status;
    if (entry->callback)
      entry->callback({status, entry->key, entry->expires, reason});
  }
  void invalidate(const std::shared_ptr<Entry> &entry, const std::string &reason) {
    ++entry->sequence;
    entry->pending = false;
    if (entry->cancel)
      entry->cancel();
    entry->cancel = {};
    auto released = std::move(entry->released);
    notify(entry, ResourceState::Lost, reason);
    if (released)
      released(false);
  }
  void cleanup(const LeaseIdentity &key, ResourceOperation operation) {
    auto call = std::make_shared<Cleanup>();
    call->key = key;
    call->operation = operation;
    cleanup_calls.push_back(call);
    send_cleanup(call);
  }
  void send_cleanup(const std::shared_ptr<Cleanup> &call) {
    if (++call->attempts > 20) {
      call->done = true;
      return;
    }
    call->next = clock() + 0.25;
    if (!transport->available(call->operation))
      return;
    call->pending = true;
    call->sent = clock();
    const auto sequence = ++call->sequence;
    auto weak = weak_from_this();
    try {
      auto cancel = transport->send(call->operation, call->key,
                                    [weak, call, sequence](ResourceReply) {
                                      if (weak.lock() && sequence == call->sequence) {
                                        // A negative exact release is not clearance.
                                        // Cleanup cannot authorize any physical effect;
                                        // central quarantine remains authoritative.
                                        call->pending = false;
                                        call->done = true;
                                        call->cancel = {};
                                      }
                                    });
      if (call->pending)
        call->cancel = std::move(cancel);
    } catch (const std::exception &) {
      call->pending = false;
    }
  }
  void send(const std::shared_ptr<Entry> &entry, ResourceOperation operation) {
    entry->operation = operation;
    if (!transport->available(operation)) {
      if (operation == ResourceOperation::Acquire)
        retry(entry, "service unavailable");
      else
        invalidate(entry, "service unavailable");
      return;
    }
    const auto sequence = ++entry->sequence;
    entry->pending = true;
    entry->sent = clock();
    const auto sent = entry->sent;
    auto weak = weak_from_this();
    try {
      auto cancel = transport->send(
          operation, entry->key,
          [weak, entry, sequence, operation, sent](ResourceReply reply) {
            auto self = weak.lock();
            if (!self)
              return;
            if (sequence != entry->sequence) {
              if (operation == ResourceOperation::Acquire && !reply.ok &&
                  entry->status == ResourceState::Cancelled) {
                self->cleanup(entry->key, ResourceOperation::CancelWait);
              }
              if (operation == ResourceOperation::Acquire && reply.ok &&
                  entry->status != ResourceState::Waiting && !reply.lease_id.empty() &&
                  reply.lease_id != entry->key.lease_id) {
                auto key = entry->key;
                key.lease_id = reply.lease_id;
                self->cleanup(key, ResourceOperation::Release);
              }
              return;
            }
            entry->pending = false;
            entry->cancel = {};
            const double current = self->clock();
            if (operation == ResourceOperation::Release) {
              auto released = std::move(entry->released);
              if (reply.ok)
                self->notify(entry, ResourceState::Released, reply.reason);
              else
                self->invalidate(entry, reply.reason);
              if (released)
                released(reply.ok);
              return;
            }
            if (operation == ResourceOperation::Acquire && !reply.ok) {
              self->retry(entry, reply.reason);
              return;
            }
            if (!reply.ok || !std::isfinite(reply.ttl) || reply.ttl <= 0 ||
                sent + reply.ttl <= current ||
                (operation == ResourceOperation::Acquire && reply.lease_id.empty()) ||
                (operation == ResourceOperation::Renew && current >= entry->expires)) {
              if (operation == ResourceOperation::Acquire && reply.ok &&
                  !reply.lease_id.empty()) {
                entry->key.lease_id = reply.lease_id;
              }
              self->invalidate(entry, reply.reason.empty() ? "invalid or expired lease"
                                                           : reply.reason);
              return;
            }
            if (operation == ResourceOperation::Acquire)
              entry->key.lease_id = reply.lease_id;
            entry->expires = sent + reply.ttl;
            entry->renew_at = sent + reply.ttl / 3;
            if (operation == ResourceOperation::Acquire)
              self->notify(entry, ResourceState::Granted, reply.reason);
          });
      if (entry->pending && sequence == entry->sequence)
        entry->cancel = std::move(cancel);
    } catch (const std::exception &) {
      entry->pending = false;
      if (operation == ResourceOperation::Acquire)
        retry(entry, "service dispatch failed");
      else
        invalidate(entry, "service dispatch failed");
    }
  }
  void retry(const std::shared_ptr<Entry> &entry, const std::string &reason) {
    if (++entry->attempts >= 120) {
      cleanup(entry->key, ResourceOperation::CancelWait);
      invalidate(entry, "resource attempts exhausted: " + reason);
    } else {
      entry->next = clock() + 0.25;
      notify(entry, ResourceState::Waiting, reason);
    }
  }
};
ResourceAdapter::ResourceAdapter(std::shared_ptr<ResourceTransport> transport,
                                 std::string robot_id, std::function<double()> clock)
    : state_(std::make_shared<State>()) {
  if (!transport || robot_id.empty() || !clock)
    throw std::invalid_argument(
        "resource adapter requires transport, identity and clock");
  state_->transport = std::move(transport);
  state_->robot = std::move(robot_id);
  state_->clock = std::move(clock);
}
void ResourceAdapter::acquire(const std::string &resource, const std::string &mission,
                              Notice callback) {
  if (resource.empty() || mission.empty())
    throw std::invalid_argument("resource and mission must be nonempty");
  const auto index = std::make_pair(mission, resource);
  auto found = state_->entries.find(index);
  if (found != state_->entries.end() &&
      found->second->status != ResourceState::Released) {
    callback({found->second->status, found->second->key, found->second->expires,
              "existing request"});
    return;
  }
  auto entry = std::make_shared<State::Entry>();
  entry->key = {state_->robot, mission, resource, ""};
  entry->callback = std::move(callback);
  state_->entries[index] = entry;
  state_->notify(entry, ResourceState::Waiting, "acquiring");
  if (entry->status == ResourceState::Waiting)
    state_->send(entry, ResourceOperation::Acquire);
}
bool ResourceAdapter::authorized(const std::string &resource,
                                 const std::string &mission) const {
  auto found = state_->entries.find({mission, resource});
  return found != state_->entries.end() &&
         found->second->status == ResourceState::Granted &&
         state_->clock() < found->second->expires &&
         state_->transport->available(ResourceOperation::Renew);
}
bool ResourceAdapter::enter(const std::string &resource, const std::string &mission) {
  if (!authorized(resource, mission))
    return false;
  state_->entries.at({mission, resource})->occupied = true;
  return true;
}
void ResourceAdapter::exited(const std::string &resource, const std::string &mission) {
  auto found = state_->entries.find({mission, resource});
  if (found != state_->entries.end())
    found->second->occupied = false;
}
void ResourceAdapter::release(const std::string &resource, const std::string &mission,
                              std::function<void(bool)> callback) {
  auto found = state_->entries.find({mission, resource});
  if (found == state_->entries.end() || found->second->occupied ||
      !authorized(resource, mission)) {
    callback(false);
    return;
  }
  auto entry = found->second;
  if (entry->cancel)
    entry->cancel();
  entry->cancel = {};
  entry->pending = false;
  entry->released = std::move(callback);
  // A release in flight no longer authorizes another physical effect.
  entry->status = ResourceState::Waiting;
  state_->send(entry, ResourceOperation::Release);
}
bool ResourceAdapter::release_all(const std::string &mission) {
  bool safe = true;
  std::vector<std::shared_ptr<State::Entry>> entries;
  for (const auto &pair : state_->entries)
    if (pair.first.first == mission)
      entries.push_back(pair.second);
  for (const auto &entry : entries) {
    if (entry->status == ResourceState::Released ||
        entry->status == ResourceState::Cancelled)
      continue;
    if (entry->status == ResourceState::Waiting &&
        entry->operation == ResourceOperation::Acquire) {
      ++entry->sequence;
      if (entry->cancel)
        entry->cancel();
      entry->pending = false;
      entry->cancel = {};
      state_->notify(entry, ResourceState::Cancelled, "mission cancelled");
      state_->cleanup(entry->key, ResourceOperation::CancelWait);
    } else if (entry->occupied || entry->status == ResourceState::Lost) {
      safe = false;
    } else if (entry->operation == ResourceOperation::Release && entry->pending) {
      // Preserve the real release acknowledgement; never claim it already cleared.
      safe = false;
    } else {
      release(entry->key.resource_id, mission, [](bool) {});
      // A requested release is not acknowledged clearance. Terminal callers must
      // expose recovery until central ownership is actually confirmed released.
      safe = entry->status == ResourceState::Released && safe;
    }
  }
  return safe;
}
void ResourceAdapter::tick() {
  const auto cleanups = state_->cleanup_calls;
  for (const auto &call : cleanups) {
    if (call->done)
      continue;
    const auto current = state_->clock();
    if (call->pending && current - call->sent >= 1) {
      ++call->sequence;
      call->pending = false;
      if (call->cancel)
        call->cancel();
      call->cancel = {};
      call->next = current + 0.25;
    }
    if (!call->pending && current >= call->next)
      state_->send_cleanup(call);
  }
  auto &calls = state_->cleanup_calls;
  calls.erase(std::remove_if(calls.begin(), calls.end(),
                             [](const auto &call) { return call->done; }),
              calls.end());
  std::vector<std::shared_ptr<State::Entry>> entries;
  for (const auto &pair : state_->entries)
    entries.push_back(pair.second);
  for (const auto &entry : entries) {
    const double current = state_->clock();
    if (entry->status == ResourceState::Granted &&
        (current >= entry->expires ||
         !state_->transport->available(ResourceOperation::Renew))) {
      state_->invalidate(entry, "lease expired or service lost");
      continue;
    }
    if (entry->pending && current - entry->sent >= 1) {
      ++entry->sequence;
      entry->pending = false;
      if (entry->cancel)
        entry->cancel();
      entry->cancel = {};
      if (entry->operation == ResourceOperation::Acquire)
        state_->retry(entry, "service response timeout");
      else
        state_->invalidate(entry, "service response timeout");
      continue;
    }
    if (entry->pending)
      continue;
    if (entry->status == ResourceState::Waiting &&
        entry->operation == ResourceOperation::Acquire && current >= entry->next)
      state_->send(entry, ResourceOperation::Acquire);
    else if (entry->status == ResourceState::Granted && current >= entry->renew_at)
      state_->send(entry, ResourceOperation::Renew);
  }
}
} // namespace mission_coordinator

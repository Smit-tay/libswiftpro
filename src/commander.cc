// commander.cc
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.

#include "swiftpro/commander.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <vector>

namespace swiftpro {

// The firmware accepts at most 9 digits in "#N"; a longer one is answered
// without $N and could never be matched.
static constexpr uint32_t MAX_SEQ = 999999999;

Commander::Commander(serial::Serial& ser, float stale_after_s)
: ser_(ser)
, stale_after_s_(stale_after_s)
{}

Commander::~Commander()
{
    stop();
}

void Commander::start()
{
    if (running_.exchange(true)) { return; }
    writer_thread_ = std::thread(&Commander::writer_loop, this);
}

void Commander::stop()
{
    if (!running_.exchange(false)) { return; }

    // Wake the writer thread out of either CV.
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
    }
    queue_cv_.notify_all();
    pending_cv_.notify_all();

    if (writer_thread_.joinable()) { writer_thread_.join(); }

    // Commands never written, then commands never answered.
    std::deque<PendingCmd> unsent;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        unsent.swap(queue_);
    }
    for (auto& entry : unsent) {
        if (entry.cb) { entry.cb(false, "DISCONNECTED"); }
    }
    fail_pending("DISCONNECTED");
}

void Commander::send(const std::string& cmd, ResponseCallback cb)
{
    PendingCmd entry;

    if (cb) {
        entry.seq = next_seq();
        // Prefix with #N so the firmware echoes the sequence number back.
        entry.cmd = "#" + std::to_string(entry.seq) + cmd;
    } else {
        entry.seq = 0;
        entry.cmd = cmd;
    }
    entry.cb = std::move(cb);

    ResponseCallback evicted;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        if (!running_) {
            evicted = std::move(entry.cb);
        } else {
            if (queue_.size() >= QUEUE_CAPACITY) {
                // Outbound queue full — evict oldest. This is honest because
                // the firmware never saw the evicted command.
                std::cerr << "[swiftpro::Commander] queue full — evicting oldest command: "
                          << queue_.front().cmd << "\n";
                evicted = std::move(queue_.front().cb);
                queue_.pop_front();
            }
            queue_.push_back(std::move(entry));
            queue_cv_.notify_one();
        }
    }

    if (evicted) {
        evicted(false, running_ ? "QUEUE_FULL" : "NOT_CONNECTED");
    }
}

Result<std::string> Commander::send_sync(const std::string& cmd, float timeout_s)
{
    auto promise = std::make_shared<std::promise<Result<std::string>>>();
    auto future  = promise->get_future();

    send(cmd, [promise](bool ok, const std::string& response) {
        if (ok) {
            promise->set_value(Result<std::string>::ok(response));
        } else {
            promise->set_value(Result<std::string>::err(response));
        }
    });

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<float>(timeout_s);

    if (future.wait_until(deadline) == std::future_status::timeout) {
        // The pending entry stays: the command may still be running, and its
        // reply frees the slot. A reply that never comes is handled by
        // fail_pending() on reboot or link loss, or by the stale check.
        return Result<std::string>::err("TIMEOUT");
    }

    return future.get();
}

bool Commander::deliver_response(uint32_t seq, bool ok, const std::string& response)
{
    ResponseCallback cb;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        auto it = pending_.find(seq);
        if (it == pending_.end()) { return false; }
        cb = std::move(it->second.cb);
        pending_.erase(it);
    }

    // A slot just opened in the pending map — wake the writer thread if it
    // was back-pressured waiting for one.
    pending_cv_.notify_one();

    if (cb) { cb(ok, response); }
    return true;
}

void Commander::on_unsequenced_response(const std::string& response)
{
    // An error without $N belongs to a fire-and-forget command, or to a line
    // the firmware could not attribute (RX overflow). Nobody else can see it.
    if (response[0] == 'E') {
        std::cerr << "[swiftpro::Commander] firmware error without sequence: "
                  << response << "\n";
    }
}

void Commander::fail_pending(const std::string& reason)
{
    std::unordered_map<uint32_t, Pending> failed;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        failed.swap(pending_);
    }
    pending_cv_.notify_all();

    for (auto& [seq, pending] : failed) {
        if (pending.cb) { pending.cb(false, reason); }
    }
}

// ---------------------------------------------------------------------------

void Commander::writer_loop()
{
    while (running_) {
        PendingCmd entry;

        // Step 1 — wait for a command on the outbound queue.
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !queue_.empty() || !running_;
            });

            if (!running_) { break; }

            entry = std::move(queue_.front());
            queue_.pop_front();
        }

        // Step 2 — for sequenced commands, reserve a slot in the pending map.
        // If the map is at capacity, wait for a response to free one.
        // This back-pressures the writer to firmware throughput without
        // issuing false TIMEOUT errors to callers.
        if (entry.seq != 0) {
            if (!reserve_slot()) {
                entry.cb(false, "DISCONNECTED");
                return;
            }
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_[entry.seq] = Pending{std::move(entry.cb),
                                          std::chrono::steady_clock::now()};
        }

        // Step 3 — write to serial. Append newline if not present.
        std::string line = entry.cmd;
        if (line.empty() || line.back() != '\n') { line += '\n'; }

        try {
            ser_.write(line);
        } catch (const std::exception& e) {
            std::cerr << "[swiftpro::Commander] serial write error: " << e.what() << "\n";

            // Fire the callback with an error if it was registered, and
            // remove the pending entry so the slot is recycled.
            if (entry.seq != 0) {
                ResponseCallback cb;
                {
                    std::lock_guard<std::mutex> lock(pending_mutex_);
                    auto it = pending_.find(entry.seq);
                    if (it != pending_.end()) {
                        cb = std::move(it->second.cb);
                        pending_.erase(it);
                    }
                }
                pending_cv_.notify_one();
                if (cb) { cb(false, "IO_ERROR"); }
            }
        }
    }
}

bool Commander::reserve_slot()
{
    const auto stale_after = std::chrono::duration<float>(stale_after_s_);

    std::unique_lock<std::mutex> lock(pending_mutex_);
    while (running_ && pending_.size() >= PENDING_CAPACITY) {
        pending_cv_.wait_for(lock, std::chrono::milliseconds(250));

        // Slots whose reply never came (its line was lost) would otherwise
        // block the writer for good. Only checked when the map is full.
        std::vector<ResponseCallback> stale;
        const auto now = std::chrono::steady_clock::now();
        for (auto it = pending_.begin(); it != pending_.end();) {
            if (now - it->second.written >= stale_after) {
                std::cerr << "[swiftpro::Commander] no reply for seq "
                          << it->first << " — giving up on it\n";
                stale.push_back(std::move(it->second.cb));
                it = pending_.erase(it);
            } else {
                ++it;
            }
        }

        lock.unlock();
        for (auto& cb : stale) {
            if (cb) { cb(false, "TIMEOUT"); }
        }
        lock.lock();
    }
    return running_;
}

uint32_t Commander::next_seq()
{
    // 1 to MAX_SEQ; 0 is reserved for fire-and-forget.
    return seq_counter_.fetch_add(1) % MAX_SEQ + 1;
}

} // namespace swiftpro

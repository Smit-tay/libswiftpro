// commander.cc
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.

#include "swiftpro/commander.h"

#include <chrono>
#include <iostream>
#include <sstream>

namespace swiftpro {

Commander::Commander(serial::Serial& ser)
: ser_(ser)
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
    queue_cv_.notify_all();
    if (writer_thread_.joinable()) { writer_thread_.join(); }

    // Drain pending map — fire all outstanding callbacks with a timeout error.
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto& [seq, cb] : pending_) {
        if (cb) { cb(false, "TIMEOUT"); }
    }
    pending_.clear();
}

void Commander::send(const std::string& cmd, ResponseCallback cb)
{
    PendingCmd entry;
    entry.cb = cb;

    if (cb) {
        entry.seq = next_seq();
        // Prefix with #N so the firmware echoes the sequence number back.
        entry.cmd = "#" + std::to_string(entry.seq) + cmd;
    } else {
        entry.seq = 0;
        entry.cmd = cmd;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        if (queue_.size() >= QUEUE_CAPACITY) {
            // Evict oldest entry to avoid blocking the caller.
            std::cerr << "[uarm::Commander] queue full — evicting oldest command: "
                      << queue_.front().cmd << "\n";
            auto& evicted = queue_.front();
            if (evicted.cb) {
                evicted.cb(false, "QUEUE_FULL");
            }
            queue_.pop_front();
        }

        queue_.push_back(std::move(entry));
    }

    queue_cv_.notify_one();
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
        cb = std::move(it->second);
        pending_.erase(it);
    }
    if (cb) { cb(ok, response); }
    return true;
}

void Commander::on_unsequenced_response(const std::string& /*response*/)
{
    // Currently informational only.
}

// ---------------------------------------------------------------------------

void Commander::writer_loop()
{
    while (running_) {
        PendingCmd entry;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !queue_.empty() || !running_;
            });

            if (!running_ && queue_.empty()) { break; }
            if (queue_.empty()) { continue; }

            entry = std::move(queue_.front());
            queue_.pop_front();
        }

        // Register callback in pending map before writing — avoids a race
        // where the firmware responds before we register.
        if (entry.cb && entry.seq != 0) {
            std::lock_guard<std::mutex> lock(pending_mutex_);

            if (pending_.size() >= PENDING_CAPACITY) {
                // Evict oldest pending entry.
                auto oldest = pending_.begin();
                std::cerr << "[uarm::Commander] pending map full — evicting seq "
                          << oldest->first << "\n";
                if (oldest->second) { oldest->second(false, "TIMEOUT"); }
                pending_.erase(oldest);
            }

            pending_[entry.seq] = std::move(entry.cb);
        }

        // Write to serial. Append newline if not present.
        std::string line = entry.cmd;
        if (line.empty() || line.back() != '\n') { line += '\n'; }

        try {
            ser_.write(line);
        } catch (const std::exception& e) {
            std::cerr << "[uarm::Commander] serial write error: " << e.what() << "\n";

            // Fire the callback with an error if it was registered.
            if (entry.seq != 0) {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                auto it = pending_.find(entry.seq);
                if (it != pending_.end()) {
                    if (it->second) { it->second(false, "IO_ERROR"); }
                    pending_.erase(it);
                }
            }
        }
    }
}

uint32_t Commander::next_seq()
{
    // Skip 0 — reserved for fire-and-forget.
    uint32_t s = ++seq_counter_;
    if (s == 0) { s = ++seq_counter_; }
    return s;
}

} // namespace swiftpro

// commander.h
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Commander — owns the outbound command queue and the writer thread.
//
// Design:
//   - Callers enqueue commands via send(). Never blocks.
//   - Writer thread dequeues and writes to serial, one command at a time.
//   - Each command carries a sequence number and an optional response callback.
//   - Response callbacks are registered in a pending map, keyed by sequence
//     number. The Receiver calls deliver_response() when a response arrives.
//   - send_sync() wraps send() with a std::promise/future for callers that
//     need a blocking call.
//   - Queue capacity is fixed. If full, the oldest unacknowledged entry is
//     evicted with a warning. Callers are never blocked.

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "serial/serial.h"
#include "result.h"

namespace swiftpro {

// Maximum number of commands waiting to be written to serial.
static constexpr size_t QUEUE_CAPACITY = 32;

// Maximum number of commands awaiting a response from the firmware.
static constexpr size_t PENDING_CAPACITY = 16;

// Default command timeout in seconds.
static constexpr float DEFAULT_TIMEOUT_S = 5.0f;

// Response callback type. Called by Receiver on the recv thread when the
// firmware responds to a sequenced command.
using ResponseCallback = std::function<void(bool ok, const std::string& response)>;

class Commander
{
public:
    explicit Commander(serial::Serial& ser);
    ~Commander();

    // Non-copyable, non-movable.
    Commander(const Commander&)            = delete;
    Commander& operator=(const Commander&) = delete;

    // Start/stop the writer thread. Must call start() after serial is open.
    void start();
    void stop();

    // Enqueue a command. Never blocks. cb is called when the firmware responds.
    // If cb is nullptr the command is fire-and-forget (no sequence number).
    void send(const std::string& cmd, ResponseCallback cb = nullptr);

    // Blocking variant. Enqueues the command and waits up to timeout_s seconds
    // for the firmware response. Returns the raw response string on success.
    Result<std::string> send_sync(const std::string& cmd,
                                  float timeout_s = DEFAULT_TIMEOUT_S);

    // Called by Receiver when a sequenced response arrives.
    // response is the full response line (e.g. "ok V42.0").
    // Returns true if the sequence number was found and the callback fired.
    bool deliver_response(uint32_t seq, bool ok, const std::string& response);

    // Called by Receiver for fire-and-forget commands (no sequence number).
    // Not used for matching — just informational.
    void on_unsequenced_response(const std::string& response);

private:
    struct PendingCmd
    {
        uint32_t         seq;
        std::string      cmd;       // already formatted with #N prefix if sequenced
        ResponseCallback cb;        // nullptr = fire-and-forget
    };

    // Writer thread entry point.
    void writer_loop();

    // Allocate next sequence number (1-based, wraps at UINT32_MAX).
    uint32_t next_seq();

    serial::Serial&   ser_;

    // Outbound queue — commands waiting to be written.
    std::deque<PendingCmd>  queue_;
    std::mutex              queue_mutex_;
    std::condition_variable queue_cv_;

    // Pending map — commands written but not yet responded to.
    std::unordered_map<uint32_t, ResponseCallback> pending_;
    std::mutex                                      pending_mutex_;

    std::thread       writer_thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> seq_counter_{0};
};

} // namespace swiftpro

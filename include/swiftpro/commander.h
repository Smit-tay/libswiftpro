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
//
// Back-pressure model:
//   - Outbound queue has fixed capacity (QUEUE_CAPACITY). When full, send()
//     evicts the oldest unwritten command and fires its callback with
//     "QUEUE_FULL". Caller is never blocked.
//   - Pending map has fixed capacity (PENDING_CAPACITY). When full, the writer
//     thread waits on pending_cv_ until a response arrives and frees a slot.
//     This back-pressures the writer to firmware throughput without ever
//     issuing false TIMEOUT errors to callers whose commands the firmware
//     is actually processing.
//   - Urgent commands (send_urgent) overtake both: a stop must not wait for
//     the moves it is meant to stop.
//   - A reply that will never come must not hold a slot for good. All pending
//     commands fail on a firmware reboot or link loss (fail_pending). While the
//     map is full, entries unanswered for stale_after_s are failed with TIMEOUT.

#pragma once

#include <atomic>
#include <chrono>
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
static constexpr size_t PENDING_CAPACITY = 7;   // 7 x ~36 bytes fits the firmware's 256-byte RX buffer

// Default command timeout in seconds.
static constexpr float DEFAULT_TIMEOUT_S = 5.0f;

// How long a full pending map waits on an unanswered command before failing it.
// Longer than any sensible move: a move's reply comes only when it has finished.
static constexpr float STALE_PENDING_S = 120.0f;

// Response callback type. Called by Receiver on the recv thread when the
// firmware responds to a sequenced command.
using ResponseCallback = std::function<void(bool ok, const std::string& response)>;

class Commander
{
public:
    explicit Commander(serial::Serial& ser, float stale_after_s = STALE_PENDING_S);
    ~Commander();

    // Non-copyable, non-movable.
    Commander(const Commander&)            = delete;
    Commander& operator=(const Commander&) = delete;

    // Start/stop the writer thread. Must call start() after serial is open.
    void start();
    void stop();

    // Enqueue a command. Never blocks. cb is called when the firmware responds.
    // If cb is nullptr the command is fire-and-forget (no sequence number).
    // After stop(), cb is called at once with "NOT_CONNECTED".
    void send(const std::string& cmd, ResponseCallback cb = nullptr);

    // Enqueue a command ahead of everything unwritten, ignoring the pending
    // capacity: the writer sends it even while waiting for a free slot.
    // drop_queued fails every unwritten command with "STOPPED"; otherwise they
    // would be written as soon as the stop freed the pending map.
    void send_urgent(const std::string& cmd, ResponseCallback cb, bool drop_queued);

    // Blocking variant. Enqueues the command and waits up to timeout_s seconds
    // for the firmware response. Returns the raw response string on success.
    Result<std::string> send_sync(const std::string& cmd,
                                  float timeout_s = DEFAULT_TIMEOUT_S);

    // Called by Receiver when a sequenced response arrives.
    // response is the full response line (e.g. "ok V42.0").
    // Returns true if the sequence number was found and the callback fired.
    bool deliver_response(uint32_t seq, bool ok, const std::string& response);

    // Called by Receiver for fire-and-forget commands (no sequence number).
    // Not used for matching — just informational. response must not be empty.
    void on_unsequenced_response(const std::string& response);

    // Fails every command awaiting a response: their replies will never come.
    void fail_pending(const std::string& reason);

private:
    struct PendingCmd
    {
        uint32_t         seq;
        std::string      cmd;       // already formatted with #N prefix if sequenced
        ResponseCallback cb;        // nullptr = fire-and-forget
        uint64_t         generation = 0;
    };

    struct Pending
    {
        ResponseCallback                      cb;
        std::chrono::steady_clock::time_point written;
    };

    // Writer thread entry point.
    void writer_loop();

    // Waits for room in the pending map, sending urgent commands meanwhile.
    // False if stopping.
    bool reserve_slot();

    // Writes every urgent command.
    void write_urgent();

    // Registers a sequenced command as pending and writes it to serial.
    void write_entry(PendingCmd& entry);

    // Allocate next sequence number (1 to 999999999, then wraps).
    uint32_t next_seq();

    serial::Serial&   ser_;
    const float       stale_after_s_;

    // Outbound queue — commands waiting to be written.
    std::deque<PendingCmd>  queue_;
    std::deque<PendingCmd>  urgent_;
    uint64_t                generation_ = 0;  // bumped by a drop_queued
    std::atomic<bool>       urgent_waiting_{false};
    std::mutex              queue_mutex_;
    std::condition_variable queue_cv_;

    // Pending map — commands written but not yet responded to.
    // pending_cv_ signals when the map drops below capacity (writer wakes).
    std::unordered_map<uint32_t, Pending> pending_;
    std::mutex                            pending_mutex_;
    std::condition_variable               pending_cv_;

    std::thread       writer_thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> seq_counter_{0};
};

} // namespace swiftpro

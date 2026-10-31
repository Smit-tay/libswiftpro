// receiver.h
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Receiver — owns the recv thread and parses all incoming serial data.
//
// Incoming lines fall into three categories:
//
//   1. Sequenced responses — "$N ok ..." or "$N E2x ..."
//      Delivered to Commander::deliver_response(N, ok, line).
//
//   2. Unsequenced responses — "ok ..." or "E2x ..."
//      Delivered to Commander::on_unsequenced_response(line).
//
//   3. Unsolicited firmware events — "@N ..."
//      Parsed and delivered to the registered event callbacks.
//
// Event callbacks are set by Swift and called on the receiver thread.
// Callbacks must be non-blocking.

#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <mutex>
#include "serial/serial.h"

namespace swiftpro {

class Commander;

// Event callback types.
// @3 — periodic position report: x, y, z, wrist_angle (degrees)
using PositionCallback       = std::function<void(float x, float y, float z, float wrist)>;

// @3 joint angles: b=base with 0 = straight ahead, l=left, r=right (degrees)
using JointCallback          = std::function<void(float b, float l, float r)>;

using MotionCompleteCallback = std::function<void()>;
using LimitSwitchCallback    = std::function<void(bool state)>;
using PowerCallback          = std::function<void(bool on)>;

// @8 — lost steps: the arm has braked and its running commands fail with E25.
// joint: 0=base, 1=left, 2=right. error_deg: step tally minus encoder.
using LostStepsCallback      = std::function<void(int joint, float error_deg)>;

class Receiver
{
public:
    Receiver(serial::Serial& ser, Commander& commander);
    ~Receiver();

    Receiver(const Receiver&)            = delete;
    Receiver& operator=(const Receiver&) = delete;

    // Start/stop the receiver thread.
    void start();
    void stop();

    // Event callback registration. Thread-safe — may be called at any time.
    void on_position(PositionCallback cb);
    void on_joint(JointCallback cb);
    void on_motion_complete(MotionCompleteCallback cb);
    void on_limit_switch(LimitSwitchCallback cb);
    void on_power(PowerCallback cb);
    void on_lost_steps(LostStepsCallback cb);

    // False once a serial error has ended the receiver thread (e.g. the USB
    // cable was pulled). Recovery is a new Swift::connect().
    bool link_ok() const { return !link_lost_; }

private:
    void recv_loop();
    void dispatch_line(const std::string& line);
    void dispatch_event(const std::string& line);
    void dispatch_response(const std::string& line);

    // Event parsers.
    void parse_at3(const std::string& payload);
    void parse_at5(const std::string& payload);
    void parse_at6(const std::string& payload);
    void parse_at8(const std::string& payload);
    void parse_at9(const std::string& payload);

    serial::Serial& ser_;
    Commander&      commander_;

    std::thread       recv_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> link_lost_{false};

    // Event callbacks — protected by callbacks_mutex_.
    std::mutex               callbacks_mutex_;
    PositionCallback         cb_position_;
    JointCallback            cb_joint_;
    MotionCompleteCallback   cb_motion_complete_;
    LimitSwitchCallback      cb_limit_switch_;
    PowerCallback            cb_power_;
    LostStepsCallback        cb_lost_steps_;
};

} // namespace swiftpro

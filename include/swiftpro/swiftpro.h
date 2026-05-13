// uarm.h
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Swift — C++ interface to the UArm Swift Pro robotic arm.
//
// All motion/control commands are async by default. Pass a callback to be
// notified on completion, or use the _sync() variants to block.
//
// Example — async move:
//   arm.set_position(200, 0, 150, 50, [](VoidResult r) {
//       if (!r) std::cerr << r.error() << "\n";
//   });
//
// Example — sync move:
//   auto r = arm.set_position_sync(200, 0, 150, 50);
//   if (!r) std::cerr << r.error() << "\n";
//
// Event callbacks are called on the receiver thread. They must be fast and
// non-blocking.
//
// Thread safety: all public methods are thread-safe.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "result.h"
#include "swiftpro/receiver.h"    // for callback type aliases
#include "serial/serial.h"

namespace swiftpro {

class Commander;
class Receiver;

class Swift
{
public:
    Swift();
    ~Swift();

    Swift(const Swift&)            = delete;
    Swift& operator=(const Swift&) = delete;

    // ── Connection ────────────────────────────────────────────────────────

    // Connect to arm. Returns true on success.
    // Blocks until serial port is open. Does NOT wait for firmware boot —
    // caller must sleep ~2s before issuing commands (DTR reset).
    bool connect(const std::string& port, uint32_t baudrate = 115200);
    void disconnect();
    bool is_connected() const;

    // Send a raw GRBL $ command (e.g. "$11=0.5"). Response via callback.
    void send_raw(const std::string& cmd,
                  std::function<void(VoidResult)> cb = nullptr);
    VoidResult send_raw_sync(const std::string& cmd, float timeout_s = 5.0f);

    // ── Event callbacks ───────────────────────────────────────────────────
    // Called on the receiver thread. Must be non-blocking.

    // @3 periodic position report (fires at set_report_interval rate)
    // x,y,z in mm (firmware frame); wrist in degrees
    void on_position(PositionCallback cb);

    // @3 extended — joint angles (fires when firmware is extended)
    // b=base, l=left, r=right in firmware degrees
    void on_joint(JointCallback cb);

    // @9 motion complete (fires after set_motion_report(true))
    void on_motion_complete(MotionCompleteCallback cb);

    // @6 limit switch state change
    void on_limit_switch(LimitSwitchCallback cb);

    // @5 power state change
    void on_power(PowerCallback cb);

    // ── Configuration ─────────────────────────────────────────────────────

    // Enable/disable periodic @3 position reports.
    // interval_s: report interval in seconds (0 = disable)
    void set_report_interval(float interval_s,
                             std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_report_interval_sync(float interval_s,
                                        float timeout_s = 5.0f);

    // Enable/disable @9 motion-complete events.
    void set_motion_report(bool on,
                           std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_motion_report_sync(bool on, float timeout_s = 5.0f);

    // Set work mode (0=normal,1=laser,2=3Dprint,3=pen,4-8=extended)
    void set_mode(int mode, std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_mode_sync(int mode, float timeout_s = 5.0f);

    // Set acceleration (mm/s²)
    void set_acceleration(float acc,
                          std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_acceleration_sync(float acc, float timeout_s = 5.0f);

    // ── Motion ────────────────────────────────────────────────────────────

    // Move to Cartesian position. speed in mm/min.
    void set_position(float x, float y, float z, float speed,
                      bool relative = false,
                      std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_position_sync(float x, float y, float z, float speed,
                                 bool relative = false,
                                 float timeout_s = 30.0f);

    // Move to polar coordinates. speed in mm/min.
    void set_polar(float stretch, float rotation, float height, float speed,
                   bool relative = false,
                   std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_polar_sync(float stretch, float rotation, float height,
                              float speed, bool relative = false,
                              float timeout_s = 30.0f);

    // Move all three joints simultaneously. speed in firmware units.
    void set_joint_angles(float base, float left, float right, float speed,
                          std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_joint_angles_sync(float base, float left, float right,
                                     float speed, float timeout_s = 30.0f);

    // Move a single servo. id: 0=base,1=left,2=right,3=wrist. speed in fw units.
    void set_servo_angle(int id, float angle, float speed,
                         std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_servo_angle_sync(int id, float angle, float speed,
                                    float timeout_s = 30.0f);

    // Move wrist to angle. speed in firmware units.
    void set_wrist(float angle, float speed,
                   std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_wrist_sync(float angle, float speed,
                              float timeout_s = 30.0f);

    // Feed hold (pause in-flight motion).
    void pause_motion(std::function<void(VoidResult)> cb = nullptr);
    VoidResult pause_motion_sync(float timeout_s = 5.0f);

    // Cycle start (resume after feed hold).
    void resume_motion(std::function<void(VoidResult)> cb = nullptr);
    VoidResult resume_motion_sync(float timeout_s = 5.0f);

    // Hard abort — clears planner buffer, stops all motion.
    void motion_reset(std::function<void(VoidResult)> cb = nullptr);
    VoidResult motion_reset_sync(float timeout_s = 5.0f);

    // Composite reset: lock motors, release end effectors, move to home.
    void reset(float x = 200.0f, float y = 0.0f, float z = 150.0f,
               float speed = 50.0f,
               std::function<void(VoidResult)> cb = nullptr);
    VoidResult reset_sync(float x = 200.0f, float y = 0.0f,
                          float z = 150.0f, float speed = 50.0f,
                          float timeout_s = 30.0f);

    // ── Servo management ──────────────────────────────────────────────────

    // Attach (lock) servo(s). id=-1 for all.
    void set_servo_attach(int id,
                          std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_servo_attach_sync(int id, float timeout_s = 5.0f);

    // Detach (unlock) servo(s). id=-1 for all.
    void set_servo_detach(int id,
                          std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_servo_detach_sync(int id, float timeout_s = 5.0f);

    // Get servo attach state. Returns 1=attached, 0=detached.
    void get_servo_attach(int id,
                          std::function<void(IntResult)> cb);
    IntResult get_servo_attach_sync(int id, float timeout_s = 5.0f);

    // ── End effectors ─────────────────────────────────────────────────────

    void set_pump(bool on, std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_pump_sync(bool on, float timeout_s = 5.0f);

    void set_gripper(bool catch_obj,
                     std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_gripper_sync(bool catch_obj, float timeout_s = 5.0f);

    // Beep. freq in Hz, duration in seconds.
    void set_buzzer(int freq, float duration_s,
                    std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_buzzer_sync(int freq, float duration_s,
                               float timeout_s = 10.0f);

    // ── Queries ───────────────────────────────────────────────────────────

    // Current Cartesian position. Returns {x, y, z} mm.
    void get_position(std::function<void(VecResult)> cb);
    VecResult get_position_sync(float timeout_s = 5.0f);

    // Current polar position. Returns {stretch_mm, rotation_deg, height_mm}.
    void get_polar(std::function<void(VecResult)> cb);
    VecResult get_polar_sync(float timeout_s = 5.0f);

    // Current joint angles. Returns {base_deg, left_deg, right_deg}.
    void get_joint_angles(std::function<void(VecResult)> cb);
    VecResult get_joint_angles_sync(float timeout_s = 5.0f);

    // Single joint angle. id: 0=base,1=left,2=right,3=wrist.
    // Note: firmware P2206 returns only 2 tokens — use get_joint_angles for
    // bulk reads.
    void get_servo_angle(int id, std::function<void(FloatResult)> cb);
    FloatResult get_servo_angle_sync(int id, float timeout_s = 5.0f);

    // Check if Cartesian position is reachable. Returns 1=yes, 0=no.
    void is_reachable(float x, float y, float z,
                      std::function<void(IntResult)> cb);
    IntResult is_reachable_sync(float x, float y, float z,
                                float timeout_s = 5.0f);

    // Convert Cartesian → joint angles. Returns {base, left, right} degrees.
    void coord_to_angles(float x, float y, float z,
                         std::function<void(VecResult)> cb);
    VecResult coord_to_angles_sync(float x, float y, float z,
                                   float timeout_s = 5.0f);

    // Convert joint angles → Cartesian. Returns {x, y, z} mm.
    void angles_to_coord(float base, float left, float right,
                         std::function<void(VecResult)> cb);
    VecResult angles_to_coord_sync(float base, float left, float right,
                                   float timeout_s = 5.0f);

    // Pump status. Returns 0=off, 1=working, 2=holding.
    void get_pump_status(std::function<void(IntResult)> cb);
    IntResult get_pump_status_sync(float timeout_s = 5.0f);

    // Gripper status. Returns 0=released, 1=working, 2=holding.
    void get_gripper_status(std::function<void(IntResult)> cb);
    IntResult get_gripper_status_sync(float timeout_s = 5.0f);

    // Encoder health. Returns 0=all healthy, bitmask otherwise
    // (bit0=base, bit1=right, bit2=left). Returns -1 on query failure.
    void get_encoder_status(std::function<void(IntResult)> cb);
    IntResult get_encoder_status_sync(float timeout_s = 5.0f);

    // Motion state. Returns 0=idle, 1=moving.
    void get_is_moving(std::function<void(IntResult)> cb);
    IntResult get_is_moving_sync(float timeout_s = 5.0f);

    // Work mode.
    void get_mode(std::function<void(IntResult)> cb);
    IntResult get_mode_sync(float timeout_s = 5.0f);

    // Device info. Returns array of 5 strings:
    // [device_type, hw_version, fw_version, api_version, uuid]
    void get_device_info(std::function<void(InfoResult)> cb);
    InfoResult get_device_info_sync(float timeout_s = 10.0f);

    // ── GPIO ─────────────────────────────────────────────────────────────

    void set_digital_output(int pin, int value,
                            std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_digital_output_sync(int pin, int value,
                                       float timeout_s = 5.0f);

    void set_digital_direction(int pin, int value,
                               std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_digital_direction_sync(int pin, int value,
                                          float timeout_s = 5.0f);

    void get_digital(int pin, std::function<void(IntResult)> cb);
    IntResult get_digital_sync(int pin, float timeout_s = 5.0f);

    void get_analog(int pin, std::function<void(IntResult)> cb);
    IntResult get_analog_sync(int pin, float timeout_s = 5.0f);

private:
    // Parse helpers — used by query implementations.
    void query_vec3(const std::string& cmd,
                    std::function<void(VecResult)> cb);

    void query_int(const std::string& cmd,
                   std::function<void(IntResult)> cb);

    void query_float(const std::string& cmd,
                     std::function<void(FloatResult)> cb);

    // Send a void command (response is just "ok" or error).
    void cmd_void(const std::string& cmd,
                  std::function<void(VoidResult)> cb);

    serial::Serial            ser_;
    std::unique_ptr<Commander> commander_;
    std::unique_ptr<Receiver>  receiver_;
};

} // namespace swiftpro

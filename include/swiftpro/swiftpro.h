// swiftpro.h
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
// Thread safety: all public methods are thread-safe, except that connect() and
// disconnect() must not be called from a callback (they join its thread).
//
// Speed: every speed parameter is the firmware's F. 1 unit = 80 deg/min of
// joint-space vector speed (F50 = 66.7 deg/s), not mm/min. Each joint is capped
// at 300 deg/s by the firmware. F <= 0 is rejected with E21.
//
// Errors: Result::error() holds the firmware's reply ("E22", ...; see
// docs/protocol.md) or one of TIMEOUT, NOT_CONNECTED, DISCONNECTED,
// FIRMWARE_RESET, QUEUE_FULL, IO_ERROR. TIMEOUT from a _sync call means only that
// the wait ended: a move's reply comes when it has finished.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <mutex>
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

    // False once the serial link has failed (e.g. the USB cable was pulled).
    // connect() again to recover; all commands in flight fail with DISCONNECTED.
    bool is_connected() const;

    // Send a raw firmware command (at most 48 characters without spaces,
    // including the library's "#N"; "$" commands no longer exist). Only success
    // or failure is returned, not the reply's fields.
    void send_raw(const std::string& cmd,
                  std::function<void(VoidResult)> cb = nullptr);
    VoidResult send_raw_sync(const std::string& cmd, float timeout_s = 5.0f);

    // ── Event callbacks ───────────────────────────────────────────────────
    // Called on the receiver thread. Must be non-blocking. Register them after
    // connect(): each connection starts with none.

    // @3 periodic position report (fires at set_report_interval rate)
    // x,y,z in mm (firmware frame); wrist in degrees. From the step tally, so
    // it does not follow joints moved by hand while released: use
    // get_position() / get_joint_angles(), which read the encoders.
    void on_position(PositionCallback cb);

    // @3 joint angles from the step tally. l=left, r=right in firmware degrees;
    // b=base with 0 = straight ahead, unlike get_joint_angles() (90 = ahead).
    void on_joint(JointCallback cb);

    // @9 motion idle (fires after set_motion_report(true)). Also fires after a
    // move that was stopped or refused part-way: check the move's own result.
    void on_motion_complete(MotionCompleteCallback cb);

    // @6 limit switch state change
    void on_limit_switch(LimitSwitchCallback cb);

    // @5 power state change
    void on_power(PowerCallback cb);

    // @8 lost steps: the arm has stopped itself, moves in flight fail with E25.
    void on_lost_steps(LostStepsCallback cb);

    // ── Configuration ─────────────────────────────────────────────────────

    // Enable/disable periodic @3 position reports.
    // interval_s: report interval in seconds, 0.01 to 3600 (0 = disable)
    void set_report_interval(float interval_s,
                             std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_report_interval_sync(float interval_s,
                                        float timeout_s = 5.0f);

    // Enable/disable @9 motion-complete events.
    void set_motion_report(bool on,
                           std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_motion_report_sync(bool on, float timeout_s = 5.0f);

    // Streaming replies. Off (the default after boot): a move's result comes when
    // it has finished, and the firmware refuses commands once 20 are unanswered.
    // On: a move's result comes when the planner has accepted it, so a continuous
    // stream of moves can run; send this with the arm idle. A later non-motion
    // command still waits for idle.
    void set_streaming(bool on, std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_streaming_sync(bool on, float timeout_s = 5.0f);

    // Set work mode, saved in EEPROM. 0=normal, 1=laser, 2=3D printer, 3=pen,
    // 4=stepper flat, 5=stepper standard, 6=touch pen, 7=user, 8=claw jaw,
    // 9=steering gear, 10=clamp, 11=test. 4 and 5 reply after an origin search,
    // which can take longer than the default timeout.
    void set_mode(int mode, std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_mode_sync(int mode, float timeout_s = 5.0f);

    // Set acceleration, saved in EEPROM. acc * 80 deg/s² per joint, 0 < acc <= 50.
    void set_acceleration(float acc,
                          std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_acceleration_sync(float acc, float timeout_s = 5.0f);

    // ── Motion ────────────────────────────────────────────────────────────

    // Move to Cartesian position. speed is F (see top of file). Fails with E22,
    // nothing moved, if the target is outside the limits; with E27 if any joint
    // is released. The firmware does not know where the table is.
    void set_position(float x, float y, float z, float speed,
                      bool relative = false,
                      std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_position_sync(float x, float y, float z, float speed,
                                 bool relative = false,
                                 float timeout_s = 30.0f);

    // Move to polar coordinates, in a straight line. speed is F.
    void set_polar(float stretch, float rotation, float height, float speed,
                   bool relative = false,
                   std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_polar_sync(float stretch, float rotation, float height,
                              float speed, bool relative = false,
                              float timeout_s = 30.0f);

    // Move all three joints simultaneously. speed is F.
    void set_joint_angles(float base, float left, float right, float speed,
                          std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_joint_angles_sync(float base, float left, float right,
                                     float speed, float timeout_s = 30.0f);

    // Move a single servo. id: 0=base,1=left,2=right,3=wrist. speed is F.
    void set_servo_angle(int id, float angle, float speed,
                         std::function<void(VoidResult)> cb = nullptr);
    VoidResult set_servo_angle_sync(int id, float angle, float speed,
                                    float timeout_s = 30.0f);

    // Move wrist to angle, 0-180, at once. speed is ignored by the firmware.
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

    // Stop: the arm brakes, every queued command fails with E25, the position is
    // re-read from the encoders. The arm stays usable.
    void motion_reset(std::function<void(VoidResult)> cb = nullptr);
    VoidResult motion_reset_sync(float timeout_s = 5.0f);

    // Composite reset: lock motors, release end effectors, move to home. Only the
    // final move's result is returned.
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
    // Base is 90 = straight ahead; wrist is the commanded angle.
    void get_servo_angle(int id, std::function<void(FloatResult)> cb);
    FloatResult get_servo_angle_sync(int id, float timeout_s = 5.0f);

    // Check if Cartesian position is reachable. Returns 1=yes, 0=no.
    // This and the two conversions below wait for the arm to be idle.
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
    // (bit0=base, bit1=right, bit2=left). Result is an error on query failure.
    void get_encoder_status(std::function<void(IntResult)> cb);
    IntResult get_encoder_status_sync(float timeout_s = 5.0f);

    // Motion state, answered at once. Returns 0=idle, 1=moving.
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
    // Stops the threads and closes the port. Caller holds lifecycle_mutex_.
    void teardown();

    std::shared_ptr<Commander> current_commander() const;
    std::shared_ptr<Receiver>  current_receiver() const;

    // Sends through the current connection, or fails cb with NOT_CONNECTED.
    void send_cmd(const std::string& cmd,
                  std::function<void(bool, const std::string&)> cb);

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
    std::mutex                 lifecycle_mutex_;  // serializes connect/disconnect
    mutable std::mutex         link_mutex_;       // guards the two pointers
    std::shared_ptr<Commander> commander_;
    std::shared_ptr<Receiver>  receiver_;
};

} // namespace swiftpro

// swiftpro.cc
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.

#include "swiftpro/swiftpro.h"
#include "swiftpro/commander.h"
#include "swiftpro/receiver.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

namespace swiftpro {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Parse "ok V3.14 ..." response — extract float after first 'V' token.
static FloatResult parse_float_response(const std::string& resp)
{
    // tokens: ["ok", "V3.14", ...]
    std::istringstream iss(resp);
    std::string tok;
    iss >> tok;  // "ok"
    if (tok != "ok") {
        return FloatResult::err(resp);
    }
    if (!(iss >> tok)) {
        return FloatResult::err("no value in response: " + resp);
    }
    try {
        return FloatResult::ok(std::stof(tok.substr(1)));
    } catch (...) {
        return FloatResult::err("float parse failed: " + tok);
    }
}

// Parse "ok V42" — extract int after first 'V' token.
static IntResult parse_int_response(const std::string& resp)
{
    std::istringstream iss(resp);
    std::string tok;
    iss >> tok;
    if (tok != "ok") {
        // Check for E26 (encoder error) — special case, value still present.
        if (tok.rfind("E26", 0) == 0) {
            if (iss >> tok) {
                try {
                    return IntResult::ok(std::stoi(tok.substr(1)));
                } catch (...) {}
            }
        }
        return IntResult::err(resp);
    }
    if (!(iss >> tok)) {
        return IntResult::err("no value in response: " + resp);
    }
    try {
        return IntResult::ok(std::stoi(tok.substr(1)));
    } catch (...) {
        return IntResult::err("int parse failed: " + tok);
    }
}

// Parse "ok X1.0 Y2.0 Z3.0" or "ok B1.0 L2.0 R3.0" — extract 3 floats.
static VecResult parse_vec3_response(const std::string& resp)
{
    std::istringstream iss(resp);
    std::string tok;
    iss >> tok;
    if (tok != "ok") {
        return VecResult::err(resp);
    }
    std::vector<float> vals;
    while (iss >> tok && vals.size() < 3) {
        try {
            vals.push_back(std::stof(tok.substr(1)));
        } catch (...) {
            return VecResult::err("vec3 parse failed at token: " + tok);
        }
    }
    if (vals.size() < 3) {
        return VecResult::err("insufficient values in response: " + resp);
    }
    return VecResult::ok(vals);
}


// Swift
// ---------------------------------------------------------------------------

Swift::Swift() = default;

Swift::~Swift()
{
    disconnect();
}

bool Swift::connect(const std::string& port, uint32_t baudrate)
{
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    teardown();
    try {
        ser_.setPort(port);
        ser_.setBaudrate(baudrate);
        auto timeout = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(timeout);
        ser_.open();

        if (!ser_.isOpen()) {
            std::cerr << "[swiftpro] failed to open " << port << "\n";
            return false;
        }

        try {
            ser_.setRTS(true);
            ser_.setDTR(true);
        } catch (const std::exception& e) {
            // Not every port has modem lines; the link works without them.
            std::cerr << "[swiftpro] RTS/DTR not set: " << e.what() << "\n";
        }

        auto commander = std::make_shared<Commander>(ser_);
        auto receiver  = std::make_shared<Receiver>(ser_, *commander);

        receiver->start();
        commander->start();

        {
            std::lock_guard<std::mutex> lock(link_mutex_);
            commander_ = std::move(commander);
            receiver_  = std::move(receiver);
        }

        std::cerr << "[swiftpro] connected to " << port << "\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[swiftpro] connect error: " << e.what() << "\n";
        return false;
    }
}

void Swift::disconnect()
{
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    teardown();
}

void Swift::teardown()
{
    std::shared_ptr<Commander> commander;
    std::shared_ptr<Receiver>  receiver;
    {
        std::lock_guard<std::mutex> lock(link_mutex_);
        commander = std::move(commander_);
        receiver  = std::move(receiver_);
        commander_.reset();
        receiver_.reset();
    }

    // The receiver first: it calls into the commander.
    if (receiver)  { receiver->stop(); }
    if (commander) { commander->stop(); }

    if (ser_.isOpen()) {
        try {
            ser_.close();
        } catch (const std::exception& e) {
            std::cerr << "[swiftpro] close error: " << e.what() << "\n";
        }
        std::cerr << "[swiftpro] disconnected\n";
    }
}

bool Swift::is_connected() const
{
    const auto receiver = current_receiver();
    return receiver && receiver->link_ok();
}

std::shared_ptr<Commander> Swift::current_commander() const
{
    std::lock_guard<std::mutex> lock(link_mutex_);
    return commander_;
}

std::shared_ptr<Receiver> Swift::current_receiver() const
{
    std::lock_guard<std::mutex> lock(link_mutex_);
    return receiver_;
}

// ── Event callbacks ──────────────────────────────────────────────────────

void Swift::on_position(PositionCallback cb)
{
    if (const auto r = current_receiver()) { r->on_position(std::move(cb)); }
}

void Swift::on_joint(JointCallback cb)
{
    if (const auto r = current_receiver()) { r->on_joint(std::move(cb)); }
}

void Swift::on_motion_complete(MotionCompleteCallback cb)
{
    if (const auto r = current_receiver()) { r->on_motion_complete(std::move(cb)); }
}

void Swift::on_limit_switch(LimitSwitchCallback cb)
{
    if (const auto r = current_receiver()) { r->on_limit_switch(std::move(cb)); }
}

void Swift::on_power(PowerCallback cb)
{
    if (const auto r = current_receiver()) { r->on_power(std::move(cb)); }
}

void Swift::on_lost_steps(LostStepsCallback cb)
{
    if (const auto r = current_receiver()) { r->on_lost_steps(std::move(cb)); }
}

// ── Private helpers ──────────────────────────────────────────────────────

void Swift::send_cmd(const std::string& cmd,
                     std::function<void(bool, const std::string&)> cb)
{
    const auto commander = current_commander();
    if (!commander) {
        if (cb) { cb(false, "NOT_CONNECTED"); }
        return;
    }
    commander->send(cmd, std::move(cb));
}

void Swift::cmd_void(const std::string& cmd,
                     std::function<void(VoidResult)> cb)
{
    if (!cb) {
        send_cmd(cmd, nullptr);
        return;
    }
    send_cmd(cmd, [cb](bool ok, const std::string& resp) {
        cb(ok ? VoidResult::ok() : VoidResult::err(resp));
    });
}

void Swift::cmd_urgent(const std::string& cmd,
                       std::function<void(VoidResult)> cb,
                       bool drop_queued)
{
    std::function<void(bool, const std::string&)> response;
    if (cb) {
        response = [cb](bool ok, const std::string& resp) {
            cb(ok ? VoidResult::ok() : VoidResult::err(resp));
        };
    }

    const auto commander = current_commander();
    if (!commander) {
        if (response) { response(false, "NOT_CONNECTED"); }
        return;
    }
    commander->send_urgent(cmd, std::move(response), drop_queued);
}

void Swift::query_vec3(const std::string& cmd,
                       std::function<void(VecResult)> cb)
{
    send_cmd(cmd, [cb](bool /*ok*/, const std::string& resp) {
        cb(parse_vec3_response(resp));
    });
}

void Swift::query_int(const std::string& cmd,
                      std::function<void(IntResult)> cb)
{
    send_cmd(cmd, [cb](bool /*ok*/, const std::string& resp) {
        cb(parse_int_response(resp));
    });
}

void Swift::query_float(const std::string& cmd,
                        std::function<void(FloatResult)> cb)
{
    send_cmd(cmd, [cb](bool /*ok*/, const std::string& resp) {
        cb(parse_float_response(resp));
    });
}

// Sync wrapper template — reduces boilerplate for all _sync() methods.
template<typename R>
static R sync_call(std::function<void(std::function<void(R)>)> async_fn,
                   float timeout_s)
{
    auto promise = std::make_shared<std::promise<R>>();
    auto future  = promise->get_future();

    async_fn([promise](R result) {
        promise->set_value(std::move(result));
    });

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<float>(timeout_s);

    if (future.wait_until(deadline) == std::future_status::timeout) {
        if constexpr (std::is_same_v<R, VoidResult>) {
            return VoidResult::err("TIMEOUT");
        } else if constexpr (std::is_same_v<R, IntResult>) {
            return IntResult::err("TIMEOUT");
        } else if constexpr (std::is_same_v<R, FloatResult>) {
            return FloatResult::err("TIMEOUT");
        } else if constexpr (std::is_same_v<R, VecResult>) {
            return VecResult::err("TIMEOUT");
        } else if constexpr (std::is_same_v<R, StringResult>) {
            return StringResult::err("TIMEOUT");
        } else if constexpr (std::is_same_v<R, InfoResult>) {
            return InfoResult::err("TIMEOUT");
        }
    }

    return future.get();
}

// ── Configuration ────────────────────────────────────────────────────────

void Swift::set_report_interval(float interval_s,
                                std::function<void(VoidResult)> cb)
{
    char _ri[32]; std::snprintf(_ri,sizeof(_ri),"M2120 V%.3f",interval_s); cmd_void(_ri,std::move(cb));
}

VoidResult Swift::set_report_interval_sync(float interval_s, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, interval_s](auto cb) { set_report_interval(interval_s, cb); },
        timeout_s);
}

void Swift::set_motion_report(bool on, std::function<void(VoidResult)> cb)
{
    cmd_void(std::string("M2122 V") + (on ? "1" : "0"), std::move(cb));
}

VoidResult Swift::set_motion_report_sync(bool on, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, on](auto cb) { set_motion_report(on, cb); },
        timeout_s);
}

void Swift::set_streaming(bool on, std::function<void(VoidResult)> cb)
{
    cmd_void(std::string("M2126 V") + (on ? "1" : "0"), std::move(cb));
}

VoidResult Swift::set_streaming_sync(bool on, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, on](auto cb) { set_streaming(on, cb); },
        timeout_s);
}

void Swift::set_mode(int mode, std::function<void(VoidResult)> cb)
{
    cmd_void("M2400 S" + std::to_string(mode), std::move(cb));
}

VoidResult Swift::set_mode_sync(int mode, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, mode](auto cb) { set_mode(mode, cb); },
        timeout_s);
}

void Swift::set_acceleration(float acc, std::function<void(VoidResult)> cb)
{
    char _ac[32]; std::snprintf(_ac,sizeof(_ac),"M204 A%.1f",acc); cmd_void(_ac,std::move(cb));
}

VoidResult Swift::set_acceleration_sync(float acc, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, acc](auto cb) { set_acceleration(acc, cb); },
        timeout_s);
}

// ── Motion ───────────────────────────────────────────────────────────────

void Swift::set_position(float x, float y, float z, float speed,
                         bool relative,
                         std::function<void(VoidResult)> cb)
{
    char _p[80];
    std::snprintf(_p,sizeof(_p),
        relative ? "G2204 X%.1f Y%.1f Z%.1f F%.1f"
                 : "G0 X%.1f Y%.1f Z%.1f F%.1f",
        x,y,z,speed);
    cmd_void(_p, std::move(cb));
}

VoidResult Swift::set_position_sync(float x, float y, float z, float speed,
                                     bool relative, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, x, y, z, speed, relative](auto cb) {
            set_position(x, y, z, speed, relative, cb);
        }, timeout_s);
}

void Swift::set_polar(float stretch, float rotation, float height,
                      float speed, bool relative,
                      std::function<void(VoidResult)> cb)
{
    char _pl[80];
    std::snprintf(_pl,sizeof(_pl),
        relative ? "G2205 S%.1f R%.1f H%.1f F%.1f"
                 : "G2201 S%.1f R%.1f H%.1f F%.1f",
        stretch,rotation,height,speed);
    cmd_void(_pl, std::move(cb));
}

VoidResult Swift::set_polar_sync(float stretch, float rotation, float height,
                                  float speed, bool relative, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, stretch, rotation, height, speed, relative](auto cb) {
            set_polar(stretch, rotation, height, speed, relative, cb);
        }, timeout_s);
}

void Swift::set_joint_angles(float base, float left, float right, float speed,
                              std::function<void(VoidResult)> cb)
{
    char _j[64];
    std::snprintf(_j,sizeof(_j),"G2206 B%.1f L%.1f R%.1f F%.1f",base,left,right,speed);
    cmd_void(_j, std::move(cb));
}

VoidResult Swift::set_joint_angles_sync(float base, float left, float right,
                                         float speed, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, base, left, right, speed](auto cb) {
            set_joint_angles(base, left, right, speed, cb);
        }, timeout_s);
}

void Swift::set_servo_angle(int id, float angle, float speed,
                             std::function<void(VoidResult)> cb)
{
    char _sa[48];
    std::snprintf(_sa,sizeof(_sa),"G2202 N%d V%.1f F%.1f",id,angle,speed);
    cmd_void(_sa, std::move(cb));
}

VoidResult Swift::set_servo_angle_sync(int id, float angle, float speed,
                                        float timeout_s)
{
    return sync_call<VoidResult>(
        [this, id, angle, speed](auto cb) {
            set_servo_angle(id, angle, speed, cb);
        }, timeout_s);
}

void Swift::set_wrist(float angle, float speed,
                      std::function<void(VoidResult)> cb)
{
    set_servo_angle(3, angle, speed, std::move(cb));
}

VoidResult Swift::set_wrist_sync(float angle, float speed, float timeout_s)
{
    return set_servo_angle_sync(3, angle, speed, timeout_s);
}

void Swift::pause_motion(std::function<void(VoidResult)> cb)
{
    cmd_urgent("S1000 V0", std::move(cb), false);
}

VoidResult Swift::pause_motion_sync(float timeout_s)
{
    return sync_call<VoidResult>(
        [this](auto cb) { pause_motion(cb); }, timeout_s);
}

void Swift::resume_motion(std::function<void(VoidResult)> cb)
{
    cmd_urgent("S1000 V1", std::move(cb), false);
}

VoidResult Swift::resume_motion_sync(float timeout_s)
{
    return sync_call<VoidResult>(
        [this](auto cb) { resume_motion(cb); }, timeout_s);
}

void Swift::motion_reset(std::function<void(VoidResult)> cb)
{
    cmd_urgent("S1100", std::move(cb), true);
}

VoidResult Swift::motion_reset_sync(float timeout_s)
{
    return sync_call<VoidResult>(
        [this](auto cb) { motion_reset(cb); }, timeout_s);
}

void Swift::reset(float x, float y, float z, float speed,
                  std::function<void(VoidResult)> cb)
{
    // Composite: lock motors, release end effectors, move home.
    // Fire-and-forget the setup commands, callback on the final move.
    cmd_void("M17", nullptr);
    cmd_void("G2202 N3 V90 F1000", nullptr);
    cmd_void("M2231 V0", nullptr);
    cmd_void("M2232 V0", nullptr);
    set_position(x, y, z, speed, false, std::move(cb));
}

VoidResult Swift::reset_sync(float x, float y, float z, float speed,
                               float timeout_s)
{
    return sync_call<VoidResult>(
        [this, x, y, z, speed](auto cb) { reset(x, y, z, speed, cb); },
        timeout_s);
}
// ── Raw command ─────────────────────────────────────────────────────

void Swift::send_raw(const std::string& cmd,
                     std::function<void(VoidResult)> cb)
{
    cmd_void(cmd, std::move(cb));
}

VoidResult Swift::send_raw_sync(const std::string& cmd, float timeout_s)
{
    const auto commander = current_commander();
    if (!commander) { return VoidResult::err("NOT_CONNECTED"); }
    auto r = commander->send_sync(cmd, timeout_s);
    if (!r) { return VoidResult::err(r.error()); }
    return r->rfind("ok", 0) == 0 ? VoidResult::ok() : VoidResult::err(*r);
}

// ── Servo management ─────────────────────────────────────────────────────

void Swift::set_servo_attach(int id, std::function<void(VoidResult)> cb)
{
    std::string cmd = (id < 0) ? "M17" : "M2201 N" + std::to_string(id);
    cmd_void(cmd, std::move(cb));
}

VoidResult Swift::set_servo_attach_sync(int id, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, id](auto cb) { set_servo_attach(id, cb); }, timeout_s);
}

void Swift::set_servo_detach(int id, std::function<void(VoidResult)> cb)
{
    std::string cmd = (id < 0) ? "M2019" : "M2202 N" + std::to_string(id);
    cmd_void(cmd, std::move(cb));
}

VoidResult Swift::set_servo_detach_sync(int id, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, id](auto cb) { set_servo_detach(id, cb); }, timeout_s);
}

void Swift::get_servo_attach(int id, std::function<void(IntResult)> cb)
{
    query_int("M2203 N" + std::to_string(id), std::move(cb));
}

IntResult Swift::get_servo_attach_sync(int id, float timeout_s)
{
    return sync_call<IntResult>(
        [this, id](auto cb) { get_servo_attach(id, cb); }, timeout_s);
}

// ── End effectors ─────────────────────────────────────────────────────────

void Swift::set_pump(bool on, std::function<void(VoidResult)> cb)
{
    cmd_void(std::string("M2231 V") + (on ? "1" : "0"), std::move(cb));
}

VoidResult Swift::set_pump_sync(bool on, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, on](auto cb) { set_pump(on, cb); }, timeout_s);
}

void Swift::set_gripper(bool catch_obj, std::function<void(VoidResult)> cb)
{
    cmd_void(std::string("M2232 V") + (catch_obj ? "1" : "0"), std::move(cb));
}

VoidResult Swift::set_gripper_sync(bool catch_obj, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, catch_obj](auto cb) { set_gripper(catch_obj, cb); }, timeout_s);
}

void Swift::set_buzzer(int freq, float duration_s,
                       std::function<void(VoidResult)> cb)
{
    // Firmware expects duration in ms.
    const int duration_ms = static_cast<int>(duration_s * 1000.0f);
    char _bz[48];
    std::snprintf(_bz,sizeof(_bz),"M2210 F%d T%d",freq,duration_ms);
    cmd_void(_bz, std::move(cb));
}

VoidResult Swift::set_buzzer_sync(int freq, float duration_s, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, freq, duration_s](auto cb) { set_buzzer(freq, duration_s, cb); },
        timeout_s);
}

// ── Queries ───────────────────────────────────────────────────────────────

void Swift::get_position(std::function<void(VecResult)> cb)
{
    query_vec3("P2220", std::move(cb));
}

VecResult Swift::get_position_sync(float timeout_s)
{
    return sync_call<VecResult>(
        [this](auto cb) { get_position(cb); }, timeout_s);
}

void Swift::get_polar(std::function<void(VecResult)> cb)
{
    query_vec3("P2221", std::move(cb));
}

VecResult Swift::get_polar_sync(float timeout_s)
{
    return sync_call<VecResult>(
        [this](auto cb) { get_polar(cb); }, timeout_s);
}

void Swift::get_joint_angles(std::function<void(VecResult)> cb)
{
    query_vec3("P2200", std::move(cb));
}

VecResult Swift::get_joint_angles_sync(float timeout_s)
{
    return sync_call<VecResult>(
        [this](auto cb) { get_joint_angles(cb); }, timeout_s);
}

void Swift::get_servo_angle(int id, std::function<void(FloatResult)> cb)
{
    query_float("P2206 N" + std::to_string(id), std::move(cb));
}

FloatResult Swift::get_servo_angle_sync(int id, float timeout_s)
{
    return sync_call<FloatResult>(
        [this, id](auto cb) { get_servo_angle(id, cb); }, timeout_s);
}

void Swift::is_reachable(float x, float y, float z,
                          std::function<void(IntResult)> cb)
{
    char _ir[48];
    std::snprintf(_ir,sizeof(_ir),"M2222 X%.1f Y%.1f Z%.1f P0",x,y,z);
    query_int(_ir, std::move(cb));
}

IntResult Swift::is_reachable_sync(float x, float y, float z, float timeout_s)
{
    return sync_call<IntResult>(
        [this, x, y, z](auto cb) { is_reachable(x, y, z, cb); }, timeout_s);
}

void Swift::coord_to_angles(float x, float y, float z,
                             std::function<void(VecResult)> cb)
{
    char _ca[48];
    std::snprintf(_ca,sizeof(_ca),"M2220 X%.1f Y%.1f Z%.1f",x,y,z);
    query_vec3(_ca, std::move(cb));
}

VecResult Swift::coord_to_angles_sync(float x, float y, float z,
                                       float timeout_s)
{
    return sync_call<VecResult>(
        [this, x, y, z](auto cb) { coord_to_angles(x, y, z, cb); }, timeout_s);
}

void Swift::angles_to_coord(float base, float left, float right,
                             std::function<void(VecResult)> cb)
{
    char _atc[48];
    std::snprintf(_atc,sizeof(_atc),"M2221 B%.1f L%.1f R%.1f",base,left,right);
    query_vec3(_atc, std::move(cb));
}

VecResult Swift::angles_to_coord_sync(float base, float left, float right,
                                       float timeout_s)
{
    return sync_call<VecResult>(
        [this, base, left, right](auto cb) {
            angles_to_coord(base, left, right, cb);
        }, timeout_s);
}

void Swift::get_pump_status(std::function<void(IntResult)> cb)
{
    query_int("P2231", std::move(cb));
}

IntResult Swift::get_pump_status_sync(float timeout_s)
{
    return sync_call<IntResult>(
        [this](auto cb) { get_pump_status(cb); }, timeout_s);
}

void Swift::get_gripper_status(std::function<void(IntResult)> cb)
{
    query_int("P2232", std::move(cb));
}

IntResult Swift::get_gripper_status_sync(float timeout_s)
{
    return sync_call<IntResult>(
        [this](auto cb) { get_gripper_status(cb); }, timeout_s);
}

void Swift::get_encoder_status(std::function<void(IntResult)> cb)
{
    // P2244 returns "ok V0" (healthy) or "E26 V<bitmask>" (fault).
    // Both cases are handled by parse_int_response.
    query_int("P2244", std::move(cb));
}

IntResult Swift::get_encoder_status_sync(float timeout_s)
{
    return sync_call<IntResult>(
        [this](auto cb) { get_encoder_status(cb); }, timeout_s);
}

void Swift::get_is_moving(std::function<void(IntResult)> cb)
{
    query_int("M2200", std::move(cb));
}

IntResult Swift::get_is_moving_sync(float timeout_s)
{
    return sync_call<IntResult>(
        [this](auto cb) { get_is_moving(cb); }, timeout_s);
}

void Swift::get_mode(std::function<void(IntResult)> cb)
{
    query_int("P2400", std::move(cb));
}

IntResult Swift::get_mode_sync(float timeout_s)
{
    return sync_call<IntResult>(
        [this](auto cb) { get_mode(cb); }, timeout_s);
}

void Swift::get_device_info(std::function<void(InfoResult)> cb)
{
    // Fire 5 queries and collect results into an array.
    // Use a shared state to gather all 5 responses.
    struct State {
        std::mutex                  mu;
        std::array<std::string, 5>  info{};
        std::string                 error;
        int                         count{0};
        std::function<void(InfoResult)> cb;
    };

    auto state = std::make_shared<State>();
    state->cb  = std::move(cb);

    const std::array<std::string, 5> cmds = {
        "P2201", "P2202", "P2203", "P2204", "P2205"
    };

    for (int i = 0; i < 5; ++i) {
        send_cmd(cmds[i],
            [state, i](bool ok, const std::string& resp) {
                // Strip "ok V" prefix — value starts after first space+char.
                // An empty UID is "ok V".
                std::string val;
                auto space = resp.find(' ');
                if (space != std::string::npos && space + 1 < resp.size()) {
                    val = resp.substr(space + 2); // skip prefix char e.g. 'V'
                }

                std::lock_guard<std::mutex> lock(state->mu);
                state->info[i] = val;
                if (!ok && state->error.empty()) { state->error = resp; }
                if (++state->count == 5) {
                    state->cb(state->error.empty()
                              ? InfoResult::ok(state->info)
                              : InfoResult::err(state->error));
                }
            });
    }
}

InfoResult Swift::get_device_info_sync(float timeout_s)
{
    return sync_call<InfoResult>(
        [this](auto cb) { get_device_info(cb); }, timeout_s);
}

// ── GPIO ─────────────────────────────────────────────────────────────────

void Swift::set_digital_output(int pin, int value,
                                std::function<void(VoidResult)> cb)
{
    cmd_void("M2240 N" + std::to_string(pin) + " V" + std::to_string(value),
             std::move(cb));
}

VoidResult Swift::set_digital_output_sync(int pin, int value, float timeout_s)
{
    return sync_call<VoidResult>(
        [this, pin, value](auto cb) { set_digital_output(pin, value, cb); },
        timeout_s);
}

void Swift::set_digital_direction(int pin, int value,
                                   std::function<void(VoidResult)> cb)
{
    cmd_void("M2241 N" + std::to_string(pin) + " V" + std::to_string(value),
             std::move(cb));
}

VoidResult Swift::set_digital_direction_sync(int pin, int value,
                                              float timeout_s)
{
    return sync_call<VoidResult>(
        [this, pin, value](auto cb) { set_digital_direction(pin, value, cb); },
        timeout_s);
}

void Swift::get_digital(int pin, std::function<void(IntResult)> cb)
{
    query_int("P2240 N" + std::to_string(pin), std::move(cb));
}

IntResult Swift::get_digital_sync(int pin, float timeout_s)
{
    return sync_call<IntResult>(
        [this, pin](auto cb) { get_digital(pin, cb); }, timeout_s);
}

void Swift::get_analog(int pin, std::function<void(IntResult)> cb)
{
    query_int("P2241 N" + std::to_string(pin), std::move(cb));
}

IntResult Swift::get_analog_sync(int pin, float timeout_s)
{
    return sync_call<IntResult>(
        [this, pin](auto cb) { get_analog(pin, cb); }, timeout_s);
}

} // namespace swiftpro

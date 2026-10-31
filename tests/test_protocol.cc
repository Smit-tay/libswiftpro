// test_protocol.cc
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Protocol tests against a fake firmware on a pty: no arm needed.
//
// The fake is scripted, not a simulation. Each reply it sends is the exact text
// of the firmware's golden traces (SwiftPro-plusplus test/golden/*.txt), so these
// tests check the library against the protocol, not the firmware itself.
//
// Usage: test_protocol <case>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "swiftpro/commander.h"
#include "swiftpro/swiftpro.h"

using namespace std::chrono_literals;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << __FILE__ << ":" << __LINE__ << ": failed: "      \
                      << #cond << "\n";                                   \
            ++g_failures;                                                 \
        }                                                                 \
    } while (false)

#define CHECK_EQ(a, b)                                                    \
    do {                                                                  \
        const auto va_ = (a);                                             \
        const auto vb_ = (b);                                             \
        if (!(va_ == vb_)) {                                              \
            std::cerr << __FILE__ << ":" << __LINE__ << ": " << #a        \
                      << " is '" << va_ << "', expected '" << vb_ << "'\n"; \
            ++g_failures;                                                 \
        }                                                                 \
    } while (false)

bool near(float a, float b) { return std::fabs(a - b) < 1e-3f; }

bool wait_until(const std::function<bool()>& pred,
                std::chrono::milliseconds limit = 5s)
{
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) { return true; }
        std::this_thread::sleep_for(2ms);
    }
    return pred();
}

// A line as the firmware sees it: "#12M2231 V1" is seq "12", body "M2231 V1".
struct Line {
    std::string seq;
    std::string body;
};

Line split_line(const std::string& raw)
{
    if (raw.empty() || raw[0] != '#') { return {"", raw}; }
    size_t i = 1;
    while (i < raw.size() && raw[i] >= '0' && raw[i] <= '9') { ++i; }
    return {raw.substr(1, i - 1), raw.substr(i)};
}

// What the firmware would say. nullopt: nothing, the test answers by hand.
using Script = std::function<std::optional<std::string>(const Line&)>;

// "$N ok", or "ok" for a command sent without a sequence number.
std::string with_seq(const Line& l, const std::string& text)
{
    return l.seq.empty() ? text : "$" + l.seq + " " + text;
}

class FakeArm
{
public:
    explicit FakeArm(Script script = [](const Line& l) -> std::optional<std::string> {
        return with_seq(l, "ok");
    })
    : script_(std::move(script))
    {
        master_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (master_ < 0 || grantpt(master_) != 0 || unlockpt(master_) != 0) {
            std::cerr << "no pty available\n";
            std::exit(2);
        }
        path_ = ptsname(master_);

        // Keep a slave open, raw, so the master doesn't see EIO before the
        // library opens it.
        slave_ = open(path_.c_str(), O_RDWR | O_NOCTTY);
        termios t{};
        tcgetattr(slave_, &t);
        cfmakeraw(&t);
        tcsetattr(slave_, TCSANOW, &t);

        thread_ = std::thread([this] { run(); });
    }

    ~FakeArm() { unplug(); }

    FakeArm(const FakeArm&)            = delete;
    FakeArm& operator=(const FakeArm&) = delete;

    const std::string& path() const { return path_; }

    void say(const std::string& line)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (master_ < 0) { return; }
        const std::string text = line + "\n";
        const ssize_t n = write(master_, text.data(), text.size());
        (void)n;
    }

    std::vector<Line> lines()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lines_;
    }

    // The body of line index, waiting for it to arrive.
    std::string body_at(size_t index)
    {
        wait_until([&] { return lines().size() > index; });
        const auto all = lines();
        return index < all.size() ? all[index].body : "<nothing>";
    }

    // As pulling the USB cable: the library's end of the pty hangs up.
    void unplug()
    {
        stop_ = true;
        if (thread_.joinable()) { thread_.join(); }
        std::lock_guard<std::mutex> lock(mutex_);
        if (slave_ >= 0)  { close(slave_);  slave_  = -1; }
        if (master_ >= 0) { close(master_); master_ = -1; }
    }

private:
    void run()
    {
        std::string buf;
        while (!stop_) {
            pollfd p{master_, POLLIN, 0};
            if (poll(&p, 1, 10) <= 0 || !(p.revents & POLLIN)) { continue; }
            char tmp[256];
            const ssize_t n = read(master_, tmp, sizeof tmp);
            if (n <= 0) { continue; }
            buf.append(tmp, static_cast<size_t>(n));
            size_t pos;
            while ((pos = buf.find('\n')) != std::string::npos) {
                const Line line = split_line(buf.substr(0, pos));
                buf.erase(0, pos + 1);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    lines_.push_back(line);
                }
                if (const auto reply = script_(line)) { say(*reply); }
            }
        }
    }

    Script                script_;
    int                   master_{-1};
    int                   slave_{-1};
    std::string           path_;
    std::thread           thread_;
    std::atomic<bool>     stop_{false};
    std::mutex            mutex_;
    std::vector<Line>     lines_;
};

// ---------------------------------------------------------------------------

// The exact bytes of every command the library can send.
void wire_format()
{
    FakeArm fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    const std::vector<std::pair<std::function<void()>, std::string>> cases = {
        {[&] { arm.set_report_interval_sync(0.1f); },              "M2120 V0.100"},
        {[&] { arm.set_report_interval_sync(0.004f); },            "M2120 V0.004"},
        {[&] { arm.set_report_interval_sync(0.0f); },              "M2120 V0.000"},
        {[&] { arm.set_motion_report_sync(true); },                "M2122 V1"},
        {[&] { arm.set_streaming_sync(true); },                    "M2126 V1"},
        {[&] { arm.set_streaming_sync(false); },                   "M2126 V0"},
        {[&] { arm.set_mode_sync(11); },                           "M2400 S11"},
        {[&] { arm.set_acceleration_sync(5.0f); },                 "M204 A5.0"},
        {[&] { arm.set_position_sync(200, 0, 150, 50); },          "G0 X200.0 Y0.0 Z150.0 F50.0"},
        {[&] { arm.set_position_sync(1, -2, 3, 50, true); },       "G2204 X1.0 Y-2.0 Z3.0 F50.0"},
        {[&] { arm.set_polar_sync(200, 90, 150, 50); },            "G2201 S200.0 R90.0 H150.0 F50.0"},
        {[&] { arm.set_polar_sync(1, 2, 3, 50, true); },           "G2205 S1.0 R2.0 H3.0 F50.0"},
        {[&] { arm.set_joint_angles_sync(90, 45, 60, 20); },       "G2206 B90.0 L45.0 R60.0 F20.0"},
        {[&] { arm.set_servo_angle_sync(1, 45, 20); },             "G2202 N1 V45.0 F20.0"},
        {[&] { arm.set_wrist_sync(90, 20); },                      "G2202 N3 V90.0 F20.0"},
        {[&] { arm.pause_motion_sync(); },                         "S1000 V0"},
        {[&] { arm.resume_motion_sync(); },                        "S1000 V1"},
        {[&] { arm.motion_reset_sync(); },                         "S1100"},
        {[&] { arm.set_servo_attach_sync(-1); },                   "M17"},
        {[&] { arm.set_servo_attach_sync(2); },                    "M2201 N2"},
        {[&] { arm.set_servo_detach_sync(-1); },                   "M2019"},
        {[&] { arm.set_servo_detach_sync(2); },                    "M2202 N2"},
        {[&] { arm.get_servo_attach_sync(2); },                    "M2203 N2"},
        {[&] { arm.set_pump_sync(true); },                         "M2231 V1"},
        {[&] { arm.set_gripper_sync(true); },                      "M2232 V1"},
        {[&] { arm.set_buzzer_sync(1000, 0.25f); },                "M2210 F1000 T250"},
        {[&] { arm.get_position_sync(); },                         "P2220"},
        {[&] { arm.get_polar_sync(); },                            "P2221"},
        {[&] { arm.get_joint_angles_sync(); },                     "P2200"},
        {[&] { arm.get_servo_angle_sync(3); },                     "P2206 N3"},
        {[&] { arm.is_reachable_sync(200, 0, 150); },              "M2222 X200.0 Y0.0 Z150.0 P0"},
        {[&] { arm.coord_to_angles_sync(200, 0, 150); },           "M2220 X200.0 Y0.0 Z150.0"},
        {[&] { arm.angles_to_coord_sync(90, 45, 60); },            "M2221 B90.0 L45.0 R60.0"},
        {[&] { arm.get_pump_status_sync(); },                      "P2231"},
        {[&] { arm.get_gripper_status_sync(); },                   "P2232"},
        {[&] { arm.get_encoder_status_sync(); },                   "P2244"},
        {[&] { arm.get_is_moving_sync(); },                        "M2200"},
        {[&] { arm.get_mode_sync(); },                             "P2400"},
        {[&] { arm.set_digital_output_sync(10, 1); },              "M2240 N10 V1"},
        {[&] { arm.set_digital_direction_sync(10, 1); },           "M2241 N10 V1"},
        {[&] { arm.get_digital_sync(10); },                        "P2240 N10"},
        {[&] { arm.get_analog_sync(3); },                          "P2241 N3"},
    };

    size_t index = 0;
    for (const auto& [call, expected] : cases) {
        call();
        CHECK_EQ(fake.body_at(index), expected);
        CHECK(!fake.lines()[index].seq.empty());
        ++index;
    }

    // Without a callback there is no sequence number.
    arm.set_pump(false);
    CHECK_EQ(fake.body_at(index), std::string("M2231 V0"));
    CHECK(fake.lines()[index].seq.empty());
    ++index;

    // reset: locks, wrist, pump, gripper, then the move; only the move is tracked.
    CHECK(arm.reset_sync(200, 0, 150, 50));
    const std::vector<std::string> reset = {
        "M17", "G2202 N3 V90 F1000", "M2231 V0", "M2232 V0", "G0 X200.0 Y0.0 Z150.0 F50.0"};
    for (const auto& expected : reset) {
        CHECK_EQ(fake.body_at(index), expected);
        ++index;
    }
    CHECK(!fake.lines()[index - 1].seq.empty());
}

// The firmware rejects lines over 48 characters, counted without spaces.
void line_length()
{
    FakeArm fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    arm.set_position_sync(-1000, -1000, -1000, 50000, true);
    arm.set_polar_sync(-1000, -1000, -1000, 50000);
    arm.set_joint_angles_sync(-100, -100, -100, 50000);
    arm.angles_to_coord_sync(-100, -100, -100);
    wait_until([&] { return fake.lines().size() == 4; });

    constexpr size_t LONGEST_SEQ = 10;  // "#999999999"
    for (const auto& line : fake.lines()) {
        std::string stripped = line.body;
        stripped.erase(std::remove(stripped.begin(), stripped.end(), ' '), stripped.end());
        CHECK(stripped.size() + LONGEST_SEQ <= 48);
    }
}

// Every firmware error reaches the caller, as sent.
void error_codes()
{
    std::string code;
    std::mutex  mutex;
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        std::lock_guard<std::mutex> lock(mutex);
        return with_seq(l, code);
    });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    for (const char* const e : {"E20", "E21", "E22", "E23", "E25", "E26", "E27"}) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            code = e;
        }
        const auto r = arm.set_position_sync(200, 0, 150, 50);
        CHECK(!r);
        if (!r) { CHECK_EQ(r.error(), std::string(e)); }
    }
}

void query_parsing()
{
    const std::map<std::string, std::string> answers = {
        {"P2200", "ok B89.9121 L45.6789 R66.0411"},
        {"P2220", "ok X233.6009 Y-0.3583 Z-10.6346"},
        {"P2206 N3", "ok V90.0000"},
        {"M2222 X200.0 Y0.0 Z150.0 P0", "ok V1"},
        {"M2200", "ok V1"},
        {"P2244", "E26 V5"},
        {"P2201", "ok VSwiftPro"},
        {"P2202", "ok V3.3.1"},
        {"P2203", "ok V5.0.0-pp.1"},
        {"P2204", "ok V4.0.5"},
        {"P2205", "ok V"},
    };
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        const auto it = answers.find(l.body);
        return with_seq(l, it == answers.end() ? "E21" : it->second);
    });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    const auto joints = arm.get_joint_angles_sync();
    CHECK(joints);
    if (joints) {
        CHECK(near(joints->at(0), 89.9121f));
        CHECK(near(joints->at(1), 45.6789f));
        CHECK(near(joints->at(2), 66.0411f));
    }

    const auto pos = arm.get_position_sync();
    CHECK(pos);
    if (pos) { CHECK(near(pos->at(1), -0.3583f)); }

    const auto wrist = arm.get_servo_angle_sync(3);
    CHECK(wrist && near(*wrist, 90.0f));

    const auto reachable = arm.is_reachable_sync(200, 0, 150);
    CHECK(reachable && *reachable == 1);

    const auto moving = arm.get_is_moving_sync();
    CHECK(moving && *moving == 1);

    // P2244's fault report is a value: the mask, base and left here.
    const auto encoders = arm.get_encoder_status_sync();
    CHECK(encoders && *encoders == 5);

    const auto info = arm.get_device_info_sync();
    CHECK(info);
    if (info) {
        CHECK_EQ(info->at(0), std::string("SwiftPro"));
        CHECK_EQ(info->at(1), std::string("3.3.1"));
        CHECK_EQ(info->at(2), std::string("5.0.0-pp.1"));
        CHECK_EQ(info->at(3), std::string("4.0.5"));
        CHECK_EQ(info->at(4), std::string(""));  // no UID stored
    }

    // A query that fails is an error, not a value.
    const auto angle = arm.get_servo_angle_sync(9);
    CHECK(!angle);
    if (!angle) { CHECK_EQ(angle.error(), std::string("E21")); }
}

void query_info_error()
{
    FakeArm fake([](const Line& l) -> std::optional<std::string> {
        return with_seq(l, l.body == "P2203" ? "E20" : "ok V1");
    });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));
    const auto info = arm.get_device_info_sync();
    CHECK(!info);
    if (!info) { CHECK_EQ(info.error(), std::string("E20")); }
}

// P/S replies overtake G/M replies: matching is by $N, not by order.
void reply_matching()
{
    std::vector<Line> moves;
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        if (l.body[0] == 'G') { moves.push_back(l); return std::nullopt; }
        return with_seq(l, "ok X1.0000 Y2.0000 Z3.0000");
    });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    std::atomic<int> move_done{0};
    arm.set_position(200, 0, 150, 50, false, [&](swiftpro::VoidResult r) {
        CHECK(r);
        ++move_done;
    });
    CHECK(wait_until([&] { return fake.lines().size() == 1; }));

    const auto pos = arm.get_position_sync();
    CHECK(pos);
    CHECK_EQ(move_done.load(), 0);

    fake.say("$" + fake.lines()[0].seq + " ok");
    CHECK(wait_until([&] { return move_done == 1; }));
}

void report_events()
{
    FakeArm fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    std::mutex         mutex;
    std::vector<float> position, joints;
    std::atomic<int>   motion{0}, limit{-1}, power{-1}, lost_joint{-1};
    std::atomic<float> lost_error{0.0f};

    arm.on_position([&](float x, float y, float z, float w) {
        std::lock_guard<std::mutex> lock(mutex);
        position = {x, y, z, w};
    });
    arm.on_joint([&](float b, float l, float r) {
        std::lock_guard<std::mutex> lock(mutex);
        joints = {b, l, r};
    });
    arm.on_motion_complete([&] { ++motion; });
    arm.on_limit_switch([&](bool s) { limit = s ? 1 : 0; });
    arm.on_power([&](bool p) { power = p ? 1 : 0; });
    arm.on_lost_steps([&](int j, float e) { lost_joint = j; lost_error = e; });

    // Full form: two-letter prefixes, a measured joint.
    fake.say("@3 X233.6009 Y-0.3583 Z-10.6346 R90.0000 B-0.0879 L45.6789 AR66.0411 MB-0.0880 A12 T5000 S7");
    CHECK(wait_until([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return joints.size() == 3;
    }));
    {
        std::lock_guard<std::mutex> lock(mutex);
        CHECK(near(position[0], 233.6009f));
        CHECK(near(position[1], -0.3583f));
        CHECK(near(position[3], 90.0f));
        CHECK(near(joints[0], -0.0879f));
        CHECK(near(joints[1], 45.6789f));
        CHECK(near(joints[2], 66.0411f));  // the AR field
        joints.clear();
    }

    // No good encoder reading: "MR-" and no A.
    fake.say("@3 X1.0000 Y2.0000 Z3.0000 R4.0000 B5.0000 L6.0000 AR7.0000 MR- T5100 S8");
    CHECK(wait_until([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return joints.size() == 3;
    }));
    {
        std::lock_guard<std::mutex> lock(mutex);
        CHECK(near(joints[2], 7.0f));
    }

    fake.say("@9 V0");
    CHECK(wait_until([&] { return motion == 1; }));
    fake.say("@6 N0 V1");
    CHECK(wait_until([&] { return limit == 1; }));
    fake.say("@6 N0 V0");
    CHECK(wait_until([&] { return limit == 0; }));
    fake.say("@5 V1");
    CHECK(wait_until([&] { return power == 1; }));
    fake.say("@8 N1 V-12.3456");
    CHECK(wait_until([&] { return lost_joint == 1; }));
    CHECK(near(lost_error, -12.3456f));

    // Lines that must be ignored, not crash: boot banner, realtime replies, buttons.
    fake.say("device name : SwiftPro\r");
    fake.say("ok @9 V0");
    fake.say("ok");
    fake.say("@4 N0 V1");
    fake.say("@");
    fake.say("@8");
    CHECK(arm.set_pump_sync(true));
}

// golden 06-stop: S1100 "ok" at once, then the moves get E25 in FIFO order.
void stop_during_move()
{
    std::vector<std::string> moves;
    FakeArm* self = nullptr;
    std::mutex mutex;
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        std::lock_guard<std::mutex> lock(mutex);
        if (l.body[0] == 'G') { moves.push_back(l.seq); return std::nullopt; }
        self->say("$" + l.seq + " ok");
        for (const auto& seq : moves) { self->say("$" + seq + " E25"); }
        return std::nullopt;
    });
    self = &fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    std::promise<std::string> first, second;
    arm.set_position(200, 0, 150, 50, false, [&](swiftpro::VoidResult r) {
        first.set_value(r ? "ok" : r.error());
    });
    arm.set_position(210, 0, 150, 50, false, [&](swiftpro::VoidResult r) {
        second.set_value(r ? "ok" : r.error());
    });
    CHECK(wait_until([&] { return fake.lines().size() == 2; }));

    CHECK(arm.motion_reset_sync());
    CHECK_EQ(first.get_future().get(), std::string("E25"));
    CHECK_EQ(second.get_future().get(), std::string("E25"));
}

// A stop overtakes a full window, and what was queued behind it never runs.
void stop_overtakes_queue()
{
    std::vector<std::string> moves;
    std::mutex mutex;
    FakeArm* self = nullptr;
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        std::lock_guard<std::mutex> lock(mutex);
        if (l.body[0] == 'G') { moves.push_back(l.seq); return std::nullopt; }
        self->say("$" + l.seq + " ok");
        for (const auto& seq : moves) { self->say("$" + seq + " E25"); }
        return std::nullopt;
    });
    self = &fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    const int total = static_cast<int>(swiftpro::PENDING_CAPACITY) + 5;
    std::atomic<int> stopped{0};
    for (int i = 0; i < total; ++i) {
        arm.set_position(200, 0, 150, 50, false, [&](swiftpro::VoidResult r) {
            if (!r && r.error() == "STOPPED") { ++stopped; }
        });
    }
    CHECK(wait_until([&] { return fake.lines().size() == swiftpro::PENDING_CAPACITY; }));

    const auto start = std::chrono::steady_clock::now();
    CHECK(arm.motion_reset_sync());
    CHECK(std::chrono::steady_clock::now() - start < 200ms);

    // Nothing queued gets written after the stop.
    std::this_thread::sleep_for(300ms);
    CHECK_EQ(stopped.load(), 5);
    CHECK_EQ(fake.lines().size(), swiftpro::PENDING_CAPACITY + 1);
    arm.disconnect();
}

// Resume must get past the moves that a hold keeps unanswered.
void resume_overtakes_queue()
{
    FakeArm* self = nullptr;
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        if (l.body[0] == 'G') { return std::nullopt; }
        self->say("$" + l.seq + " ok");
        return std::nullopt;
    });
    self = &fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    for (size_t i = 0; i < swiftpro::PENDING_CAPACITY + 2; ++i) {
        arm.set_position(200, 0, 150, 50, false);
    }
    std::atomic<int> answered{0};
    for (size_t i = 0; i < swiftpro::PENDING_CAPACITY + 2; ++i) {
        arm.set_position(200, 0, 150, 50, false, [&](swiftpro::VoidResult) { ++answered; });
    }
    CHECK(wait_until([&] { return fake.lines().size() >= swiftpro::PENDING_CAPACITY; }));

    CHECK(arm.pause_motion_sync(1.0f));
    CHECK(arm.resume_motion_sync(1.0f));
    arm.disconnect();
    CHECK_EQ(answered.load(), static_cast<int>(swiftpro::PENDING_CAPACITY) + 2);
}

// A reboot loses everything in flight: fail it now, and carry on.
void reboot_fails_pending()
{
    std::atomic<bool> answer{false};
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        if (!answer) { return std::nullopt; }
        return with_seq(l, "ok");
    });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    std::promise<std::string> result;
    arm.set_position(200, 0, 150, 50, false, [&](swiftpro::VoidResult r) {
        result.set_value(r ? "ok" : r.error());
    });
    CHECK(wait_until([&] { return fake.lines().size() == 1; }));

    fake.say("device name : SwiftPro\r");
    fake.say("soft ver : V5.0.0-pp.1\r");
    fake.say("api ver : V4.0.5\r");
    fake.say("work mode : 0\r");
    fake.say("@1");
    CHECK_EQ(result.get_future().get(), std::string("FIRMWARE_RESET"));

    answer = true;
    CHECK(arm.set_pump_sync(true));
    CHECK(arm.is_connected());
}

// A move outlasting its sync timeout: the caller gets TIMEOUT, the late reply
// is harmless, and the slot is freed.
void late_reply_after_timeout()
{
    std::atomic<bool> answer{false};
    FakeArm fake([&](const Line& l) -> std::optional<std::string> {
        if (!answer) { return std::nullopt; }
        return with_seq(l, "ok");
    });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));

    const auto r = arm.set_position_sync(200, 0, 150, 50, false, 0.05f);
    CHECK(!r);
    if (!r) { CHECK_EQ(r.error(), std::string("TIMEOUT")); }

    fake.say("$" + fake.lines()[0].seq + " ok");
    answer = true;
    CHECK(arm.set_pump_sync(true));
}

// A bare E23 (RX overflow, golden 03-framing) belongs to no command we can name.
void bare_error_keeps_running()
{
    FakeArm fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));
    fake.say("E23");
    fake.say("E21");
    CHECK(arm.set_pump_sync(true));
    CHECK(arm.is_connected());
}

// Replies that never come must not block the writer for good.
void stale_pending_is_freed()
{
    FakeArm fake([](const Line&) -> std::optional<std::string> { return std::nullopt; });

    serial::Serial ser(fake.path(), 115200, serial::Timeout::simpleTimeout(100));
    swiftpro::Commander commander(ser, 0.5f);
    commander.start();

    std::atomic<int> failed{0};
    for (size_t i = 0; i < swiftpro::PENDING_CAPACITY + 1; ++i) {
        commander.send("P2201", [&](bool ok, const std::string& resp) {
            CHECK(!ok);
            CHECK(resp == "TIMEOUT" || resp == "DISCONNECTED");  // the latter at stop()
            ++failed;
        });
    }

    // The map fills; the extra command can only be written after an eviction.
    CHECK(wait_until([&] {
        return fake.lines().size() == swiftpro::PENDING_CAPACITY + 1;
    }));
    CHECK(failed >= 1);
    commander.stop();
    CHECK_EQ(failed.load(), static_cast<int>(swiftpro::PENDING_CAPACITY + 1));
}

void link_loss_and_reconnect()
{
    auto fake = std::make_unique<FakeArm>(
        [](const Line& l) -> std::optional<std::string> {
            if (l.body[0] == 'G') { return std::nullopt; }
            return with_seq(l, "ok");
        });
    swiftpro::Swift arm;
    CHECK(arm.connect(fake->path()));
    CHECK(arm.is_connected());

    std::promise<std::string> result;
    arm.set_position(200, 0, 150, 50, false, [&](swiftpro::VoidResult r) {
        result.set_value(r ? "ok" : r.error());
    });
    CHECK(wait_until([&] { return fake->lines().size() == 1; }));

    fake->unplug();
    CHECK(wait_until([&] { return !arm.is_connected(); }));
    CHECK_EQ(result.get_future().get(), std::string("DISCONNECTED"));

    FakeArm replacement;
    CHECK(arm.connect(replacement.path()));
    CHECK(arm.is_connected());
    CHECK(arm.set_pump_sync(true));
}

void connect_twice()
{
    FakeArm fake;
    swiftpro::Swift arm;
    CHECK(arm.connect(fake.path()));
    CHECK(arm.set_pump_sync(true));
    CHECK(arm.connect(fake.path()));
    CHECK(arm.set_pump_sync(true));
    arm.disconnect();
    arm.disconnect();
    CHECK(!arm.is_connected());
}

void not_connected()
{
    swiftpro::Swift arm;
    CHECK(!arm.is_connected());

    const auto r = arm.set_position_sync(200, 0, 150, 50);
    CHECK(!r);
    if (!r) { CHECK_EQ(r.error(), std::string("NOT_CONNECTED")); }

    const auto q = arm.get_position_sync();
    CHECK(!q);

    const auto raw = arm.send_raw_sync("P2201");
    CHECK(!raw);

    arm.set_pump(true);                    // fire-and-forget: nothing to report
    arm.on_position([](float, float, float, float) {});  // no connection: no-op
    CHECK(!arm.connect("/dev/does-not-exist"));
    CHECK(!arm.is_connected());
}

// Run under ASan and TSan: connect and disconnect while callers and reports run.
void disconnect_under_traffic()
{
    FakeArm fake;
    swiftpro::Swift arm;

    std::atomic<bool> running{true};
    std::thread reports([&] {
        while (running) {
            fake.say("@3 X1.0000 Y2.0000 Z3.0000 R4.0000 B5.0000 L6.0000 AR7.0000 MB5.0000 A1 T1 S1");
            fake.say("$999999 ok");  // keeps the receiver calling into the commander
            std::this_thread::sleep_for(100us);
        }
    });
    std::thread caller_a([&] {
        while (running) { arm.get_position_sync(0.05f); }
    });
    std::thread caller_b([&] {
        while (running) {
            arm.set_pump(true);
            arm.is_connected();
            std::this_thread::sleep_for(1ms);
        }
    });

    for (int i = 0; i < 30; ++i) {
        CHECK(arm.connect(fake.path()));
        arm.on_position([](float, float, float, float) {});
        std::this_thread::sleep_for(5ms);
        arm.disconnect();
    }

    running = false;
    reports.join();
    caller_a.join();
    caller_b.join();
}

}  // namespace

int main(int argc, char* argv[])
{
    const std::map<std::string, void (*)()> cases = {
        {"wire_format", wire_format},
        {"line_length", line_length},
        {"error_codes", error_codes},
        {"query_parsing", [] { query_parsing(); query_info_error(); }},
        {"reply_matching", reply_matching},
        {"report_events", report_events},
        {"stop_during_move", stop_during_move},
        {"stop_overtakes_queue", stop_overtakes_queue},
        {"resume_overtakes_queue", resume_overtakes_queue},
        {"reboot_fails_pending", reboot_fails_pending},
        {"late_reply_after_timeout", late_reply_after_timeout},
        {"bare_error_keeps_running", bare_error_keeps_running},
        {"stale_pending_is_freed", stale_pending_is_freed},
        {"link_loss_and_reconnect", link_loss_and_reconnect},
        {"connect_twice", connect_twice},
        {"not_connected", not_connected},
        {"disconnect_under_traffic", disconnect_under_traffic},
    };

    const auto it = argc == 2 ? cases.find(argv[1]) : cases.end();
    if (it == cases.end()) {
        std::cerr << "usage: test_protocol <case>\n";
        return 2;
    }

    it->second();
    if (g_failures != 0) {
        std::cerr << g_failures << " check(s) failed\n";
        return 1;
    }
    return 0;
}

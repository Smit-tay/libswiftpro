// teach_playback_joint.cc — replay a joint-space teach recording with
// cosine interpolation and windowed pipeline flow control.
//
// Usage: teach_playback_joint [port] [input_file] [speed_factor] [interp_steps] [window] [method]
//   port          defaults to /dev/ttyACM0
//   input_file    defaults to teach_joint.urec
//   speed_factor  playback speed multiplier (default 1.0)
//   interp_steps  interpolation steps between waypoints (default 1)
//   window        number of commands in-flight at once (default 4)
//   method        interpolation method: cosine (default) or catmull
//
// Flow control — windowed pipeline:
//   Up to `window` G2206 commands are in-flight simultaneously. Each time a
//   firmware "ok" is received, the next command is sent to refill the pipeline.
//   This keeps the GRBL planner buffer primed for junction blending (smooth
//   motion) while preventing serial buffer overflow (deterministic ordering).
//
//   window=1  → one command at a time, perfectly faithful but ~2x too slow
//   window=4  → 4 commands queued, GRBL blends junctions, near real-time speed
//   window=24 → full planner buffer, maximum smoothness, fastest playback
//
// Stationary filter:
//   Consecutive samples where all joints differ by less than STATIONARY_THRESHOLD
//   degrees are collapsed to two samples (start + end of the stationary period).
//   This eliminates encoder quantisation noise during still periods while
//   preserving the recorded sit-still duration during playback.

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>

#include "swiftpro/swiftpro.h"

// ---------------------------------------------------------------------------

static constexpr float STATIONARY_THRESHOLD = 0.5f;  // degrees

struct JointSample {
    int64_t t_ms;
    float   b, l, r;
};

static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running.store(false); }

// ---------------------------------------------------------------------------

static std::vector<JointSample> load_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Failed to open input file: " << path << "\n";
        return {};
    }

    std::vector<JointSample> samples;
    JointSample              cur{};
    int                      fields_read = 0;
    std::string              line;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            if (fields_read == 4) {
                samples.push_back(cur);
                cur = {};
                fields_read = 0;
            }
            continue;
        }

        const auto colon = line.find(':');
        if (colon == std::string::npos) { continue; }

        const std::string key = line.substr(0, colon);
        const std::string val = line.substr(colon + 2);

        try {
            if      (key == "t") { cur.t_ms = std::stoll(val); ++fields_read; }
            else if (key == "b") { cur.b    = std::stof(val);  ++fields_read; }
            else if (key == "l") { cur.l    = std::stof(val);  ++fields_read; }
            else if (key == "r") { cur.r    = std::stof(val);  ++fields_read; }
        } catch (...) {
            std::cerr << "Parse error on line: " << line << "\n";
        }
    }

    if (fields_read == 4) { samples.push_back(cur); }
    return samples;
}

// ---------------------------------------------------------------------------
// Stationary filter.
//
// Collapses runs of near-identical samples to two samples (first + last),
// preserving the timing gap so the arm sits still for the recorded duration.
// Moving samples are kept verbatim.
// ---------------------------------------------------------------------------

static std::vector<JointSample> filter_stationary(
    const std::vector<JointSample>& in,
    float threshold = STATIONARY_THRESHOLD)
{
    if (in.empty()) { return in; }

    std::vector<JointSample> out;
    out.push_back(in.front());

    bool was_stationary = false;

    for (size_t i = 1; i < in.size(); ++i) {
        const auto& prev = in[i-1];
        const auto& cur  = in[i];

        const bool moving =
            std::abs(cur.b - prev.b) > threshold ||
            std::abs(cur.l - prev.l) > threshold ||
            std::abs(cur.r - prev.r) > threshold;

        if (moving) {
            if (was_stationary) {
                // Emit the last stationary sample to anchor the sit-still
                // duration before the motion begins.
                out.push_back(prev);
            }
            out.push_back(cur);
            was_stationary = false;
        } else {
            was_stationary = true;
        }
    }

    // Always keep the final sample.
    if (out.back().t_ms != in.back().t_ms) {
        out.push_back(in.back());
    }

    return out;
}

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Interpolation methods
// ---------------------------------------------------------------------------

// Cosine ease-in/ease-out — smooth within each segment but velocity
// discontinuity at segment junctions (arm micro-stops between waypoints).
static float cosine_interp(float a, float b, float t)
{
    const float s = 0.5f * (1.0f - std::cos(static_cast<float>(M_PI) * t));
    return a + s * (b - a);
}

// Catmull-Rom spline — continuous velocity at junctions (no micro-stops).
// Interpolates between p1 and p2 using p0 and p3 as tangent anchors.
static float catmull_rom(float p0, float p1, float p2, float p3, float t)
{
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f*p0 - 5.0f*p1 + 4.0f*p2 - p3) * t2 +
        (-p0 + 3.0f*p1 - 3.0f*p2 + p3) * t3
    );
}

// Cosine interpolation between two samples.
static std::vector<JointSample> interpolate_cosine(const JointSample& a,
                                                    const JointSample& b,
                                                    int steps)
{
    std::vector<JointSample> result;
    result.reserve(steps);
    for (int i = 1; i <= steps; ++i) {
        const float   t    = static_cast<float>(i) / steps;
        const int64_t t_ms = a.t_ms + static_cast<int64_t>(
            t * static_cast<float>(b.t_ms - a.t_ms));
        result.push_back({
            t_ms,
            cosine_interp(a.b, b.b, t),
            cosine_interp(a.l, b.l, t),
            cosine_interp(a.r, b.r, t)
        });
    }
    return result;
}

// Catmull-Rom interpolation for a full recording.
// Requires all 4 samples (p0..p3); handles endpoints by reflecting.
// Returns the complete interpolated trajectory in one pass.
static std::vector<JointSample> interpolate_catmull(
    const std::vector<JointSample>& recorded, int steps)
{
    if (recorded.size() < 2) { return recorded; }

    std::vector<JointSample> result;
    result.push_back(recorded.front());

    const size_t n = recorded.size();

    for (size_t seg = 0; seg + 1 < n; ++seg) {
        // Clamp indices to valid range for endpoint handling.
        const size_t i0 = (seg == 0)     ? 0     : seg - 1;
        const size_t i1 = seg;
        const size_t i2 = seg + 1;
        const size_t i3 = (seg + 2 < n)  ? seg+2 : n - 1;

        const auto& p0 = recorded[i0];
        const auto& p1 = recorded[i1];
        const auto& p2 = recorded[i2];
        const auto& p3 = recorded[i3];

        for (int i = 1; i <= steps; ++i) {
            const float   t    = static_cast<float>(i) / steps;
            const int64_t t_ms = p1.t_ms + static_cast<int64_t>(
                t * static_cast<float>(p2.t_ms - p1.t_ms));
            result.push_back({
                t_ms,
                catmull_rom(p0.b, p1.b, p2.b, p3.b, t),
                catmull_rom(p0.l, p1.l, p2.l, p3.l, t),
                catmull_rom(p0.r, p1.r, p2.r, p3.r, t)
            });
        }
    }

    return result;
}

enum class InterpMethod { COSINE, CATMULL_ROM };

// ---------------------------------------------------------------------------
// PlaybackState
// ---------------------------------------------------------------------------

struct PlaybackState {
    swiftpro::Swift&                          arm;
    const std::vector<JointSample>&       trajectory;
    float                                 speed_factor;
    int                                   window;
    std::chrono::steady_clock::time_point wall_start;

    std::atomic<size_t>     send_idx{0};
    std::atomic<int>        in_flight{0};
    std::atomic<bool>       done{false};
    std::mutex              done_mutex;
    std::condition_variable done_cv;

    PlaybackState(swiftpro::Swift& arm_,
                  const std::vector<JointSample>& traj_,
                  float speed_,
                  int window_,
                  std::chrono::steady_clock::time_point start_)
    : arm(arm_)
    , trajectory(traj_)
    , speed_factor(speed_)
    , window(window_)
    , wall_start(start_)
    {}
};

// ---------------------------------------------------------------------------

static void send_one(std::shared_ptr<PlaybackState> state, size_t i)
{
    const auto& wp = state->trajectory[i];

    // Sleep if we're ahead of the recorded timeline.
    const int64_t wall_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - state->wall_start).count();
    const int64_t playback_ms =
        static_cast<int64_t>(wall_ms * state->speed_factor);
    const int64_t ahead_ms = wp.t_ms - playback_ms;

    if (ahead_ms > 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ahead_ms));
    }

    if (i % 50 == 0) {
        std::cout << "\r  wp " << i + 1 << "/" << state->trajectory.size()
                  << "  b=" << std::fixed << std::setprecision(1)
                  << wp.b << " l=" << wp.l << " r=" << wp.r
                  << "  t=" << wp.t_ms << "ms     " << std::flush;
    }

    state->in_flight.fetch_add(1);

    state->arm.set_joint_angles(wp.b, wp.l, wp.r, 20.0f,
        [state](swiftpro::VoidResult result) {
            if (!result) {
                std::cerr << "\nG2206 error: " << result.error() << "\n";
            }

            state->in_flight.fetch_sub(1);

            if (!g_running.load()) {
                std::lock_guard<std::mutex> lock(state->done_mutex);
                state->done.store(true);
                state->done_cv.notify_all();
                return;
            }

            const size_t next = state->send_idx.fetch_add(1);
            if (next >= state->trajectory.size()) {
                if (state->in_flight.load() == 0) {
                    std::lock_guard<std::mutex> lock(state->done_mutex);
                    state->done.store(true);
                    state->done_cv.notify_all();
                }
                return;
            }

            send_one(state, next);
        });
}

static void fill_pipeline(std::shared_ptr<PlaybackState> state)
{
    const size_t to_send = std::min(
        static_cast<size_t>(state->window),
        state->trajectory.size());

    for (size_t i = 0; i < to_send; ++i) {
        const size_t idx = state->send_idx.fetch_add(1);
        if (idx >= state->trajectory.size()) { break; }
        send_one(state, idx);
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    const std::string port         = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string infile       = (argc > 2) ? argv[2] : "teach_joint.urec";
    const float       speed_factor = (argc > 3) ? std::stof(argv[3]) : 1.0f;
    const int         interp_steps = (argc > 4) ? std::stoi(argv[4]) : 1;
    const int         window       = (argc > 5) ? std::stoi(argv[5]) : 4;
    const InterpMethod method      = (argc > 6 && std::string(argv[6]) == "catmull")
                                     ? InterpMethod::CATMULL_ROM
                                     : InterpMethod::COSINE;

    if (speed_factor <= 0.0f) {
        std::cerr << "speed_factor must be > 0\n";
        return 1;
    }
    if (window < 1 || window > 24) {
        std::cerr << "window must be 1-24\n";
        return 1;
    }
    std::cout << "Interpolation: "
              << (method == InterpMethod::CATMULL_ROM ? "Catmull-Rom" : "Cosine")
              << "\n";

    struct sigaction sa{};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Load, filter, and interpolate.
    std::cout << "Loading recording from: " << infile << "\n";
    const auto raw = load_file(infile);
    if (raw.empty()) {
        std::cerr << "No samples loaded.\n";
        return 1;
    }
    std::cout << "Loaded " << raw.size() << " raw samples  ("
              << std::fixed << std::setprecision(1)
              << raw.back().t_ms / 1000.0f << "s)\n";

    const auto recorded = filter_stationary(raw);
    std::cout << "After stationary filter: " << recorded.size()
              << " samples  (" << std::setprecision(1)
              << 100.0f * recorded.size() / raw.size() << "% of raw)\n";

    std::vector<JointSample> trajectory;
    if (method == InterpMethod::CATMULL_ROM) {
        trajectory = interpolate_catmull(recorded, interp_steps);
    } else {
        trajectory.push_back(recorded.front());
        for (size_t i = 0; i + 1 < recorded.size(); ++i) {
            auto interp = interpolate_cosine(recorded[i], recorded[i+1], interp_steps);
            for (auto& s : interp) { trajectory.push_back(s); }
        }
    }

    std::cout << "Interpolated to " << trajectory.size()
              << " waypoints (" << interp_steps
              << " steps between samples)\n";
    std::cout << "Speed factor: " << speed_factor << "x"
              << "  window: " << window << "\n\n";

    // Connect.
    swiftpro::Swift arm;
    std::cout << "Connecting to " << port << " ...\n";
    if (!arm.connect(port)) {
        std::cerr << "connect() failed\n";
        return 1;
    }
    std::cout << "Connected. Waiting for firmware boot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    arm.set_motion_report_sync(true);

    // Move to start position synchronously.
    std::cout << "Moving to start position...\n";
    const auto& first = trajectory.front();
    arm.set_joint_angles_sync(first.b, first.l, first.r, 20.0f);
    std::cout << "At start. Beginning playback...\n\n";

    // Create state and prime the pipeline.
    auto state = std::make_shared<PlaybackState>(
        arm, trajectory, speed_factor, window,
        std::chrono::steady_clock::now());

    fill_pipeline(state);

    // Wait for all waypoints to be sent and acknowledged.
    {
        std::unique_lock<std::mutex> lock(state->done_mutex);
        state->done_cv.wait(lock, [&] { return state->done.load(); });
    }

    std::cout << "\n\nPlayback complete.\n";
    arm.disconnect();
    std::cout << "Done.\n";
    return 0;
}

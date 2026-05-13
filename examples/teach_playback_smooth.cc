// teach_playback_smooth.cc — replay a teach recording with trajectory
// simplification via Ramer-Douglas-Peucker, then smooth playback via
// GRBL junction blending.
//
// Usage: teach_playback_smooth [port] [input_file] [speed_factor] [rdp_tolerance_mm]
//   port              defaults to /dev/ttyACM0
//   input_file        defaults to teach_recording.urec
//   speed_factor      playback speed multiplier (default 1.0)
//   rdp_tolerance_mm  RDP simplification tolerance in mm (default 2.0)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <string>
#include <thread>
#include <vector>

#include "swiftpro/swiftpro.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr int64_t LOOKAHEAD_MS               = 500;
static constexpr float   JUNCTION_DEVIATION_SMOOTH  = 0.5f;
static constexpr float   JUNCTION_DEVIATION_DEFAULT = 0.050f;

// ---------------------------------------------------------------------------

struct Sample {
    int64_t t_ms;
    float   x, y, z, w;
};

// Simple flag — set by signal handler, checked in main loop.
static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running.store(false); }

// ---------------------------------------------------------------------------
// File loader
// ---------------------------------------------------------------------------

static std::vector<Sample> load_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Failed to open input file: " << path << "\n";
        return {};
    }

    std::vector<Sample> samples;
    Sample              cur{};
    int                 fields_read = 0;
    std::string         line;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            if (fields_read == 5) {
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
            else if (key == "x") { cur.x    = std::stof(val);  ++fields_read; }
            else if (key == "y") { cur.y    = std::stof(val);  ++fields_read; }
            else if (key == "z") { cur.z    = std::stof(val);  ++fields_read; }
            else if (key == "w") { cur.w    = std::stof(val);  ++fields_read; }
        } catch (...) {
            std::cerr << "Parse error on line: " << line << "\n";
        }
    }

    if (fields_read == 5) { samples.push_back(cur); }
    return samples;
}

// ---------------------------------------------------------------------------
// Ramer-Douglas-Peucker
// ---------------------------------------------------------------------------

static float point_to_segment_dist(const Sample& p,
                                    const Sample& a,
                                    const Sample& b)
{
    const float abx = b.x - a.x;
    const float aby = b.y - a.y;
    const float abz = b.z - a.z;
    const float ab2 = abx*abx + aby*aby + abz*abz;

    if (ab2 < 1e-10f) {
        const float dx = p.x - a.x;
        const float dy = p.y - a.y;
        const float dz = p.z - a.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    const float t  = std::clamp(
        ((p.x-a.x)*abx + (p.y-a.y)*aby + (p.z-a.z)*abz) / ab2,
        0.0f, 1.0f);
    const float dx = p.x - (a.x + t*abx);
    const float dy = p.y - (a.y + t*aby);
    const float dz = p.z - (a.z + t*abz);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static void rdp_recursive(const std::vector<Sample>& s,
                           size_t start, size_t end,
                           float tol, std::vector<bool>& keep)
{
    if (end <= start + 1) { return; }

    float  max_d = 0.0f;
    size_t max_i = start;

    for (size_t i = start + 1; i < end; ++i) {
        const float d = point_to_segment_dist(s[i], s[start], s[end]);
        if (d > max_d) { max_d = d; max_i = i; }
    }

    if (max_d > tol) {
        keep[max_i] = true;
        rdp_recursive(s, start,  max_i, tol, keep);
        rdp_recursive(s, max_i,  end,   tol, keep);
    }
}

static std::vector<Sample> rdp_simplify(const std::vector<Sample>& samples,
                                         float tolerance)
{
    if (samples.size() <= 2) { return samples; }

    std::vector<bool> keep(samples.size(), false);
    keep.front() = keep.back() = true;
    rdp_recursive(samples, 0, samples.size() - 1, tolerance, keep);

    std::vector<Sample> result;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (keep[i]) { result.push_back(samples[i]); }
    }
    return result;
}

// ---------------------------------------------------------------------------

static float sample_speed(const Sample& a, const Sample& b, float speed_factor)
{
    const int64_t dt_ms = b.t_ms - a.t_ms;
    if (dt_ms <= 0) { return 500.0f; }
    const float dx   = b.x - a.x;
    const float dy   = b.y - a.y;
    const float dz   = b.z - a.z;
    const float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    const float mms  = dist / (dt_ms / 1000.0f);
    return std::clamp(mms * 60.0f * speed_factor, 10.0f, 5000.0f);
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    const std::string port          = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string infile        = (argc > 2) ? argv[2] : "teach_recording.urec";
    const float       speed_factor  = (argc > 3) ? std::stof(argv[3]) : 1.0f;
    const float       rdp_tolerance = (argc > 4) ? std::stof(argv[4]) : 2.0f;

    if (speed_factor <= 0.0f) {
        std::cerr << "speed_factor must be > 0\n";
        return 1;
    }

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // Load and simplify.
    std::cout << "Loading recording from: " << infile << "\n";
    const auto raw = load_file(infile);
    if (raw.empty()) { std::cerr << "No samples loaded.\n"; return 1; }

    const auto samples = rdp_simplify(raw, rdp_tolerance);
    std::cout << "Raw: " << raw.size() << " samples  →  RDP: "
              << samples.size() << " waypoints  ("
              << std::fixed << std::setprecision(1)
              << 100.0f * samples.size() / raw.size() << "% of original, "
              << "tolerance=" << rdp_tolerance << "mm)\n";
    std::cout << "Duration: " << raw.back().t_ms / 1000.0f << "s"
              << "  speed: " << speed_factor << "x\n\n";

    // Connect.
    swiftpro::Swift arm;
    std::cout << "Connecting to " << port << " ...\n";
    if (!arm.connect(port)) { std::cerr << "connect() failed\n"; return 1; }
    std::cout << "Connected. Waiting for firmware boot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    arm.set_motion_report_sync(true);

    // Increase junction deviation.
    std::cout << "Setting $11=" << JUNCTION_DEVIATION_SMOOTH << "...\n";
    arm.send_raw_sync("$11=" + std::to_string(JUNCTION_DEVIATION_SMOOTH));

    // Move to start.
    std::cout << "Moving to start position...\n";
    arm.set_position_sync(samples.front().x,
                          samples.front().y,
                          samples.front().z, 50.0f);
    arm.set_wrist_sync(samples.front().w, 1000.0f);
    std::cout << "At start. Beginning smooth playback...\n\n";

    // -----------------------------------------------------------------------
    // Playback loop — lookahead pipeline.
    // -----------------------------------------------------------------------

    const auto wall_start = std::chrono::steady_clock::now();
    size_t send_idx = 0;

    while (g_running.load() && send_idx < samples.size()) {
        const int64_t now_wall_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wall_start).count();

        const int64_t playback_ms =
            static_cast<int64_t>(now_wall_ms * speed_factor);

        while (send_idx < samples.size() &&
               samples[send_idx].t_ms <= playback_ms + LOOKAHEAD_MS)
        {
            const auto& s = samples[send_idx];
            const float spd = (send_idx + 1 < samples.size())
                ? sample_speed(s, samples[send_idx + 1], speed_factor)
                : 50.0f;

            arm.set_position(s.x, s.y, s.z, spd);

            std::cout << "\r  wp " << send_idx + 1 << "/" << samples.size()
                      << "  (" << std::fixed << std::setprecision(1)
                      << s.x << ", " << s.y << ", " << s.z << ") mm"
                      << "  t=" << s.t_ms << "ms"
                      << "  spd=" << std::setprecision(0) << spd << "   "
                      << std::flush;
            ++send_idx;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    std::cout << "\n";

    // -----------------------------------------------------------------------
    // Wait for motion complete — using shared_ptr so the callback is safe
    // even if this scope exits before the callback fires.
    // -----------------------------------------------------------------------

    if (g_running.load()) {
        std::cout << "Waiting for motion to finish...\n";

        auto motion_done  = std::make_shared<std::atomic<bool>>(false);
        auto motion_mutex = std::make_shared<std::mutex>();
        auto motion_cv    = std::make_shared<std::condition_variable>();

        arm.on_motion_complete([motion_done, motion_cv]() {
            motion_done->store(true);
            motion_cv->notify_all();
        });

        std::unique_lock<std::mutex> lock(*motion_mutex);
        motion_cv->wait_for(lock, std::chrono::seconds(15),
            [&motion_done] { return motion_done->load(); });
    }

    // -----------------------------------------------------------------------
    // Cleanup — always runs regardless of how we exit the loop.
    // -----------------------------------------------------------------------

    std::cout << "Restoring $11=" << JUNCTION_DEVIATION_DEFAULT << "...\n";
    arm.send_raw_sync("$11=" + std::to_string(JUNCTION_DEVIATION_DEFAULT));

    arm.disconnect();
    std::cout << "Done.\n";
    return 0;
}

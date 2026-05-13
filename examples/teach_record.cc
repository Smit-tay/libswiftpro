// teach_record.cc — put the arm in passive (backdriveable) mode, record
// position at ~20 Hz to a file, then re-engage motors and disconnect.
//
// Usage: teach_record [port] [output_file]
//   port         defaults to /dev/ttyACM0
//   output_file  defaults to teach_recording.urec
//
// Press Ctrl-C to stop recording. File is written after the loop exits.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
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

struct Sample {
    int64_t t_ms;
    float   x, y, z, w;
};

static std::vector<Sample> g_samples;
static std::mutex          g_mutex;
static std::atomic<bool>   g_recording{false};
static std::atomic<bool>   g_running{true};

static std::chrono::steady_clock::time_point g_start;
static std::atomic<bool>                     g_started{false};

// ---------------------------------------------------------------------------

static void sig_handler(int) { g_running.store(false); }

static void write_file(const std::string& path, const std::string& port)
{
    std::ofstream f(path);
    if (!f) {
        std::cerr << "Failed to open output file: " << path << "\n";
        return;
    }

    const auto now    = std::chrono::system_clock::now();
    const auto now_t  = std::chrono::system_clock::to_time_t(now);
    const auto now_tm = *std::localtime(&now_t);

    std::lock_guard<std::mutex> lock(g_mutex);

    f << "# libuarm teach recording\n";
    f << "# recorded: " << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
    f << "# port: " << port << "\n";
    f << "# samples: " << g_samples.size() << "\n";
    f << "# fields: t_ms, x, y, z, wrist_deg\n";
    f << "\n";

    for (size_t i = 0; i < g_samples.size(); ++i) {
        const auto&   s  = g_samples[i];
        const int64_t dt = (i == 0) ? 0 : (s.t_ms - g_samples[i-1].t_ms);

        if (i == 0) {
            f << "# sample " << i << "\n";
        } else {
            f << "# sample " << i << "  (dt: " << dt << "ms)\n";
        }

        f << std::fixed << std::setprecision(4);
        f << "t: " << s.t_ms << "\n";
        f << "x: " << s.x    << "\n";
        f << "y: " << s.y    << "\n";
        f << "z: " << s.z    << "\n";
        f << "w: " << s.w    << "\n";
        f << "\n";
    }

    std::cout << "Wrote " << g_samples.size()
              << " samples to " << path << "\n";
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    const std::string port    = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string outfile = (argc > 2) ? argv[2] : "teach_recording.urec";

    // Use sigaction with no SA_RESTART so sleep_for is interrupted by Ctrl-C.
    struct sigaction sa{};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // no SA_RESTART
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    swiftpro::Swift arm;

    std::cout << "Connecting to " << port << " ...\n";
    if (!arm.connect(port)) {
        std::cerr << "connect() failed\n";
        return 1;
    }
    std::cout << "Connected. Waiting for firmware boot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    arm.on_position([](float x, float y, float z, float w) {
        if (!g_recording.load()) { return; }

        const auto now = std::chrono::steady_clock::now();
        if (!g_started.exchange(true)) { g_start = now; }

        const int64_t t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - g_start).count();

        std::lock_guard<std::mutex> lock(g_mutex);
        g_samples.push_back({t_ms, x, y, z, w});
    });

    // 20 Hz report interval.
    arm.set_report_interval_sync(0.05f);

    std::cout << "Detaching servos (passive/teach mode)...\n";
    arm.set_servo_detach_sync(-1);
    std::cout << "Arm is now backdriveable. Move it by hand.\n";
    std::cout << "Recording to: " << outfile << "\n";
    std::cout << "Press Ctrl-C to stop.\n\n";

    g_recording.store(true);

    // Status loop — sleep_for is interrupted by SIGINT due to no SA_RESTART.
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!g_running.load()) { break; }
        std::lock_guard<std::mutex> lock(g_mutex);
        if (!g_samples.empty()) {
            const auto& s = g_samples.back();
            std::cout << "\r  samples: " << g_samples.size()
                      << "  pos: (" << std::fixed << std::setprecision(1)
                      << s.x << ", " << s.y << ", " << s.z << ") mm   "
                      << std::flush;
        }
    }

    g_recording.store(false);
    std::cout << "\n";

    // Cleanup — always runs, no atexit.
    std::cout << "Re-attaching servos...\n";
    arm.set_servo_attach_sync(-1);
    arm.set_report_interval_sync(0.0f);
    arm.disconnect();

    write_file(outfile, port);

    return 0;
}

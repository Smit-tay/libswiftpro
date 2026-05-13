// teach_record_joint.cc — put the arm in passive mode, record joint angles
// (base, left, right) at ~20 Hz to a file, then re-engage motors.
//
// Usage: teach_record_joint [port] [output_file]
//   port         defaults to /dev/ttyACM0
//   output_file  defaults to teach_joint.urec
//
// Press Ctrl-C to stop. File is written after the loop exits.
//
// Joint angles are queried via P2200 (B/L/R in firmware degrees).
// These are directly compatible with G2206 for playback.

#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>

#include "swiftpro/swiftpro.h"

// ---------------------------------------------------------------------------

struct JointSample {
    int64_t t_ms;
    float   b, l, r;   // base, left, right — firmware degrees
};

static std::vector<JointSample> g_samples;
static std::atomic<bool>        g_running{true};

static void sig_handler(int) { g_running.store(false); }

// ---------------------------------------------------------------------------

static void write_file(const std::string& path,
                       const std::string& port,
                       const std::vector<JointSample>& samples)
{
    std::ofstream f(path);
    if (!f) {
        std::cerr << "Failed to open output file: " << path << "\n";
        return;
    }

    const auto now    = std::chrono::system_clock::now();
    const auto now_t  = std::chrono::system_clock::to_time_t(now);
    const auto now_tm = *std::localtime(&now_t);

    f << "# libuarm teach recording (joint space)\n";
    f << "# recorded: " << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
    f << "# port: " << port << "\n";
    f << "# samples: " << samples.size() << "\n";
    f << "# fields: t_ms, base_deg, left_deg, right_deg\n";
    f << "\n";

    for (size_t i = 0; i < samples.size(); ++i) {
        const auto&   s  = samples[i];
        const int64_t dt = (i == 0) ? 0 : (s.t_ms - samples[i-1].t_ms);

        if (i == 0) {
            f << "# sample " << i << "\n";
        } else {
            f << "# sample " << i << "  (dt: " << dt << "ms)\n";
        }

        f << std::fixed << std::setprecision(1);
        f << "t: " << s.t_ms << "\n";
        f << "b: " << s.b    << "\n";
        f << "l: " << s.l    << "\n";
        f << "r: " << s.r    << "\n";
        f << "\n";
    }

    std::cout << "Wrote " << samples.size()
              << " samples to " << path << "\n";
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    const std::string port    = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string outfile = (argc > 2) ? argv[2] : "teach_joint.urec";

    struct sigaction sa{};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
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

    std::cout << "Detaching servos (passive/teach mode)...\n";
    arm.set_servo_detach_sync(-1);
    std::cout << "Arm is now backdriveable. Move it by hand.\n";
    std::cout << "Recording to: " << outfile << "\n";
    std::cout << "Press Ctrl-C to stop.\n\n";

    // Record loop — poll P2200 at ~20Hz (50ms interval).
    constexpr int64_t INTERVAL_MS = 50;

    const auto record_start = std::chrono::steady_clock::now();

    while (g_running.load()) {
        const auto loop_start = std::chrono::steady_clock::now();

        auto result = arm.get_joint_angles_sync(1.0f);
        if (result && result->size() >= 3) {
            const int64_t t_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    loop_start - record_start).count();

            g_samples.push_back({
                t_ms,
                result->at(0),   // base
                result->at(1),   // left
                result->at(2)    // right
            });

            std::cout << "\r  samples: " << g_samples.size()
                      << "  b=" << std::fixed << std::setprecision(1)
                      << result->at(0)
                      << " l=" << result->at(1)
                      << " r=" << result->at(2)
                      << "        " << std::flush;
        }

        // Sleep for remainder of interval.
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        const int64_t remaining = INTERVAL_MS - elapsed;
        if (remaining > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(remaining));
        }
    }

    std::cout << "\n";

    std::cout << "Re-attaching servos...\n";
    arm.set_servo_attach_sync(-1);
    arm.disconnect();

    write_file(outfile, port, g_samples);

    return 0;
}

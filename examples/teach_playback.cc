// teach_playback.cc — replay a teach recording captured by teach_record.
//
// Usage: teach_playback [port] [input_file] [speed_factor]
//   port          defaults to /dev/ttyACM0
//   input_file    defaults to teach_recording.urec
//   speed_factor  playback speed multiplier (default 1.0, 2.0 = double speed)
//
// Positions are sent async to the firmware planner. The inter-sample delay
// from the recording is honoured as closely as possible. The GRBL planner
// buffers and smooths the motion.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "swiftpro/swiftpro.h"

// ---------------------------------------------------------------------------

struct Sample {
    int64_t t_ms;
    float   x, y, z, w;
};

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
        // Skip comments and blank lines between samples.
        if (line.empty() || line[0] == '#') {
            if (fields_read == 5) {
                samples.push_back(cur);
                cur = {};
                fields_read = 0;
            }
            continue;
        }

        // Parse "key: value"
        const auto colon = line.find(':');
        if (colon == std::string::npos) { continue; }

        const std::string key = line.substr(0, colon);
        const std::string val = line.substr(colon + 2);  // skip ": "

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

    // Flush last sample if file doesn't end with blank line.
    if (fields_read == 5) {
        samples.push_back(cur);
    }

    return samples;
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    const std::string port         = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string infile       = (argc > 2) ? argv[2] : "teach_recording.urec";
    const float       speed_factor = (argc > 3) ? std::stof(argv[3]) : 1.0f;

    if (speed_factor <= 0.0f) {
        std::cerr << "speed_factor must be > 0\n";
        return 1;
    }

    std::cout << "Loading recording from: " << infile << "\n";
    const auto samples = load_file(infile);
    if (samples.empty()) {
        std::cerr << "No samples loaded.\n";
        return 1;
    }
    std::cout << "Loaded " << samples.size() << " samples.\n";

    const int64_t total_ms = samples.back().t_ms;
    std::cout << "Recording duration: " << total_ms << "ms"
              << " (" << std::fixed << std::setprecision(1)
              << total_ms / 1000.0f << "s)\n";
    std::cout << "Playback speed: " << speed_factor << "x\n\n";

    swiftpro::Swift arm;

    std::cout << "Connecting to " << port << " ...\n";
    if (!arm.connect(port)) {
        std::cerr << "connect() failed\n";
        return 1;
    }
    std::cout << "Connected. Waiting for firmware boot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    arm.set_motion_report_sync(true);

    // Move to the first recorded position before starting playback.
    std::cout << "Moving to start position...\n";
    const auto& first = samples.front();
    arm.set_position_sync(first.x, first.y, first.z, 50.0f);
    arm.set_wrist_sync(first.w, 1000.0f);
    std::cout << "At start position. Beginning playback...\n\n";

    // Playback loop.
    // For each sample, send the position async then sleep for the recorded
    // inter-sample interval (adjusted by speed_factor).
    const auto playback_start = std::chrono::steady_clock::now();

    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];

        // Send position async — firmware planner queues and smooths.
        // Speed parameter: use inter-sample distance / inter-sample time
        // as a rough mm/min estimate, clamped to sane range.
        float speed = 50.0f;
        if (i + 1 < samples.size()) {
            const auto& next = samples[i + 1];
            const int64_t dt_ms = next.t_ms - s.t_ms;
            if (dt_ms > 0) {
                const float dx   = next.x - s.x;
                const float dy   = next.y - s.y;
                const float dz   = next.z - s.z;
                const float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                // dist in mm, dt in ms → mm/s → mm/min
                const float mms  = dist / (dt_ms / 1000.0f);
                speed = std::clamp(mms * 60.0f * speed_factor, 10.0f, 50000.0f);
            }
        }

        arm.set_position(s.x, s.y, s.z, speed);

        // Status output.
        std::cout << "\r  sample " << i + 1 << "/" << samples.size()
                  << "  (" << std::fixed << std::setprecision(1)
                  << s.x << ", " << s.y << ", " << s.z << ") mm"
                  << "  speed=" << std::setprecision(0) << speed << "mm/min   "
                  << std::flush;

        // Sleep for inter-sample interval adjusted by speed_factor.
        if (i + 1 < samples.size()) {
            const int64_t dt_ms = samples[i + 1].t_ms - s.t_ms;
            const int64_t sleep_ms = static_cast<int64_t>(
                dt_ms / speed_factor);
            if (sleep_ms > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(sleep_ms));
            }
        }
    }

    std::cout << "\n\nPlayback complete. Waiting for motion to finish...\n";

    // Wait for final motion to complete.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    arm.disconnect();
    std::cout << "Done.\n";
    return 0;
}

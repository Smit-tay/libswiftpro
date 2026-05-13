// basic_move.cc — connect, home, visit two waypoints, disconnect.
//
// Usage: basic_move [port]
//   port defaults to /dev/ttyACM0

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "swiftpro/swiftpro.h"

static void check(const char* op, const swiftpro::VoidResult& r)
{
    if (!r) {
        std::cerr << op << " failed: " << r.error() << "\n";
        std::exit(1);
    }
}

int main(int argc, char* argv[])
{
    const std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM0";

    swiftpro::Swift arm;

    std::cout << "Connecting to " << port << " ...\n";
    if (!arm.connect(port)) {
        std::cerr << "connect() failed\n";
        return 1;
    }
    std::cout << "Connected. Waiting for firmware boot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Enable motion-complete events so _sync() moves know when to return.
    arm.set_motion_report_sync(true);

    // Home the arm.
    std::cout << "Homing...\n";
    check("reset", arm.reset_sync());

    // Confirm current position.
    auto pos = arm.get_position_sync();
    if (pos) {
        std::cout << "Position after home:"
                  << " x=" << pos->at(0)
                  << " y=" << pos->at(1)
                  << " z=" << pos->at(2) << " mm\n";
    }

    // Move to waypoint A.
    std::cout << "Moving to waypoint A (200, 50, 180)...\n";
    check("set_position A",
          arm.set_position_sync(200.0f, 50.0f, 180.0f, 50.0f));

    pos = arm.get_position_sync();
    if (pos) {
        std::cout << "At A: x=" << pos->at(0)
                  << " y=" << pos->at(1)
                  << " z=" << pos->at(2) << "\n";
    }

    // Move to waypoint B.
    std::cout << "Moving to waypoint B (200, 50, 150)...\n";
    check("set_position B",
          arm.set_position_sync(200.0f, 50.0f, 150.0f, 50.0f));

    pos = arm.get_position_sync();
    if (pos) {
        std::cout << "At B: x=" << pos->at(0)
                  << " y=" << pos->at(1)
                  << " z=" << pos->at(2) << "\n";
    }

    // Return home.
    std::cout << "Returning home...\n";
    check("reset", arm.reset_sync());

    arm.disconnect();
    std::cout << "Done.\n";
    return 0;
}

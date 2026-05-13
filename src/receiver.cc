// receiver.cc
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.

#include "swiftpro/receiver.h"
#include "swiftpro/commander.h"

#include <charconv>
#include <cstring>
#include <iostream>
#include <sstream>

namespace swiftpro {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Split a string by whitespace into tokens.
static std::vector<std::string> split(const std::string& s)
{
    std::vector<std::string> tokens;
    std::istringstream       iss(s);
    std::string              tok;
    while (iss >> tok) { tokens.push_back(tok); }
    return tokens;
}

// Parse a token of the form "X3.14" — strip first char, parse float.
// Returns false on failure.
static bool parse_prefixed_float(const std::string& tok, float& out)
{
    if (tok.size() < 2) { return false; }
    try {
        out = std::stof(tok.substr(1));
        return true;
    } catch (...) {
        return false;
    }
}

// Parse a token of the form "V42" — strip first char, parse int.
static bool parse_prefixed_int(const std::string& tok, int& out)
{
    if (tok.size() < 2) { return false; }
    try {
        out = std::stoi(tok.substr(1));
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------------------

Receiver::Receiver(serial::Serial& ser, Commander& commander)
: ser_(ser)
, commander_(commander)
{}

Receiver::~Receiver()
{
    stop();
}

void Receiver::start()
{
    if (running_.exchange(true)) { return; }
    recv_thread_ = std::thread(&Receiver::recv_loop, this);
}

void Receiver::stop()
{
    if (!running_.exchange(false)) { return; }
    if (recv_thread_.joinable()) { recv_thread_.join(); }
}

void Receiver::on_position(PositionCallback cb)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    cb_position_ = std::move(cb);
}

void Receiver::on_joint(JointCallback cb)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    cb_joint_ = std::move(cb);
}

void Receiver::on_motion_complete(MotionCompleteCallback cb)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    cb_motion_complete_ = std::move(cb);
}

void Receiver::on_limit_switch(LimitSwitchCallback cb)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    cb_limit_switch_ = std::move(cb);
}

void Receiver::on_power(PowerCallback cb)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    cb_power_ = std::move(cb);
}

// ---------------------------------------------------------------------------
// Recv loop
// ---------------------------------------------------------------------------

void Receiver::recv_loop()
{
    std::string buf;

    while (running_) {
        try {
            if (!ser_.isOpen()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            // Read available bytes into buffer.
            const size_t avail = ser_.available();
            if (avail == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            std::string chunk;
            ser_.read(chunk, avail);
            buf += chunk;

            // Process complete lines.
            size_t pos;
            while ((pos = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, pos);
                buf.erase(0, pos + 1);

                // Strip carriage return if present.
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                if (!line.empty()) {
                    dispatch_line(line);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[uarm::Receiver] error: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

void Receiver::dispatch_line(const std::string& line)
{
    if (line.empty()) { return; }

    if (line[0] == '@') {
        dispatch_event(line);
        return;
    }

    dispatch_response(line);
}

void Receiver::dispatch_response(const std::string& line)
{
    // Sequenced response: "$N ok ..." or "$N E2x ..."
    if (line[0] == '$') {
        // Parse sequence number.
        size_t space = line.find(' ');
        if (space == std::string::npos) { return; }

        uint32_t seq = 0;
        try {
            seq = static_cast<uint32_t>(std::stoul(line.substr(1, space - 1)));
        } catch (...) {
            return;
        }

        const std::string payload = line.substr(space + 1);
        const bool ok = (payload.rfind("ok", 0) == 0);

        if (!commander_.deliver_response(seq, ok, payload)) {
            std::cerr << "[uarm::Receiver] unmatched seq " << seq
                      << " response: " << payload << "\n";
        }
        return;
    }

    // Unsequenced response.
    commander_.on_unsequenced_response(line);
}

void Receiver::dispatch_event(const std::string& line)
{
    // line starts with '@'
    if (line.size() < 2) { return; }

    const int event_id = line[1] - '0';

    // Payload starts after "@N " (3 chars) or "@N\0" if no payload.
    const std::string payload = (line.size() > 3) ? line.substr(3) : "";

    switch (event_id) {
        case 1:
            // @1 — boot complete. No callback — just log.
            std::cerr << "[uarm] firmware boot complete\n";
            break;
        case 3:
            parse_at3(payload);
            break;
        case 5:
            parse_at5(payload);
            break;
        case 6:
            parse_at6(payload);
            break;
        case 9:
            parse_at9(payload);
            break;
        default:
            std::cerr << "[uarm::Receiver] unknown event: " << line << "\n";
            break;
    }
}

// ---------------------------------------------------------------------------
// Event parsers
// ---------------------------------------------------------------------------

// @3 X<x> Y<y> Z<z> R<wrist> [B<base> L<left> AR<right>]
//
// Current firmware: 4 tokens (X Y Z R)
// Extended firmware (planned): 7 tokens (X Y Z R B L AR)
//
// The parser handles both forms gracefully.
void Receiver::parse_at3(const std::string& payload)
{
    const auto tokens = split(payload);
    if (tokens.size() < 4) { return; }

    float x = 0.0f, y = 0.0f, z = 0.0f, wrist = 0.0f;
    if (!parse_prefixed_float(tokens[0], x))     { return; }
    if (!parse_prefixed_float(tokens[1], y))     { return; }
    if (!parse_prefixed_float(tokens[2], z))     { return; }
    if (!parse_prefixed_float(tokens[3], wrist)) { return; }

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        if (cb_position_) { cb_position_(x, y, z, wrist); }
    }

    // Extended form — joint angles present.
    if (tokens.size() >= 7) {
        float b = 0.0f, l = 0.0f, r = 0.0f;
        if (!parse_prefixed_float(tokens[4], b)) { return; }
        if (!parse_prefixed_float(tokens[5], l)) { return; }
        if (!parse_prefixed_float(tokens[6], r)) { return; }

        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        if (cb_joint_) { cb_joint_(b, l, r); }
    }
}

// @5 V<0|1>
void Receiver::parse_at5(const std::string& payload)
{
    const auto tokens = split(payload);
    if (tokens.empty()) { return; }

    int val = 0;
    if (!parse_prefixed_int(tokens[0], val)) { return; }

    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    if (cb_power_) { cb_power_(val != 0); }
}

// @6 N0 V<bitmask>
void Receiver::parse_at6(const std::string& payload)
{
    const auto tokens = split(payload);
    if (tokens.size() < 2) { return; }

    int val = 0;
    if (!parse_prefixed_int(tokens[1], val)) { return; }

    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    if (cb_limit_switch_) { cb_limit_switch_(val != 0); }
}

// @9 V0
void Receiver::parse_at9(const std::string& /*payload*/)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    if (cb_motion_complete_) { cb_motion_complete_(); }
}

} // namespace swiftpro

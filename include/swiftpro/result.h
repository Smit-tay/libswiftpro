// result.h
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Result<T> — a simple value-or-error type for libuarm API returns.
//
// Usage:
//   Result<float> r = get_something();
//   if (r) { use(r.value); } else { log(r.error); }
//
//   Result<std::vector<float>> v = get_position_sync();
//   if (v) { float x = v->at(0); }

#pragma once

#include <string>
#include <variant>
#include <vector>
#include <array>

namespace swiftpro {

template<typename T>
struct Result
{
    // Construct a success result
    static Result ok(T val)
    {
        Result r;
        r.ok_    = true;
        r.value_ = std::move(val);
        return r;
    }

    // Construct an error result
    static Result err(std::string msg)
    {
        Result r;
        r.ok_    = false;
        r.error_ = std::move(msg);
        return r;
    }

    // bool conversion — true if success
    explicit operator bool() const { return ok_; }

    // Value accessors — undefined behaviour if !ok
    T&       value()       { return value_; }
    const T& value() const { return value_; }

    T*       operator->()       { return &value_; }
    const T* operator->() const { return &value_; }

    T&       operator*()       { return value_; }
    const T& operator*() const { return value_; }

    // Error accessor
    const std::string& error() const { return error_; }

private:
    bool        ok_{false};
    T           value_{};
    std::string error_;
};

// Convenience specialisation for void (command send/ack with no return value)
template<>
struct Result<void>
{
    static Result ok()
    {
        Result r;
        r.ok_ = true;
        return r;
    }

    static Result err(std::string msg)
    {
        Result r;
        r.ok_    = false;
        r.error_ = std::move(msg);
        return r;
    }

    explicit operator bool() const { return ok_; }
    const std::string& error() const { return error_; }

private:
    bool        ok_{false};
    std::string error_;
};

// Common result type aliases
using VoidResult   = Result<void>;
using IntResult    = Result<int>;
using FloatResult  = Result<float>;
using VecResult    = Result<std::vector<float>>;
using StringResult = Result<std::string>;
using InfoResult   = Result<std::array<std::string, 5>>;

} // namespace swiftpro

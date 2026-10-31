# libswiftpro
libswiftpro — C++17 async control library for the UFactory UArm Swift Pro (SwiftPro++ firmware, `V5.0.0-pp.1`)

Async-first serial interface to the UArm Swift Pro robotic arm.

Clean Commander/Receiver architecture with callback-driven flow control.

Result<T> error handling.

Protocol reference: [docs/protocol.md](docs/protocol.md).

Tests run against a fake serial port, no arm needed: `cmake -B build && cmake --build build && ctest --test-dir build`.

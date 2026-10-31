add_executable(test_protocol test_protocol.cc)
target_compile_options(test_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(test_protocol PRIVATE swiftpro_static)

foreach(_case
    wire_format
    line_length
    error_codes
    query_parsing
    reply_matching
    report_events
    stop_during_move
    reboot_fails_pending
    late_reply_after_timeout
    bare_error_keeps_running
    stale_pending_is_freed
    link_loss_and_reconnect
    connect_twice
    not_connected
    disconnect_under_traffic)
    add_test(NAME ${_case} COMMAND test_protocol ${_case})
    set_tests_properties(${_case} PROPERTIES TIMEOUT 60)
endforeach()

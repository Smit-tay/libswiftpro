add_executable(basic_move             basic_move.cc)
add_executable(teach_record           teach_record.cc)
add_executable(teach_record_joint     teach_record_joint.cc)
add_executable(teach_playback         teach_playback.cc)
add_executable(teach_playback_smooth  teach_playback_smooth.cc)
add_executable(teach_playback_joint   teach_playback_joint.cc)

foreach(_target
    basic_move
    teach_record
    teach_record_joint
    teach_playback
    teach_playback_smooth
    teach_playback_joint)
    target_link_libraries(${_target} PRIVATE uarm)
endforeach()

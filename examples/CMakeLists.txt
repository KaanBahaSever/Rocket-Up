set(ROCKETUP_EXAMPLES
    orbit_chaser
    teknofest_3dof
    other_worlds
    two_stage
    hil_flight_computer
    monte_carlo
    load_design
    motor_interpolation)

foreach(example ${ROCKETUP_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_include_directories(${example} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${example} PRIVATE rocketup::rocketup)
    set_target_properties(${example} PROPERTIES FOLDER examples)
endforeach()

add_executable(rocketup_tests
    main.cpp
    test_environment.cpp
    test_motor.cpp
    test_rocket.cpp
    test_simulation.cpp
    test_hil.cpp)
target_link_libraries(rocketup_tests PRIVATE rocketup::rocketup)
set_target_properties(rocketup_tests PROPERTIES FOLDER tests)
add_test(NAME rocketup_tests COMMAND rocketup_tests)

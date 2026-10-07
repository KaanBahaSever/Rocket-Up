add_executable(rocketup_cli rocketup_cli.cpp)
set_target_properties(rocketup_cli PROPERTIES OUTPUT_NAME rocketup)
target_link_libraries(rocketup_cli PRIVATE rocketup::rocketup)
install(TARGETS rocketup_cli RUNTIME DESTINATION bin)

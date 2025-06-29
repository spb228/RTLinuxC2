project(userMonitorApp)

add_executable(${PROJECT_NAME} user_monitor_app.c)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../bin)
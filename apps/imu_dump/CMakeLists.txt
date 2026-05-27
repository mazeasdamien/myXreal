add_executable(imu_dump
    main.cpp
)

add_custom_command(TARGET imu_dump POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/config/calibration.json"
    $<TARGET_FILE_DIR:imu_dump>
)

add_custom_command(TARGET imu_dump POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:imu_driver>
    $<TARGET_FILE_DIR:imu_dump>
)

add_custom_command(TARGET imu_dump POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:hidapi_winapi>
    $<TARGET_FILE_DIR:imu_dump>
)

target_link_libraries(imu_dump PRIVATE imu_driver)

add_executable(imu_debug
    main.cpp
)

add_custom_command(TARGET imu_debug POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/config/calibration.json"
    $<TARGET_FILE_DIR:imu_debug>
)

add_custom_command(TARGET imu_debug POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:imu_driver>
    $<TARGET_FILE_DIR:imu_debug>
)
add_custom_command(TARGET imu_debug POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:hidapi_winapi>
    $<TARGET_FILE_DIR:imu_debug>
)

target_link_libraries(imu_debug PRIVATE
    imu_driver
    calibration
    imgui
    d3d11
    dxgi
    d3dcompiler
    ${OpenCV_LIBS}
)

target_include_directories(imu_debug PRIVATE
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

add_library(imu_driver SHARED
    imu_driver.cpp
    stereo_camera.cpp
)

target_include_directories(imu_driver
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(imu_driver PUBLIC
    calibration
)

if(WIN32)
    target_link_libraries(imu_driver PRIVATE
        hidapi_winapi
        nlohmann_json::nlohmann_json
        strmiids
        ole32
        oleaut32
    )
    add_custom_command(TARGET imu_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:hidapi_winapi>
        $<TARGET_FILE_DIR:imu_driver>
    )
else()
    target_link_libraries(imu_driver PRIVATE hidapi_libusb nlohmann_json::nlohmann_json)
endif()

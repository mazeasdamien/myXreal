add_library(vr_scene_driver STATIC
    ${CMAKE_SOURCE_DIR}/src/vr_scene_driver/xreal_air_driver.cpp
    ${CMAKE_SOURCE_DIR}/src/vr_scene_driver/calibration.cpp
    ${CMAKE_SOURCE_DIR}/src/vr_scene_driver/sensor_fusion.cpp
    ${CMAKE_SOURCE_DIR}/src/vr_scene_driver/imu_navigator.cpp
)

target_compile_definitions(vr_scene_driver PRIVATE xreal_air_driver_EXPORTS=1)

target_include_directories(vr_scene_driver
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(vr_scene_driver
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

if(WIN32)
    target_link_libraries(vr_scene_driver PRIVATE hidapi_winapi)
else()
    target_link_libraries(vr_scene_driver PRIVATE hidapi_libusb)
endif()

add_executable(vr_stereo_scene
    main.cpp
)

target_compile_definitions(vr_stereo_scene PRIVATE
    VR_TEST_SCENE_STANDALONE=1
    xreal_air_driver_EXPORTS=1
)

target_include_directories(vr_stereo_scene PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(vr_stereo_scene PRIVATE
    vr_scene_driver
    d3d11
    dxgi
    d3dcompiler
    hidapi_winapi
)

add_custom_command(TARGET vr_stereo_scene POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/assets/funny_beaver.bin"
    $<TARGET_FILE_DIR:vr_stereo_scene>
)

add_custom_command(TARGET vr_stereo_scene POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/config/calibration.json"
    $<TARGET_FILE_DIR:vr_stereo_scene>
)

add_custom_command(TARGET vr_stereo_scene POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:hidapi_winapi>
    $<TARGET_FILE_DIR:vr_stereo_scene>
)

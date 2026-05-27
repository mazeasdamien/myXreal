#include "myxreal/imu_driver.h"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    const char* calib_path = argc > 1 ? argv[1] : "calibration.json";

    if (imu_init(calib_path) != 0) {
        fprintf(stderr, "IMU init failed. Is the glasses connected?\n");
        return 1;
    }

    if (imu_start_streaming() != 0) {
        fprintf(stderr, "Failed to start streaming.\n");
        imu_shutdown();
        return 1;
    }

    printf("Timestamp_ns           Temp(C)   Accel_X    Accel_Y    Accel_Z    Gyro_X     Gyro_Y     Gyro_Z     |Accel|   FPS\n");
    printf("----------------------  --------  ---------  ---------  ---------  ---------  ---------  ---------  --------  -----\n");

    ImuSample s;
    while (true) {
        while (imu_poll_sample(&s)) {
            float accel_mag = std::sqrt(s.accel[0]*s.accel[0] + s.accel[1]*s.accel[1] + s.accel[2]*s.accel[2]);
            printf("%22llu  %8.2f  %9.4f  %9.4f  %9.4f  %9.4f  %9.4f  %9.4f  %8.3f  %5.1f\n",
                   s.timestamp_ns,
                   s.temperature_c,
                   s.accel[0], s.accel[1], s.accel[2],
                   s.gyro[0],  s.gyro[1],  s.gyro[2],
                   accel_mag,
                   s.fps);
        }
        // Small sleep so we don't busy-spin
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    imu_stop_streaming();
    imu_shutdown();
    return 0;
}

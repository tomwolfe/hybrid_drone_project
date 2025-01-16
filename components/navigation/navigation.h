#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>

// --- Data Structures ---

// Structure to hold GPS data from the M10 module
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float heading;
    uint32_t timestamp; // Likely in milliseconds since epoch or system boot
} GPSData;

// Structure to hold IMU data from the Pixhawk 6C (ICM-45686)
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    uint32_t timestamp; // Likely in milliseconds since epoch or system boot
} IMUData;

// Structure to hold ultrasonic sensor readings
typedef struct {
    float front_distance;
    float back_distance;
    float left_distance;
    float right_distance;
    float top_distance;
    float bottom_down_distance;
    float bottom_forward_angle_distance;
    uint32_t timestamp; // Likely in milliseconds since last read
} UltrasonicReadings;

// Structure to hold visual odometry data (if implemented on ESP32-S3)
typedef struct {
    float delta_x;
    float delta_y;
    float delta_z;
    float delta_roll;
    float delta_pitch;
    float delta_yaw;
    uint32_t timestamp; // Likely in milliseconds since last update
} VisualOdomData;

// Structure to represent obstacle data (used for sending to ArduPilot)
typedef struct {
    float distance; // Distance to the obstacle
    float angle_x;  // Angle in the horizontal plane
    float angle_y;  // Angle in the vertical plane
    uint8_t sensor_type; // Identifier for the type of sensor (e.g., ultrasonic)
    uint32_t timestamp;
} ObstacleData;

// --- Function Prototypes ---

// Initialization function for the navigation module
bool navigation_init();

// Function to get the latest GPS data
bool get_gps_data(GPSData *gps);

// Function to get the latest IMU data from Pixhawk (requires MAVLink communication)
bool get_imu_data(IMUData *imu);

// Function to read and process data from the ultrasonic sensors
bool read_ultrasonic_sensors(UltrasonicReadings *readings);

// Function to process ultrasonic data (e.g., filtering, outlier removal)
void process_ultrasonic_data(UltrasonicReadings *raw_readings, UltrasonicReadings *processed_readings);

// Function to handle short-range obstacle avoidance based on ultrasonic data
void handle_short_range_avoidance(const UltrasonicReadings *readings);

// Function to implement visual odometry (if processed on ESP32-S3)
bool process_visual_odometry(VisualOdomData *odom);

// Function to send obstacle data to ArduPilot via MAVLink (using OBSTACLE_DISTANCE or ULTRASONIC_SENSOR message)
bool send_obstacle_data_to_autopilot(const ObstacleData *obstacle);

// Function to fuse sensor data (GPS, IMU, ultrasonic, visual odometry)
// You might have separate fusion functions or a single one with flags.
bool fuse_sensor_data(); // Or more specific functions

// --- Configuration Options (Optional) ---
// You might use extern variables for configuration or functions to set them.
// extern float obstacle_avoidance_threshold;

#endif // NAVIGATION_H

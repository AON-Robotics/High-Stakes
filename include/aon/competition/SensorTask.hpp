#include "EKF.hpp"
#include "pros/rtos.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/gps.hpp"
#include <iostream>
#include "../globals.hpp"
#include "../constants.hpp"

// Global EKF instance
EKF inline ekf;

// ===== EKF TUNING PARAMETERS =====
// Base noise parameters
#define POSITION_NOISE_BASE 0.03
#define HEADING_NOISE_BASE 0.01
#define VELOCITY_NOISE_BASE 0.08
#define OMEGA_NOISE_BASE 0.08

// Noise scaling factors
#define POSITION_NOISE_FACTOR 0.02
#define HEADING_NOISE_FACTOR 0.02
#define VELOCITY_NOISE_FACTOR 0.03
#define OMEGA_NOISE_FACTOR 0.04

// Measurement noise
#define GPS_NOISE 1.0
#define GYRO_NOISE 0.01
#define ENCODER_NOISE 0.1

// ===== DEBUG SETTINGS =====
#define DEBUG_PRINT_INTERVAL 10   // Print debug every X iterations
#define GYRO_UPDATE_INTERVAL 5    // Update with gyro every X iterations
#define GPS_UPDATE_INTERVAL 10    // Update with GPS every X iterations

// ===== GLOBAL VARIABLES =====
// Tracking variables
double inline last_time = 0;
double inline last_linear_pos = 0;
double inline last_heading_pos = 0;
bool inline initialized = false;
uint32_t inline update_counter = 0;

/**
 * @brief Updates noise parameters dynamically based on robot state
 */
void inline updateAdaptiveNoise(double linear_velocity, double angular_velocity) {
    // Calculate the magnitude of motion
    double speed = fabs(linear_velocity);
    double turn_rate = fabs(angular_velocity);
    
    // Adaptive position noise - increases with speed
    double pos_noise = POSITION_NOISE_BASE + POSITION_NOISE_FACTOR * speed;
    
    // Adaptive heading noise - increases with turn rate
    double heading_noise = HEADING_NOISE_BASE + HEADING_NOISE_FACTOR * turn_rate;
    
    // Adaptive velocity noise
    double vel_noise = VELOCITY_NOISE_BASE + VELOCITY_NOISE_FACTOR * speed;
    double omega_noise = OMEGA_NOISE_BASE + OMEGA_NOISE_FACTOR * turn_rate;
    
    // Set the adaptive noise parameters
    ekf.setPositionNoise(pos_noise);
    ekf.setHeadingNoise(heading_noise);
    ekf.setVelocityNoise(vel_noise, omega_noise);
    
    // Output noise adjustments periodically
    if (update_counter % DEBUG_PRINT_INTERVAL == 0) {
        std::cout << "ADAPTIVE NOISE: pos=" << pos_noise 
                  << ", heading=" << heading_noise 
                  << ", vel=" << vel_noise 
                  << ", omega=" << omega_noise << std::endl;
    }
}

/**
 * @brief Initialize sensors and EKF
 */
void inline initialize_ekf() {
    if (!initialized) {
        // Clear terminal and print header
        std::cout << "\n\n\n==== EKF INITIALIZATION ====" << std::endl;
        
        // Initialize IMU if used alongside rotation sensors
        if (GYRO_ENABLED) {
            std::cout << "Calibrating IMU..." << std::endl;
            gyroscope.reset();
            pros::delay(2000); // Wait for IMU calibration
            std::cout << "IMU calibration complete" << std::endl;
        }
        
        // Initialize GPS
        std::cout << "Initializing GPS..." << std::endl;
        // For VEX GPS, set the origin and rotation appropriately
        // Default orientation: origin at center of field, positive x towards red alliance
        gps.initialize_full(0, 0, 0, 0, 0); // x_offset, y_offset, x_position, y_position, heading
        std::cout << "GPS initialization complete" << std::endl;
        
        // Initialize rotation sensors
        std::cout << "Initializing rotation sensors..." << std::endl;
        
        // Reset rotation sensors
        encoderLeft.reset_position();
        encoderRight.reset_position();
        std::cout << "Rotation sensors reset" << std::endl;
        
        // Configure linear rotation sensor for velocity measurements
        // Reverse if needed based on mounting orientation
        encoderLeft.set_reversed(false);
        
        // Configure heading rotation sensor
        // Reverse if needed based on mounting orientation
        encoderRight.set_reversed(false);
        
        // Wait for GPS to get initial reading
        pros::delay(500);
        
        // Get initial position from GPS
        auto gps_data = gps.get_status();
        
        // Convert from meters to inches
        double x_inches = gps_data.x * 39.3701;
        double y_inches = gps_data.y * 39.3701;
        
        // Get initial heading from heading rotation sensor
        double heading_deg = encoderRight.get_position() / 100.0; // VEX rotation sensor gives position in centidegrees
        double heading_rad = heading_deg * (M_PI / 180.0);
        
        // Set initial EKF parameters
        ekf.setPositionNoise(POSITION_NOISE_BASE);
        ekf.setHeadingNoise(HEADING_NOISE_BASE);
        ekf.setVelocityNoise(VELOCITY_NOISE_BASE, OMEGA_NOISE_BASE);
        ekf.setGpsNoise(GPS_NOISE);
        ekf.setGyroNoise(GYRO_NOISE);
        ekf.setEncoderNoise(ENCODER_NOISE);
        
        // Initialize EKF with initial position
        ekf.init(x_inches, y_inches, heading_rad);
        
        // Initialize tracking variables
        last_time = pros::millis() / 1000.0;
        last_linear_pos = encoderLeft.get_position();
        last_heading_pos = encoderRight.get_position();
        
        std::cout << "EKF initialized at position: (" << x_inches << ", " << y_inches 
                  << ") inches, heading: " << heading_deg << " deg" << std::endl;
        
        // Print configuration
        std::cout << "==== CONFIGURATION ====" << std::endl;
        std::cout << "ENCODER_WHEEL_DIAMETER: " << DRIVE_WHEEL_DIAMETER << " inches" << std::endl;
        std::cout << "GYRO_ENABLED: " << (GYRO_ENABLED ? "YES" : "NO") << std::endl;
        std::cout << "GYRO_CONFIDENCE: " << GYRO_CONFIDENCE << std::endl;
        
        std::cout << "==== BEGINNING EKF OPERATION ====" << std::endl;
        initialized = true;
    }
}

/**
 * @brief Calculate robot odometry from both rotation sensors used for linear tracking
 * @param linear_pos Current position of left rotation sensor (centidegrees)
 * @param heading_pos Current position of right rotation sensor (centidegrees)
 * @param dt Time step
 * @param linear_vel Output linear velocity (inches/sec)
 * @param angular_vel Output angular velocity (rad/sec)
 */
inline void calculate_odometry_from_rotation(double linear_pos, double heading_pos, double dt,
                                    double& linear_vel, double& angular_vel) {
    // Calculate rotation deltas
    double delta_linear = linear_pos - last_linear_pos;
    double delta_heading = heading_pos - last_heading_pos;
    
    // Convert centidegrees to degrees
    delta_linear /= 100.0;
    delta_heading /= 100.0;
    
    // Convert rotation degrees to distance
    double wheel_circumference = DRIVE_WHEEL_DIAMETER * M_PI;
    double left_distance = (delta_linear / 360.0) * wheel_circumference;
    double right_distance = (delta_heading / 360.0) * wheel_circumference;
    
    // Calculate average linear distance (from both wheels)
    double avg_distance = (left_distance + right_distance) / 2.0;
    
    // Calculate linear velocity using the average
    linear_vel = avg_distance / dt;
    
    // Calculate angular velocity using the difference between wheels
    // Simplified formula: angular velocity = (right_distance - left_distance) / (wheel_base * dt)
    double wheel_base = DRIVE_WIDTH; // Distance between the wheels
    angular_vel = (right_distance - left_distance) / (wheel_base * dt);
    
    // Debug print sensor data periodically
    if (update_counter % DEBUG_PRINT_INTERVAL == 0) {
        std::cout << "ROTATION SENSORS: left=" << linear_pos 
                  << ", right=" << heading_pos 
                  << ", delta_left=" << delta_linear 
                  << ", delta_right=" << delta_heading << std::endl;
        std::cout << "ODOMETRY: left_dist=" << left_distance
                  << ", right_dist=" << right_distance
                  << ", avg_dist=" << avg_distance
                  << ", lin_vel=" << linear_vel 
                  << ", ang_vel=" << angular_vel << std::endl;
    }
}

/**
 * @brief Update the EKF with all sensor readings
 */
inline void update_ekf() {
    // Initialize if not already done
    if (!initialized) {
        initialize_ekf();
        return;
    }
    
    // Increment counter
    update_counter++;
    
    // Get current time and calculate dt
    double current_time = pros::millis() / 1000.0;
    double dt = current_time - last_time;
    
    // Only update if enough time has passed
    if (dt < 0.005) {
        pros::delay(5); // Small delay to avoid CPU hogging
        return;
    }
    
    // === SENSOR READING ===
    // Get rotation sensor positions (in centidegrees)
    double linear_pos = encoderLeft.get_position();
    double heading_pos = encoderRight.get_position();
    
    // Calculate odometry from rotation sensors
    double linear_velocity = 0, angular_velocity_rotation = 0;
    calculate_odometry_from_rotation(linear_pos, heading_pos, dt, 
                                    linear_velocity, angular_velocity_rotation);
    
    // Get gyro angular velocity if enabled
    double gyro_omega = 0;
    if (GYRO_ENABLED) {
        gyro_omega = gyroscope.get_gyro_rate().z * (M_PI / 180.0); // rad/sec
    }
    
    // Get heading from rotation sensor
    double rotation_heading = encoderRight.get_position() / 100.0 * (M_PI / 180.0); // Convert centidegrees to radians
    
    // Get GPS position using get_status instead of get_position
    auto gps_data = gps.get_status();
    
    // Convert from meters to inches
    double gps_x_inches = gps_data.x * 39.3701;
    double gps_y_inches = gps_data.y * 39.3701;
    
    // Print sensor data periodically
    if (update_counter % DEBUG_PRINT_INTERVAL == 0) {
        std::cout << "TIME: t=" << current_time << ", dt=" << dt << std::endl;
        std::cout << "GPS: x=" << gps_x_inches << ", y=" << gps_y_inches << " inches" << std::endl;
        std::cout << "ROTATION HEADING: " << (rotation_heading * 180.0 / M_PI) 
                  << " deg, omega=" << (angular_velocity_rotation * 180.0 / M_PI) << " deg/s" << std::endl;
        
        if (GYRO_ENABLED) {
            std::cout << "IMU OMEGA: " << (gyro_omega * 180.0 / M_PI) << " deg/s" << std::endl;
        }
    }
    
    // === UPDATE NOISE PARAMETERS ===
    // Use either rotation sensor or gyro for angular velocity based on GYRO_ENABLED
    double angular_velocity = GYRO_ENABLED ? gyro_omega : angular_velocity_rotation;
    
    // Update adaptive noise based on current velocities
    updateAdaptiveNoise(linear_velocity, angular_velocity);
    
    // === EKF PREDICTION ===
    // Use angular velocity from appropriate source
    ekf.predict(linear_velocity, angular_velocity, dt);
    
    // === EKF UPDATES WITH MEASUREMENTS ===
    // Update with rotation heading
    if (update_counter % GYRO_UPDATE_INTERVAL == 0) {
        ekf.updateGyro(rotation_heading);
        
        if (update_counter % DEBUG_PRINT_INTERVAL == 0) {
            std::cout << "EKF: Updated with rotation heading" << std::endl;
        }
    }
    
    // Update with GPS periodically
    if (update_counter % GPS_UPDATE_INTERVAL == 0) {
        ekf.updateGPS(gps_x_inches, gps_y_inches);
        
        if (update_counter % DEBUG_PRINT_INTERVAL == 0) {
            std::cout << "EKF: Updated with GPS position" << std::endl;
        }
    }
    
    // Update with encoder velocity
    ekf.updateEncoder(linear_velocity);
    
    // === OUTPUT CURRENT STATE ===
    // Get current EKF state
    State state = ekf.getState();
    
    // Print EKF state periodically
    if (update_counter % DEBUG_PRINT_INTERVAL == 0) {
        std::cout << "==== EKF STATE ====" << std::endl;
        std::cout << "Position: (" << state.x << ", " << state.y << ") inches" << std::endl;
        std::cout << "Heading: " << (state.theta * 180.0 / M_PI) << " degrees" << std::endl;
        std::cout << "Velocity: " << state.v << " in/s" << std::endl;
        std::cout << "Angular Velocity: " << (state.omega * 180.0 / M_PI) << " deg/s" << std::endl;
        std::cout << "Position Variance: " << ekf.getPositionVariance() << std::endl;
        std::cout << "Heading Variance: " << ekf.getHeadingVariance() << std::endl;
        std::cout << "========================" << std::endl;
    }
    
    // Update tracking variables
    last_time = current_time;
    last_linear_pos = linear_pos;
    last_heading_pos = heading_pos;
}

/**
 * @brief Debug function to check sensor consistency
 */
inline void check_sensor_consistency() {
    // Get current EKF state
    State state = ekf.getState();
    
    // Get GPS reading
    auto gps_data = gps.get_status();
    double gps_x_inches = gps_data.x * 39.3701;
    double gps_y_inches = gps_data.y * 39.3701;
    
    // Get rotation sensor heading
    double rotation_heading = encoderRight.get_position() / 100.0 * (M_PI / 180.0);
    
    // Calculate differences
    double pos_diff = sqrt(pow(gps_x_inches - state.x, 2) + pow(gps_y_inches - state.y, 2));
    
    double heading_diff = rotation_heading - state.theta;
    // Normalize angle difference
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    std::cout << "==== SENSOR CONSISTENCY ====" << std::endl;
    std::cout << "GPS-EKF position difference: " << pos_diff << " inches" << std::endl;
    std::cout << "Rotation-EKF heading difference: " << (heading_diff * 180.0 / M_PI) << " degrees" << std::endl;
    
    // Check for significant discrepancies
    if (pos_diff > 10.0) {
        std::cout << "WARNING: Large position difference detected!" << std::endl;
    }
    
    if (fabs(heading_diff) > 0.2) {
        std::cout << "WARNING: Large heading difference detected!" << std::endl;
    }
    
    std::cout << "==========================" << std::endl;
}
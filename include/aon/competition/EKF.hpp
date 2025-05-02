#pragma once

#include "pros/rtos.hpp"
#include <cmath> 

/**
 * @file ekf.hpp
 * @brief Extended Kalman Filter for 2D pose (x, y, theta) on VEX V5
 *
 * This EKF fuses differential-drive odometry (v, omega) with GPS measurements.
 * Optimized for VEX V5 Brain (Cortex A9 processor, 128MB RAM).
 */

// State structure to hold the estimated pose
struct State {
    double x;      ///< X position (inches)
    double y;      ///< Y position (inches)
    double theta;  ///< Heading (radians)
    double v;      ///< Linear velocity (inches/sec)
    double omega;  ///< Angular velocity (rad/sec)
};

class EKF {
public:
    /**
     * @brief Create EKF with default noise matrices
     */
    EKF() {
        // Initialize process noise (motion uncertainty)
        // Sparse matrix storage to save memory and computation
        Q_diag[0] = 0.05; // x position uncertainty
        Q_diag[1] = 0.05; // y position uncertainty
        Q_diag[2] = 0.02; // theta uncertainty
        Q_diag[3] = 0.10; // v uncertainty
        Q_diag[4] = 0.10; // omega uncertainty
        
        // GPS measurement noise (position uncertainty in inches^2)
        R_gps[0] = 1.0;  // x measurement variance
        R_gps[1] = 0.0;  // xy covariance (normally 0)
        R_gps[2] = 0.0;  // yx covariance (normally 0)
        R_gps[3] = 1.0;  // y measurement variance
        
        // IMU/Gyro measurement noise (rad^2)
        R_gyro = 0.01;
        
        // Encoder measurement noise (inches/sec)^2
        R_encoder = 0.1;
    }

    /**
     * @brief Initialize state and covariance
     * @param x0 initial x position (inches)
     * @param y0 initial y position (inches)
     * @param th0 initial heading (radians)
     * @param v0 initial linear velocity (inches/sec)
     * @param omega0 initial angular velocity (rad/sec)
     */
    void init(double x0, double y0, double th0, double v0 = 0.0, double omega0 = 0.0) {
        mutex_.take(TIMEOUT_MAX);
        
        // Initialize state
        x_[0] = x0;     // x position
        x_[1] = y0;     // y position
        x_[2] = th0;    // heading
        x_[3] = v0;     // linear velocity
        x_[4] = omega0; // angular velocity
        
        // Initialize covariance matrix with small uncertainty
        // Use sparse structure for efficiency (diagonal + key correlations)
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                if (i == j) {
                    P_[i][j] = 0.1; // Diagonal elements
                } else {
                    P_[i][j] = 0.0; // Off-diagonal elements
                }
            }
        }
        
        // Velocity states have higher initial uncertainty
        P_[3][3] = 0.5;  // v variance
        P_[4][4] = 0.5;  // omega variance
        
        mutex_.give();
    }

    /**
     * @brief Prediction step using odometry
     * @param v_measured linear velocity from encoders (inches/sec)
     * @param omega_measured angular velocity from gyro (radians/sec)
     * @param dt time step (sec)
     */
    void predict(double v_measured, double omega_measured, double dt) {
        mutex_.take(TIMEOUT_MAX);
        
        // Get current state values
        double x = x_[0];
        double y = x_[1];
        double theta = x_[2];
        double v = x_[3];
        double omega = x_[4];
        
        // Velocity persistence factors (how much to trust current vs measured velocity)
        // Values closer to 1.0 mean we trust our model more than measurements
        const double alpha_v = 0.7;
        const double alpha_omega = 0.7;
        
        // Update velocities with measurements (weighted average)
        v = alpha_v * v + (1.0 - alpha_v) * v_measured;
        omega = alpha_omega * omega + (1.0 - alpha_omega) * omega_measured;
        
        // State prediction using kinematic model
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        theta = theta + omega * dt;
        
        // Normalize angle to [-pi, pi]
        normalizeAngle(theta);
        
        // Store updated state
        x_[0] = x;
        x_[1] = y;
        x_[2] = theta;
        x_[3] = v;
        x_[4] = omega;
        
        // Jacobian of state transition for EKF covariance update
        // Only compute the non-zero elements to save time
        double F[5][5] = {}; // Initialize to zeros
        
        // Diagonal elements (always present)
        F[0][0] = 1.0;
        F[1][1] = 1.0;
        F[2][2] = 1.0;
        F[3][3] = alpha_v;
        F[4][4] = alpha_omega;
        
        // Non-zero off-diagonal elements
        F[0][2] = -v * sin(theta) * dt;
        F[0][3] = cos(theta) * dt;
        F[1][2] = v * cos(theta) * dt;
        F[1][3] = sin(theta) * dt;
        F[2][4] = dt;
        
        // Compute P = F*P*F^T + Q efficiently
        // This optimized approach avoids full matrix multiplications
        // Instead, it computes each element directly
        
        double new_P[5][5] = {};
        
        // Optimized matrix multiplication for better performance
        // Exploiting the sparse structure of F and Q
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                new_P[i][j] = 0.0;
                
                // Compute F*P
                for (int k = 0; k < 5; k++) {
                    if (fabs(F[i][k]) > 1e-6) { // Only multiply non-zero elements
                        new_P[i][j] += F[i][k] * P_[k][j];
                    }
                }
            }
        }
        
        // Temporary storage for P after first multiplication
        double temp_P[5][5];
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                temp_P[i][j] = new_P[i][j];
            }
        }
        
        // Compute (F*P)*F^T
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                new_P[i][j] = 0.0;
                
                for (int k = 0; k < 5; k++) {
                    if (fabs(F[j][k]) > 1e-6) { // Only multiply non-zero elements
                        new_P[i][j] += temp_P[i][k] * F[j][k];
                    }
                }
                
                // Add process noise (only on diagonal elements)
                if (i == j) {
                    new_P[i][j] += Q_diag[i];
                }
            }
        }
        
        // Update the covariance matrix
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                P_[i][j] = new_P[i][j];
            }
        }
        
        mutex_.give();
    }

    /**
     * @brief Update step with GPS measurement [x_gps, y_gps]
     * @param x_gps measured X (inches)
     * @param y_gps measured Y (inches)
     */
    void updateGPS(double x_gps, double y_gps) {
        mutex_.take(TIMEOUT_MAX);
        
        // Measurement matrix H (for position only)
        // H = [1 0 0 0 0]
        //     [0 1 0 0 0]
        
        // Innovation (measurement residual)
        double y[2] = {
            x_gps - x_[0],
            y_gps - x_[1]
        };
        
        // Innovation covariance S = H*P*H^T + R
        // Since H has a simple structure (selecting x and y),
        // we can simplify this calculation
        double S[2][2] = {
            {P_[0][0] + R_gps[0], P_[0][1] + R_gps[1]},
            {P_[1][0] + R_gps[2], P_[1][1] + R_gps[3]}
        };
        
        // Compute determinant of S
        double det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        
        // Check for numerical stability
        if (fabs(det_S) < 1e-6) {
            mutex_.give();
            return;
        }
        
        // Compute inverse of S
        double inv_det_S = 1.0 / det_S;
        double S_inv[2][2] = {
            { S[1][1] * inv_det_S, -S[0][1] * inv_det_S},
            {-S[1][0] * inv_det_S,  S[0][0] * inv_det_S}
        };
        
        // Kalman gain K = P*H^T*S^(-1)
        // Since H selects the first two rows of P
        double K[5][2];
        for (int i = 0; i < 5; i++) {
            K[i][0] = P_[i][0] * S_inv[0][0] + P_[i][1] * S_inv[1][0];
            K[i][1] = P_[i][0] * S_inv[0][1] + P_[i][1] * S_inv[1][1];
        }
        
        // State update x = x + K*y
        for (int i = 0; i < 5; i++) {
            x_[i] += K[i][0] * y[0] + K[i][1] * y[1];
        }
        
        // Normalize angle after update
        normalizeAngle(x_[2]);
        
        // Joseph form covariance update for improved numerical stability
        // P = (I - KH)P(I - KH)^T + KRK^T
        
        // First compute I - KH
        // Since H has a simple form, KH is also simple:
        // KH = [K[0][0] K[0][1] 0 0 0]
        //      [K[1][0] K[1][1] 0 0 0]
        //      [K[2][0] K[2][1] 0 0 0]
        //      [K[3][0] K[3][1] 0 0 0]
        //      [K[4][0] K[4][1] 0 0 0]
        
        double I_KH[5][5] = {}; // Initialize to zeros
        
        // Set diagonal to 1 (for identity matrix)
        for (int i = 0; i < 5; i++) {
            I_KH[i][i] = 1.0;
        }
        
        // Subtract KH from identity
        for (int i = 0; i < 5; i++) {
            I_KH[i][0] -= K[i][0]; // Subtract first column of KH
            I_KH[i][1] -= K[i][1]; // Subtract second column of KH
        }
        
        // Compute (I-KH)P efficiently
        double temp[5][5] = {};
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                for (int k = 0; k < 5; k++) {
                    temp[i][j] += I_KH[i][k] * P_[k][j];
                }
            }
        }
        
        // Compute (I-KH)P(I-KH)^T efficiently
        double new_P[5][5] = {};
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                for (int k = 0; k < 5; k++) {
                    new_P[i][j] += temp[i][k] * I_KH[j][k]; // Note: I_KH^T[k][j] = I_KH[j][k]
                }
            }
        }
        
        // Add KRK^T
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                // Computing K*R*K^T directly for 2x2 R matrix
                double KRKt = 
                    K[i][0] * (R_gps[0] * K[j][0] + R_gps[1] * K[j][1]) +
                    K[i][1] * (R_gps[2] * K[j][0] + R_gps[3] * K[j][1]);
                
                new_P[i][j] += KRKt;
            }
        }
        
        // Update covariance matrix
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                P_[i][j] = new_P[i][j];
            }
        }
        
        mutex_.give();
    }

    /**
     * @brief Update step with gyro/IMU measurement of heading
     * @param theta_measured measured heading (radians)
     */
    void updateGyro(double theta_measured) {
        mutex_.take(TIMEOUT_MAX);
        
        // Normalize angles to [-pi, pi]
        normalizeAngle(theta_measured);
        
        // Calculate angle difference accounting for wrap-around
        double innovation = theta_measured - x_[2];
        normalizeAngle(innovation);
        
        // For a single variable update, calculations are simpler
        // The measurement matrix H selects only the theta component
        // H = [0 0 1 0 0]
        
        // Innovation covariance: S = H*P*H^T + R = P[2][2] + R_gyro
        double S = P_[2][2] + R_gyro;
        
        // Check for numerical stability
        if (fabs(S) < 1e-6) {
            mutex_.give();
            return;
        }
        
        // Kalman gain K = P*H^T*S^(-1)
        // Since H selects the third column of P
        double K[5];
        for (int i = 0; i < 5; i++) {
            K[i] = P_[i][2] / S;
        }
        
        // State update x = x + K*innovation
        for (int i = 0; i < 5; i++) {
            x_[i] += K[i] * innovation;
        }
        
        // Normalize angle again
        normalizeAngle(x_[2]);
        
        // Covariance update P = (I-KH)P
        // For this simple case, we can compute directly
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                // Subtract K[i]*P[2][j] from each element
                // This is equivalent to (I-KH)P for this special case
                P_[i][j] -= K[i] * P_[2][j];
            }
        }
        
        mutex_.give();
    }

    /**
     * @brief Update step with encoder measurement of linear velocity
     * @param v_measured measured linear velocity (inches/sec)
     */
    void updateEncoder(double v_measured) {
        mutex_.take(TIMEOUT_MAX);
        
        // Innovation (measurement residual)
        double innovation = v_measured - x_[3];
        
        // Innovation covariance: S = H*P*H^T + R = P[3][3] + R_encoder
        double S = P_[3][3] + R_encoder;
        
        // Check for numerical stability
        if (fabs(S) < 1e-6) {
            mutex_.give();
            return;
        }
        
        // Kalman gain K = P*H^T*S^(-1)
        // Since H selects the fourth column of P
        double K[5];
        for (int i = 0; i < 5; i++) {
            K[i] = P_[i][3] / S;
        }
        
        // State update x = x + K*innovation
        for (int i = 0; i < 5; i++) {
            x_[i] += K[i] * innovation;
        }
        
        // Covariance update P = (I-KH)P
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                // Subtract K[i]*P[3][j] from each element
                P_[i][j] -= K[i] * P_[3][j];
            }
        }
        
        mutex_.give();
    }

    /**
     * @brief Retrieve the current state estimate
     * @return State struct containing position, heading, and velocities
     */
    State getState() const {
        mutex_.take(TIMEOUT_MAX);
        State s{x_[0], x_[1], x_[2], x_[3], x_[4]};
        mutex_.give();
        return s;
    }

    /**
     * @brief Get the covariance for the position estimate
     * @return double The position variance (average of x and y variances)
     */
    double getPositionVariance() const {
        mutex_.take(TIMEOUT_MAX);
        double variance = (P_[0][0] + P_[1][1]) / 2.0;
        mutex_.give();
        return variance;
    }

    /**
     * @brief Get the heading variance
     * @return double The heading variance in rad^2
     */
    double getHeadingVariance() const {
        mutex_.take(TIMEOUT_MAX);
        double variance = P_[2][2];
        mutex_.give();
        return variance;
    }

    /**
     * @brief Set the process noise for the position components
     * @param q_pos Process noise value for position (inches^2)
     */
    void setPositionNoise(double q_pos) {
        mutex_.take(TIMEOUT_MAX);
        Q_diag[0] = q_pos;
        Q_diag[1] = q_pos;
        mutex_.give();
    }

    /**
     * @brief Set the process noise for the heading component
     * @param q_heading Process noise value for heading (rad^2)
     */
    void setHeadingNoise(double q_heading) {
        mutex_.take(TIMEOUT_MAX);
        Q_diag[2] = q_heading;
        mutex_.give();
    }

    /**
     * @brief Set the process noise for the velocity components
     * @param q_vel Process noise for linear velocity (inches^2/s^2)
     * @param q_omega Process noise for angular velocity (rad^2/s^2)
     */
    void setVelocityNoise(double q_vel, double q_omega) {
        mutex_.take(TIMEOUT_MAX);
        Q_diag[3] = q_vel;
        Q_diag[4] = q_omega;
        mutex_.give();
    }

    /**
     * @brief Set the measurement noise for GPS updates
     * @param r_pos Measurement noise for position (inches^2)
     */
    void setGpsNoise(double r_pos) {
        mutex_.take(TIMEOUT_MAX);
        R_gps[0] = r_pos;
        R_gps[3] = r_pos;
        mutex_.give();
    }

    /**
     * @brief Set the measurement noise for gyro updates
     * @param r_gyro Measurement noise for gyro (rad^2)
     */
    void setGyroNoise(double r_gyro) {
        mutex_.take(TIMEOUT_MAX);
        R_gyro = r_gyro;
        mutex_.give();
    }

    /**
     * @brief Set the measurement noise for encoder updates
     * @param r_encoder Measurement noise for encoder (inches^2/s^2)
     */
    void setEncoderNoise(double r_encoder) {
        mutex_.take(TIMEOUT_MAX);
        R_encoder = r_encoder;
        mutex_.give();
    }

private:
    // State vector [x, y, theta, v, omega]
    double x_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // State covariance matrix (5x5)
    double P_[5][5] = {};
    
    // Process noise (diagonal elements only for efficiency)
    double Q_diag[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // GPS measurement noise (2x2 symmetric matrix stored as 4 elements)
    // [R_gps[0], R_gps[1]]
    // [R_gps[2], R_gps[3]]
    double R_gps[4] = {0.0, 0.0, 0.0, 0.0};
    
    // Gyro measurement noise (scalar)
    double R_gyro = 0.0;
    
    // Encoder measurement noise (scalar)
    double R_encoder = 0.0;
    
    // Mutex for thread safety
    mutable pros::Mutex mutex_;

    /**
     * @brief Normalize angle to the range [-pi, pi]
     * @param angle The angle to normalize (modified in-place)
     */
    static void normalizeAngle(double& angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
    }
};
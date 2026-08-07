#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

// Allocation-free EKF for differential-drive pose [x_mm, y_mm, heading_rad].
// Wheel displacement predicts the pose and the IMU yaw corrects its heading.
// Only the six unique covariance entries are stored to keep AVR RAM/stack use low.
class ExtendedKalmanFilter {
public:
    explicit ExtendedKalmanFilter(float wheel_base_mm)
        : wheel_base_mm(wheel_base_mm > 1.0f ? wheel_base_mm : 1.0f) {
        reset();
    }

    void reset(float initial_x_mm = 0.0f,
               float initial_y_mm = 0.0f,
               float initial_heading_deg = 0.0f) {
        x_mm = initial_x_mm;
        y_mm = initial_y_mm;
        heading_rad = wrapRadians(radians(initial_heading_deg));
        resetCovariance();
        initialised = false;
    }

    void begin(float imu_heading_deg) {
        heading_rad = wrapRadians(radians(imu_heading_deg));
        initialised = true;
    }

    // Inputs are signed wheel distances: positive means that wheel moved
    // forward, regardless of the electrical motor/encoder wiring polarity.
    void updateWheelDistances(float delta_left_mm,
                              float delta_right_mm,
                              float imu_heading_deg) {
        if (!initialised) {
            begin(imu_heading_deg);
            return;
        }

        if (isFinite(delta_left_mm) && isFinite(delta_right_mm)) {
            predict(delta_left_mm, delta_right_mm);
        }

        if (isFinite(imu_heading_deg)) {
            correctHeading(radians(imu_heading_deg));
        }

        if (!stateIsFinite()) {
            // Keep motor control safe if a bad sensor value ever corrupts the
            // filter. Position is restarted because it is not yet used to stop.
            x_mm = 0.0f;
            y_mm = 0.0f;
            heading_rad = isFinite(imu_heading_deg)
                ? wrapRadians(radians(imu_heading_deg))
                : 0.0f;
            resetCovariance();
        }
    }

    float getXMM() const { return x_mm; }
    float getYMM() const { return y_mm; }
    float getHeadingDeg() const { return degrees(heading_rad); }
    bool isInitialised() const { return initialised; }

    // Wheel values are variances per millimetre travelled.
    void setWheelNoise(float left_variance_per_mm,
                       float right_variance_per_mm) {
        left_wheel_noise = left_variance_per_mm > 0.0f
            ? left_variance_per_mm
            : 0.0f;
        right_wheel_noise = right_variance_per_mm > 0.0f
            ? right_variance_per_mm
            : 0.0f;
    }

    void setImuHeadingNoiseDeg(float standard_deviation_deg) {
        float sigma_deg = standard_deviation_deg > 0.1f
            ? standard_deviation_deg
            : 0.1f;
        float sigma_rad = radians(sigma_deg);
        imu_heading_variance = sigma_rad * sigma_rad;
    }

private:
    static float wrapRadians(float angle_rad) {
        while (angle_rad > PI) angle_rad -= TWO_PI;
        while (angle_rad < -PI) angle_rad += TWO_PI;
        return angle_rad;
    }

    static bool isFinite(float value) {
        return !isnan(value) && !isinf(value);
    }

    void resetCovariance() {
        p00 = 4.0f;  // 2 mm initial standard deviation
        p01 = 0.0f;
        p02 = 0.0f;
        p11 = 4.0f;
        p12 = 0.0f;
        float initial_heading_sigma = radians(2.0f);
        p22 = initial_heading_sigma * initial_heading_sigma;
    }

    bool stateIsFinite() const {
        return isFinite(x_mm) && isFinite(y_mm) &&
               isFinite(heading_rad) &&
               isFinite(p00) && isFinite(p01) && isFinite(p02) &&
               isFinite(p11) && isFinite(p12) && isFinite(p22);
    }

    void predict(float delta_left_mm, float delta_right_mm) {
        const float distance_mm =
            0.5f * (delta_left_mm + delta_right_mm);
        const float delta_heading_rad =
            (delta_right_mm - delta_left_mm) / wheel_base_mm;
        const float midpoint_heading_rad =
            heading_rad + 0.5f * delta_heading_rad;
        const float cos_heading = cos(midpoint_heading_rad);
        const float sin_heading = sin(midpoint_heading_rad);

        x_mm += distance_mm * cos_heading;
        y_mm += distance_mm * sin_heading;
        heading_rad = wrapRadians(heading_rad + delta_heading_rad);

        // F * P * F^T for F = [[1,0,a],[0,1,b],[0,0,1]].
        const float a = -distance_mm * sin_heading;
        const float b = distance_mm * cos_heading;
        float next_p00 = p00 + 2.0f * a * p02 + a * a * p22;
        float next_p01 = p01 + b * p02 + a * p12 + a * b * p22;
        float next_p02 = p02 + a * p22;
        float next_p11 = p11 + 2.0f * b * p12 + b * b * p22;
        float next_p12 = p12 + b * p22;
        float next_p22 = p22;

        // Complete first-order G * Qwheel * G^T propagation. Unlike the
        // pasted version, this adds no process noise while both wheels stop.
        const float left_variance =
            left_wheel_noise * fabs(delta_left_mm);
        const float right_variance =
            right_wheel_noise * fabs(delta_right_mm);
        const float half_distance_over_base =
            0.5f * distance_mm / wheel_base_mm;

        const float gx_left =
            0.5f * cos_heading + half_distance_over_base * sin_heading;
        const float gx_right =
            0.5f * cos_heading - half_distance_over_base * sin_heading;
        const float gy_left =
            0.5f * sin_heading - half_distance_over_base * cos_heading;
        const float gy_right =
            0.5f * sin_heading + half_distance_over_base * cos_heading;
        const float gh_left = -1.0f / wheel_base_mm;
        const float gh_right = 1.0f / wheel_base_mm;

        next_p00 += left_variance * gx_left * gx_left +
                    right_variance * gx_right * gx_right;
        next_p01 += left_variance * gx_left * gy_left +
                    right_variance * gx_right * gy_right;
        next_p02 += left_variance * gx_left * gh_left +
                    right_variance * gx_right * gh_right;
        next_p11 += left_variance * gy_left * gy_left +
                    right_variance * gy_right * gy_right;
        next_p12 += left_variance * gy_left * gh_left +
                    right_variance * gy_right * gh_right;
        next_p22 += left_variance * gh_left * gh_left +
                    right_variance * gh_right * gh_right;

        p00 = next_p00;
        p01 = next_p01;
        p02 = next_p02;
        p11 = next_p11;
        p12 = next_p12;
        p22 = next_p22;
    }

    void correctHeading(float measured_heading_rad) {
        const float innovation =
            wrapRadians(measured_heading_rad - heading_rad);
        const float innovation_variance = p22 + imu_heading_variance;

        if (!isFinite(innovation_variance) ||
            innovation_variance <= 0.0f) {
            return;
        }

        const float old_p02 = p02;
        const float old_p12 = p12;
        const float old_p22 = p22;
        const float gain_x = old_p02 / innovation_variance;
        const float gain_y = old_p12 / innovation_variance;
        const float gain_heading = old_p22 / innovation_variance;

        x_mm += gain_x * innovation;
        y_mm += gain_y * innovation;
        heading_rad = wrapRadians(
            heading_rad + gain_heading * innovation
        );

        p00 -= gain_x * old_p02;
        p01 -= gain_x * old_p12;
        p02 -= gain_x * old_p22;
        p11 -= gain_y * old_p12;
        p12 -= gain_y * old_p22;
        p22 -= gain_heading * old_p22;

        // Protect the single-precision AVR implementation from tiny negative
        // diagonal values caused by round-off.
        if (p00 < 0.000001f) p00 = 0.000001f;
        if (p11 < 0.000001f) p11 = 0.000001f;
        if (p22 < 0.000001f) p22 = 0.000001f;
    }

    const float wheel_base_mm;

    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float heading_rad = 0.0f;

    // Symmetric covariance: p10=p01, p20=p02, p21=p12.
    float p00 = 0.0f;
    float p01 = 0.0f;
    float p02 = 0.0f;
    float p11 = 0.0f;
    float p12 = 0.0f;
    float p22 = 0.0f;

    float left_wheel_noise = 0.8f;
    float right_wheel_noise = 0.8f;
    float imu_heading_variance =
        radians(3.0f) * radians(3.0f);
    bool initialised = false;
};

}  // namespace mtrn3100

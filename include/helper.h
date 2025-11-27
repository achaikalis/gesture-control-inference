/**
 * @file helper.h
 * @brief Helper functions for processing MPU9250 IMU data
 *
 * @note This file is designed to work with the SparkFun MPU9250-DMP library
 and provides C compatibility through extern "C" guards.
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <SparkFunMPU9250-DMP.h>

typedef struct {
  float w, x, y, z; /* Quaternion := {qw, qx, qy, qz} */
} Quaternion;

typedef struct {
  float roll;  /* rotation around x-axis in degrees */
  float pitch; /* rotation around y-axis in degrees */
  float yaw;   /* rotation around z-axis in degrees */
} EulerAngles;

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

extern Quaternion initial_quaternion;
extern MPU9250_DMP imu;

#ifdef __cplusplus
}

#endif /* End of CPP Guard */

/**
 * @brief Computes the initial quaternion orientation from magnetometer readings.
 * 
 * This function calculates the initial orientation quaternion based on the magnetometer
 * readings to establish a reference frame aligned with magnetic north.
 * 
 * @param mx Magnetometer reading along the x-axis.
 * @param my Magnetometer reading along the y-axis.
 * @return Quaternion The initial orientation quaternion.
 */
Quaternion ComputeInitialQuaternions(float mx, float my);

/**
 * @brief Converts a quaternion to Euler angles.
 * 
 * This function transforms a quaternion representation of orientation to
 * the corresponding Euler angles (roll, pitch, yaw) in degrees.
 * 
 * @param q The quaternion to convert.
 * @return EulerAngles The resulting Euler angles in degrees.
 */
EulerAngles QuaternionToEuler(Quaternion q);

/**
 * @brief Multiplies two quaternions together.
 * 
 * This function performs quaternion multiplication of q1 and q2, which effectively
 * combines the two rotations. It is used to apply the initial orientation quaternion
 * to subsequent orientation measurements relative to magnetic north.
 * 
 * @param q1 The first quaternion (typically the initial quaternion).
 * @param q2 The second quaternion (typically the new measurement).
 * @return Quaternion The resulting quaternion from the multiplication.
 */
Quaternion MultiplyQuaternions(Quaternion q1, Quaternion q2);

/**
 * @brief Prints the current IMU data to the serial port.
 * 
 * This function should be called after dmpUpdateFifo() has been invoked to ensure
 * that the latest IMU data is available. It formats and outputs accelerometer,
 * gyroscope, magnetometer, and quaternion values to the serial port.
 * 
 * @note The quaternion values from the IMU are stored in Q30 long format and
 *       are converted to floating-point values between -1 and 1.
 */
int PrintIMUData (void);

/**
 * @brief      Run Continuous Inference
 *
 * @param[in]  format     raw_features_buffer
 * @param[in]  format     raw_features_buffer_size
 */
extern int run_continuous_inference(float* raw_features_buffer,
                                    size_t raw_features_buffer_size);

#endif /* HELPER_H_ */

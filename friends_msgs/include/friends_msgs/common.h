
#ifndef FRIENDS_MSGS_COMMON_H
#define FRIENDS_MSGS_COMMON_H

#include <Eigen/Geometry>

namespace friends_msgs {

const double kSmallValueCheck = 1.e-6;
const double kNumNanosecondsPerSecond = 1.e9;

/// Magnitude of Earth's gravitational field at specific height [m] and latitude
/// [rad] (from wikipedia).
inline double MagnitudeOfGravity(const double height,
                                 const double latitude_radians) {
  // gravity calculation constants
  const double kGravity_0 = 9.780327;
  const double kGravity_a = 0.0053024;
  const double kGravity_b = 0.0000058;
  const double kGravity_c = 3.155 * 1e-7;

  double sin_squared_latitude = std::sin(latitude_radians) * std::sin(latitude_radians);
  double sin_squared_twice_latitude =
      std::sin(2 * latitude_radians) * std::sin(2 * latitude_radians);
  return kGravity_0 * ((1 + kGravity_a * sin_squared_latitude -
                        kGravity_b * sin_squared_twice_latitude) -
                       kGravity_c * height);
}

/**
 * \brief Extracts the yaw part from a quaternion, using RPY / euler (z-y'-z'')
 * angles.
 * RPY rotates about the fixed axes in the order x-y-z,
 * which is the same as euler angles in the order z-y'-x''.
 */
inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
  {
    assert(euler_angles != NULL);

    *euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        std::asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
        std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
}

inline double nanosecondsToSeconds(int64_t nanoseconds) {
  double seconds = nanoseconds / kNumNanosecondsPerSecond;
  return seconds;
}

inline int64_t secondsToNanoseconds(double seconds) {
  int64_t nanoseconds =
      static_cast<int64_t>(seconds * kNumNanosecondsPerSecond);
  return nanoseconds;
}

}  // namespace friends_msgs

#endif  // FRIENDS_MSGS_COMMON_H

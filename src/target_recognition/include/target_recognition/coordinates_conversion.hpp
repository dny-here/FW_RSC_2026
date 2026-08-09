#ifndef TARGET_RECOGNITION__COORDINATES_CONVERSION_HPP_
#define TARGET_RECOGNITION__COORDINATES_CONVERSION_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <cmath>
#include <iostream>

namespace target_recognition_cpp {

// WGS84 approximate Earth radius in meters
const double EARTH_RADIUS = 6378137.0;

/**
 * @class TargetFilter2D
 * @brief A 2D Kalman Filter designed to smooth noisy North/East coordinate estimates.
 * * Georeferenced pixels are highly sensitive to UAV attitude (roll/pitch) jitter.
 * This filter assumes the physical target is stationary on the ground, allowing it
 * to smooth out high-frequency measurement noise caused by the camera and IMU.
 */
class TargetFilter2D {
private:
    cv::KalmanFilter kf;
    bool is_initialized;

public:
    TargetFilter2D() : is_initialized(false) {
        // State: [North, East]. Measurement: [North, East]. No velocity terms (stationary target).
        kf = cv::KalmanFilter(2, 2, 0, CV_64F);

        cv::setIdentity(kf.transitionMatrix);
        cv::setIdentity(kf.measurementMatrix);

        // Process Noise: Very low (1e-4) because the actual physical target does not move.
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));

        // Measurement Noise: High (25.0) to distrust individual frames and favor the smoothed average.
        // This compensates for gimbal vibrations and pixel resolution limits.
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(5.0)); 

        // Initial uncertainty: High, allowing the filter to quickly snap to the first few measurements.
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(100.0));
    }

    cv::Point2d update(double measured_N, double measured_E) {
        if (!is_initialized) {
            kf.statePost.at<double>(0) = measured_N;
            kf.statePost.at<double>(1) = measured_E;
            kf.statePre.at<double>(0) = measured_N; // Add this
            kf.statePre.at<double>(1) = measured_E; // Add this
            is_initialized = true;
            return cv::Point2d(measured_N, measured_E);
        }

        kf.predict();
        cv::Mat measurement = (cv::Mat_<double>(2, 1) << measured_N, measured_E);
        cv::Mat estimated_state = kf.correct(measurement);

        return cv::Point2d(estimated_state.at<double>(0), estimated_state.at<double>(1));
    }

    void reset() {
        is_initialized = false;
        
        // Reset the uncertainty to high so it snaps quickly to the new target
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(100.0));
        cv::setIdentity(kf.errorCovPre, cv::Scalar::all(100.0));
    }
};

/**
 * @brief Calculates the local North-East-Down (NED) coordinates of a target pixel.
 * * Uses the Flat-Earth Assumption. It projects a ray from the camera through the pixel
 * and calculates where it intersects the ground plane (where altitude = 0).
 */
bool getFlatTargetNED(
    const cv::Point2f& image_point, 
    const cv::Mat& K, 
    const cv::Mat& dist_coeffs,
    const TelemetryParams& tel,
    cv::Point2d& out_ned) 
{
    // 1. Undistort pixel based on camera lens calibration parameters
    std::vector<cv::Point2f> src_pts = { image_point };
    std::vector<cv::Point2f> undistorted_pts;
    cv::undistortPoints(src_pts, undistorted_pts, K, dist_coeffs);
    cv::Point2f pt = undistorted_pts[0];

    // 2. Build Rotation Matrices (Coordinate Frame Transformations)
    double cp = cos(tel.pitch), sp = sin(tel.pitch);
    double cr = cos(tel.roll),  sr = sin(tel.roll);
    double cy = cos(tel.yaw),   sy = sin(tel.yaw);

    // Rotation: Aircraft Body frame to Local NED frame
    cv::Matx33d R_body_ned(
        cp*cy, -cr*sy + sr*sp*cy,  sr*sy + cr*sp*cy,
        cp*sy,  cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy,
       -sp,     sr*cp,             cr*cp
    );

    // Rotation: Camera Mount to Aircraft Body (Fixed 90-degree Z-rotation depending on hardware)
    cv::Matx33d R_body_mount(0, 1, 0, -1, 0, 0, 0, 0, 1);

    // Rotation: Camera Lens to Camera Mount (Gimbal compensation)
    double c_pan = cos(tel.gimbal_pan), s_pan = sin(tel.gimbal_pan);
    double c_tilt = cos(tel.gimbal_tilt), s_tilt = sin(tel.gimbal_tilt);
    cv::Matx33d R_mount_cam(
         c_pan,         s_pan,         0,
        -s_pan*c_tilt,  c_pan*c_tilt,  s_tilt,
         s_tilt*s_pan, -s_tilt*c_pan,  c_tilt
    );

    // Chain the rotations together: Camera -> Mount -> Body -> NED
    cv::Matx33d R_cam_body = (R_mount_cam * R_body_mount).t();
    cv::Matx33d R_ned_cam = (R_body_ned * R_cam_body).t();

    // 3. Translation Vector: The camera's position relative to the target's ground plane
    cv::Vec3d C(0.0, 0.0, -tel.alt);
    cv::Vec3d t = -R_ned_cam * C;

    // 4. Construct the Homography Matrix (G_NE)
    // By assuming the target is flat on the ground (Z=0), we can drop the 3rd column
    // of the rotation matrix and replace it with the translation vector.
    cv::Matx33d G_NE(
        R_ned_cam(0,0), R_ned_cam(0,1), t(0),
        R_ned_cam(1,0), R_ned_cam(1,1), t(1),
        R_ned_cam(2,0), R_ned_cam(2,1), t(2)
    );

    // Prevent mathematically impossible inversions (e.g., if the camera points perfectly at the horizon)
    double det = cv::determinant(G_NE);
    if (std::abs(det) < 1e-6) return false;

    cv::Matx33d G_NE_inv = G_NE.inv();

    // 5. Calculate final local coordinates by projecting the ray to the ground plane
    cv::Vec3d cam_ray(pt.x, pt.y, 1.0);
    cv::Vec3d ned_homogenous = G_NE_inv * cam_ray;

    // Divide by the scaling factor (lambda) to normalize homogeneous coordinates
    out_ned.x = ned_homogenous(0) / ned_homogenous(2); // North offset in meters
    out_ned.y = ned_homogenous(1) / ned_homogenous(2); // East offset in meters

    return true;
}

/**
 * @brief Converts a local Cartesian offset (North/East meters) to Global WGS84 GPS coordinates.
 * * Uses a spherical earth approximation. Safe for short distances (e.g., visual line of sight),
 * which fits the payload drop profile perfectly.
 */
GPSCoordinate localNEDToGlobalGPS(const cv::Point2d& local_ned, const TelemetryParams& tel) {
    GPSCoordinate target_gps;
    
    // Convert meters to decimal degrees
    double lat_offset = (local_ned.x / EARTH_RADIUS) * (180.0 / M_PI);
    
    // Longitude must be scaled by the cosine of the current latitude to account for meridian convergence
    double lon_offset = (local_ned.y / (EARTH_RADIUS * cos(tel.lat * M_PI / 180.0))) * (180.0 / M_PI);

    target_gps.lat = tel.lat + lat_offset;
    target_gps.lon = tel.lon + lon_offset;
    
    // Altitude is inherited from the reference point (assumed ground level)
    target_gps.alt = 0.0; 

    return target_gps;
}

} // namespace target_recognition_cpp

#endif
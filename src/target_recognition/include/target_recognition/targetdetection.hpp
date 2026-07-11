// targetdetection.hpp
// Deklarasi awal dari variabel-variabel yang digunakan
// di targetdetection.cpp

#ifndef TARGET_RECOGNITION__TARGET_DETECTION_HPP_
#define TARGET_RECOGNITION__TARGET_DETECTION_HPP_

// OpenCV Dependencies
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace target_recognition_cpp {

/**
 * @struct TelemetryParams
 * @brief Represents the instantaneous pose and position of the UAV.
 * * This data must be synchronized as closely as possible with the camera frame.
 * It provides the necessary extrinsic parameters to build the rotation matrices
 * for projecting a 2D pixel ray down to the Earth's surface.
 */
struct TelemetryParams {
    double lat;            // Current WGS84 latitude [deg]
    double lon;            // Current WGS84 longitude [deg]
    double alt;            // Altitude above ground/sea level [m]
    double roll;           // Aircraft roll [rad]
    double pitch;          // Aircraft pitch [rad]
    double yaw;            // Aircraft yaw/heading [rad]
    double gimbal_pan{0};  // Gimbal offset (if camera is not fixed forward/down) [rad]
    double gimbal_tilt{0}; // Gimbal offset [rad]
};

/**
 * @struct GPSCoordinate
 * @brief The final computed global position of the detected target.
 * * This is the exact payload expected by the Airdrop Planning node.
 */
struct GPSCoordinate {
    double lat; 
    double lon; 
    double alt; // Typically 0.0 for flat-earth assumption at the drop zone
};

/**
 * @struct ThresholdingParams
 * @brief HSV color bounds for isolating the target (e.g., orange tarp) from the background.
 * * HSV (Hue, Saturation, Value) is used instead of RGB because it isolates color (Hue) 
 * from lighting conditions (Value), making detection much more robust against 
 * cloud shadows and varying sunlight during flight.
 */
struct ThresholdingParams {
    int low_H;  // Lower color boundary
    int low_S;  // Minimum color intensity/purity
    int low_V;  // Minimum brightness (filters out dark shadows)
    int high_H; // Upper color boundary
    int high_S; 
    int high_V; 
};

/**
 * @enum State
 * @brief The Finite State Machine (FSM) controlling the vision node's behavior.
 * * This FSM prevents the UAV from dropping the payload based on a single false-positive 
 * image frame. It enforces a "temporal filter" by requiring continuous observation 
 * before committing to a drop coordinate.
 */
enum State {
    // Actively scanning camera frames for a valid contour.
    STATE_SEARCHING, 
    
    // Target spotted. Accumulating position data to feed the Kalman Filter.
    STATE_GATHERING, 
    
    // Sufficient data gathered (e.g., 75 frames). Coordinate is finalized and sent to Planner.
    STATE_LOCKED, 
    
    // Vision processing suspended. Monitoring the Planner's approach and drop sequence.
    STATE_WAITING, 
    
    // Planner aborted or target lost during gathering. Resetting to try again.
    STATE_FAILED 
};

} // namespace target_recognition_cpp

#endif
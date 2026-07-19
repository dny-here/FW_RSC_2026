#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "cv_bridge/cv_bridge.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Action and Custom Message Dependencies
#include "interfaces/msg/target_estimate.hpp"
#include "interfaces/msg/airdrop_status.hpp"
#include "interfaces/action/target_airdrop.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Local Project Dependencies
#include "target_recognition/targetdetection.hpp"
#include "target_recognition/coordinates_conversion.hpp"

using namespace std::chrono_literals;
using namespace cv;
using std::placeholders::_1;
using std::placeholders::_2;

namespace target_recognition_cpp
{

class TargetDetectionNode : public rclcpp::Node
{
public:
    using TargetAirdrop = interfaces::action::TargetAirdrop;
    using GoalHandleTargetAirdrop = rclcpp_action::ClientGoalHandle<TargetAirdrop>;

    TargetDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("target_recognition_node", options)
    {
        declareParams();
        loadParams();

        gui_timer_ = this->create_wall_timer(
        33ms, std::bind(&TargetDetectionNode::updateGuiLoop, this));

        // Use SensorDataQoS to handle high-frequency topics (like video) by dropping old messages if lagging
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        
        // Initialize the Action Client to communicate with the airdrop_planning node
        action_client_ = rclcpp_action::create_client<TargetAirdrop>(
            this, "execute_airdrop"
        );

        // Telemetry Subscribers
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
             get_parameter("topics.odom").as_string(), qos_profile, 
             std::bind(&TargetDetectionNode::loadOdom, this, _1));

        subscription_position_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
             get_parameter("topics.position").as_string(), qos_profile, 
             std::bind(&TargetDetectionNode::loadPosition, this, _1));

        // High-speed camera subscriber driving the main node logic
        subscription_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
             get_parameter("topics.camera").as_string(), qos_profile, 
             std::bind(&TargetDetectionNode::cameraCallback, this, _1));
             
        // Initialize mission state
        state_ = STATE_SEARCHING;
        detection_counter_ = 0;
    }

private:
   // -------------- State Variabels -----------------------
    ThresholdingParams thresholding_params;
    rclcpp::TimerBase::SharedPtr gui_timer_;
    bool windows_initialized_ = false;
    cv::Mat current_frame_;
    cv::Mat current_threshold_;
    cv::Mat distortion_params;
    cv::Mat camera_params;
    
    TelemetryParams telemetry_params;
    TargetFilter2D dropZoneFilter; 
    
    State state_;
    uint8_t active_dz_{0};
    int detection_counter_; // Tracks successful Kalman filter updates
    GPSCoordinate final_target_;

    cv::Mat prev_gray_frame_;
    std::vector<cv::Point2f> prev_features_;
    TelemetryParams prev_telemetry_;
    const double PARALLAX_DISTANCE_THRESHOLD = 2.0; 
    
    // This will hold the live height to feed the action server
    double estimated_object_height_ = 0.0;
    
    // -------------- ROS2 Pointers -----------------------
    rclcpp_action::Client<TargetAirdrop>::SharedPtr action_client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_camera_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_position_;

    // -------------- Initialization -----------------------
    void declareParams()
    {
        // --- HSV Color Thresholding Parameters ---
        this->declare_parameter("thresholding.high_H", 25);
        this->declare_parameter("thresholding.high_S", 255);
        this->declare_parameter("thresholding.high_V", 255);
        this->declare_parameter("thresholding.low_H", 10);
        this->declare_parameter("thresholding.low_S", 100);
        this->declare_parameter("thresholding.low_V", 100);

        // --- Camera Lens Radial/Tangential Distortion Coefficients ---
        this->declare_parameter("distortion.coef_1", -0.41802327018241026);
        this->declare_parameter("distortion.coef_2", 0.50715243805833121);
        this->declare_parameter("distortion.coef_3", 0.0);
        this->declare_parameter("distortion.coef_4", 0.0);
        this->declare_parameter("distortion.coef_5", -0.57843596847939704);

        // --- Intrinsic Camera Matrix Parameters ---
        this->declare_parameter("camera.mat_11", 657.46697810243404);
        this->declare_parameter("camera.mat_12", 0.0);
        this->declare_parameter("camera.mat_13", 319.5);
        this->declare_parameter("camera.mat_21", 0.0);
        this->declare_parameter("camera.mat_22", 657.46697810243404);
        this->declare_parameter("camera.mat_23", 239.5);
        this->declare_parameter("camera.mat_31", 0.0);
        this->declare_parameter("camera.mat_32", 0.0);
        this->declare_parameter("camera.mat_33", 1.0);

        // --- Topic Subscriptions Mapping ---
        this->declare_parameter("topics.odom", "mavros/local_position/odom");
        this->declare_parameter("topics.camera", "camera_node");
        this->declare_parameter("topics.position", "mavros/global_position/global");
    }

    void loadParams()
    {
        // --- Fetch HSV Parameters ---
        thresholding_params.high_H = this->get_parameter("thresholding.high_H").as_int();
        thresholding_params.high_S = this->get_parameter("thresholding.high_S").as_int();
        thresholding_params.high_V = this->get_parameter("thresholding.high_V").as_int();
        thresholding_params.low_H = this->get_parameter("thresholding.low_H").as_int();
        thresholding_params.low_S = this->get_parameter("thresholding.low_S").as_int();
        thresholding_params.low_V = this->get_parameter("thresholding.low_V").as_int();

        // --- Fetch and Build 1x5 Distortion Vector ---
        distortion_params = (cv::Mat_<double>(1, 5) << 
            this->get_parameter("distortion.coef_1").as_double(),
            this->get_parameter("distortion.coef_2").as_double(),
            this->get_parameter("distortion.coef_3").as_double(),
            this->get_parameter("distortion.coef_4").as_double(),
            this->get_parameter("distortion.coef_5").as_double()
        );

        // --- Fetch and Build 3x3 Intrinsic Camera Matrix (K) ---
        camera_params = (cv::Mat_<double>(3, 3) << 
            this->get_parameter("camera.mat_11").as_double(), this->get_parameter("camera.mat_12").as_double(), this->get_parameter("camera.mat_13").as_double(),
            this->get_parameter("camera.mat_21").as_double(), this->get_parameter("camera.mat_22").as_double(), this->get_parameter("camera.mat_23").as_double(),
            this->get_parameter("camera.mat_31").as_double(), this->get_parameter("camera.mat_32").as_double(), this->get_parameter("camera.mat_33").as_double()
        );
    }

    // -------------- Telemtery Loading -----------------------
    // Continuously updates the UAV's attitude (Roll, Pitch, Yaw) for georeferencing
    void loadOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(this->telemetry_params.roll, this->telemetry_params.pitch, this->telemetry_params.yaw);
    }

    // Continuously updates the UAV's origin GPS anchor for local-to-global conversion
    void loadPosition(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        this->telemetry_params.lat = msg->latitude;
        this->telemetry_params.lon = msg->longitude;
        this->telemetry_params.alt = msg->altitude;
    }

    // -------------- Camera Loading -----------------------
    void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        current_frame_ = frame.clone();
        
        // CPU Optimization: Suspend expensive image processing while the planner executes the drop
        if (state_ == STATE_LOCKED || state_ == STATE_WAITING) return;

        // Isolate visual targets from the frame
        std::vector<cv::Point2f> valid_targets = processImage(frame);

        // Define the target bounding box for the optical flow
        cv::Rect target_rect;
        if (!valid_targets.empty()) {
            // Create a roughly 40x40 pixel box around the center coordinate
            target_rect = cv::Rect(valid_targets[0].x - 20, valid_targets[0].y - 20, 40, 40);
        }

        // Run the background height estimator
        estimateHeight(frame, telemetry_params, target_rect);

        // Feed targets into the state machine to determine mission progress
        updateMissionState(valid_targets);

    }

    void updateGuiLoop()
    {
        // Initialize windows once the node is actively spinning in the main loop
        if (!windows_initialized_) {
            cv::namedWindow("Video Capture", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Object Detection", cv::WINDOW_AUTOSIZE);
            windows_initialized_ = true;
        }

        // If frames are available, update the display matrix; otherwise show a clean fallback buffer
        if (!current_frame_.empty()) {
            cv::imshow("Video Capture", current_frame_);
        } else {
            cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
            cv::putText(blank, "Waiting for video stream...", cv::Point(50, 240), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Video Capture", blank);
        }

        if (!current_threshold_.empty()) {
            cv::imshow("Object Detection", current_threshold_);
        }

        // Force the OS X11/Wayland window server to process the window events
        cv::waitKey(1);
    }

    // -------------- Image Processing -----------------------
    std::vector<cv::Point2f> processImage(cv::Mat& frame)
    {
        cv::Mat frame_HSV, frame_threshold;
        std::vector<cv::Point2f> valid_centers;

        // Convert to HSV for robust color thresholding against varied lighting
        cv::cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        cv::inRange(frame_HSV, 
                    cv::Scalar(thresholding_params.low_H, thresholding_params.low_S, thresholding_params.low_V), 
                    cv::Scalar(thresholding_params.high_H, thresholding_params.high_S, thresholding_params.high_V), 
                    frame_threshold);

        std::vector<std::vector<Point>> contours;
        findContours(frame_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            // Noise filter: Ignore tiny speckles
            if (contourArea(contour) < 50) continue; 
            
            Rect bound = boundingRect(contour);
            
            // False-positive filter: Ignore edges to prevent locking onto the horizon during sharp banks
            if (bound.x < 60 || bound.x > (frame.cols - 60) || 
                bound.y < 30 || bound.y > (frame.rows - 30)) {
                continue; 
            }

            float center_x = bound.x + (bound.width / 2.0f);
            float center_y = bound.y + (bound.height / 2.0f);
            valid_centers.push_back(cv::Point2f(center_x, center_y));

            // Draw visual debug markers
            rectangle(frame, bound.tl(), bound.br(), cv::Scalar(0, 255, 0), 2);
            circle(frame, cv::Point2f(center_x, center_y), 4, cv::Scalar(0, 255, 0), -1); 
        }

        current_threshold_ = frame_threshold.clone();
        

        return valid_centers;
    }

    // -------------- Height Estimation -----------------------
    double calculatePhysicalTranslation(const TelemetryParams& t1, const TelemetryParams& t2) {
        double dy = (t2.lat - t1.lat) * 111320.0;
        double dx = (t2.lon - t1.lon) * 111320.0 * cos(t1.lat * M_PI / 180.0);
        return std::sqrt(dx*dx + dy*dy);
    }

    void estimateHeight(const cv::Mat& current_frame, const TelemetryParams& current_telemetry, const cv::Rect& target_box) {
        cv::Mat current_gray;
        cv::cvtColor(current_frame, current_gray, cv::COLOR_BGR2GRAY);

        // Initialize first frame
        if (prev_gray_frame_.empty() || prev_features_.empty()) {
            prev_gray_frame_ = current_gray.clone();
            prev_telemetry_ = current_telemetry;
            cv::goodFeaturesToTrack(prev_gray_frame_, prev_features_, 100, 0.01, 10);
            return;
        }

        // Wait for enough physical movement (Parallax)
        double translation = calculatePhysicalTranslation(prev_telemetry_, current_telemetry);
        if (translation < PARALLAX_DISTANCE_THRESHOLD) return;

        std::vector<cv::Point2f> current_features;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray_frame_, current_gray, prev_features_, current_features, status, err);

        double ground_sum = 0.0, object_sum = 0.0;
        int ground_count = 0, object_count = 0;
        double fx = camera_params.at<double>(0, 0); 

        for (size_t i = 0; i < current_features.size(); i++) {
            if (status[i]) {
                double displacement = cv::norm(current_features[i] - prev_features_[i]);
                if (displacement > 1.0) {
                    double distance = fx * (translation / displacement);
                    if (target_box.contains(current_features[i])) {
                        object_sum += distance;
                        object_count++;
                    } else {
                        ground_sum += distance;
                        ground_count++;
                    }
                }
            }
        }

        if (ground_count > 5 && object_count > 3) {
            double avg_ground = ground_sum / ground_count;
            double avg_object = object_sum / object_count;
            double absolute_height = avg_ground - avg_object;

            if (absolute_height > 0.0 && absolute_height < 50.0) { // Sanity check max height
                estimated_object_height_ = absolute_height;
            }
        }

        // Cycle the buffers
        prev_gray_frame_ = current_gray.clone();
        prev_telemetry_ = current_telemetry;
        cv::goodFeaturesToTrack(prev_gray_frame_, prev_features_, 100, 0.01, 10);
    }

    // -------------- Georeferencing -----------------------
    bool calculateTargetEstimate(const cv::Point2f& pixel_center, GPSCoordinate& out_gps)
    {
        cv::Point2d raw_ned;
        
        // Convert 2D pixel to local North/East meters using the flat-earth assumption
        if (getFlatTargetNED(pixel_center, camera_params, distortion_params, telemetry_params, raw_ned)) {
            // Pass through Kalman Filter to smooth jitter caused by UAV vibrations
            cv::Point2d filtered_ned = dropZoneFilter.update(raw_ned.x, raw_ned.y);
            
            // Map the smoothed local coordinate back to global GPS for the planner
            out_gps = localNEDToGlobalGPS(filtered_ned, telemetry_params);
            out_gps.alt = estimated_object_height_;
            return true;
        }
        return false;
    }

    // -------------- Finite State Machine -----------------------
    void updateMissionState(const std::vector<cv::Point2f>& detected_targets)
    {
        switch (state_) {
            case STATE_SEARCHING:
                // Transition to gathering data once a valid target enters the FOV
                if (!detected_targets.empty()) {
                    state_ = STATE_GATHERING;
                    RCLCPP_INFO(this->get_logger(), "Target spotted. Entering GATHERING phase.");
                }
                break;

            case STATE_GATHERING:
                if (detected_targets.empty()) {
                    // Safety reset: If target is lost before lock, discard data to prevent inaccurate drops
                    RCLCPP_WARN(this->get_logger(), "Lost target during gathering. Resetting...");
                    detection_counter_ = 0;
                    state_ = STATE_SEARCHING;
                    return;
                }

                // Process the primary target found in this frame
                if (calculateTargetEstimate(detected_targets[0], final_target_)) {
                    detection_counter_++;
                    
                    // Temporal filter: Require multiple frames to confirm the target is stable
                    if (detection_counter_ >= 30) {
                        state_ = STATE_LOCKED;
                        RCLCPP_INFO(this->get_logger(), 
                            "Coordinate Locked! Target at Lat: %.6f, Lon: %.6f, Alt: %.2fm | Triggering Action.",
                            final_target_.lat, 
                            final_target_.lon, 
                            final_target_.alt);
                        performAction(final_target_);
                    }
                }
                break;

            case STATE_FAILED:
                // Recover from a rejected or aborted planner action
                RCLCPP_ERROR(this->get_logger(), "Airdrop failed. Returning to search.");
                detection_counter_ = 0;
                state_ = STATE_SEARCHING;
                break;
                
            default:
                break;
        }
    }

    // -------------- Action Client -----------------------
    void performAction(const GPSCoordinate& target)
    {
        // Ensure the planner node is online before dispatching
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Planner Action Server not available!");
            state_ = STATE_FAILED;
            return;
        }

        // Pack the Goal payload using the custom TargetEstimate interface
        auto goal_msg = TargetAirdrop::Goal();
        interfaces::msg::TargetEstimate estimate_msg;
        
        estimate_msg.target_id = 0; // Configured for primary Drop Zone (DZ-1)
        estimate_msg.latitude = target.lat;
        estimate_msg.longitude = target.lon;
        estimate_msg.altitude_amsl = target.alt; 
        estimate_msg.num_observations = detection_counter_;        
        estimate_msg.covariance = {0.0, 0.0, 0.0, 0.0}; // Explicit zero initialization
        
        goal_msg.gps_estimation = estimate_msg;
        goal_msg.bay_index = active_dz_;

        // Bind callbacks to monitor the asynchronous execution
        auto send_goal_options = rclcpp_action::Client<TargetAirdrop>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TargetDetectionNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&TargetDetectionNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TargetDetectionNode::result_callback, this, _1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
        
        state_ = STATE_WAITING;
    }

    // Triggered when the server accepts or rejects the initial coordinate
    void goal_response_callback(const GoalHandleTargetAirdrop::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by planner.");
            state_ = STATE_FAILED;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted. Monitoring drop execution...");
        }
    }

    // Triggered continuously while the planner aligns the Dubins path
    void feedback_callback(
        GoalHandleTargetAirdrop::SharedPtr,
        const std::shared_ptr<const TargetAirdrop::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Planner State ID: %d | Distance to Release: %.2f m | Wind: %.2f m/s", 
            feedback->current_status.state,
            feedback->current_status.distance_to_release,
            feedback->current_status.wind_speed);
    }

    // Triggered upon final success, abortion, or cancellation of the payload drop
    void result_callback(const GoalHandleTargetAirdrop::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Airdrop Complete! Success: %d", result.result->drop_successful);
                
                if (result.result->drop_successful) ++active_dz_;
                // Auto-reset the mission node for subsequent payload deployments
                state_ = STATE_SEARCHING; 
                detection_counter_ = 0;
                break;
            case rclcpp_action::ResultCode::ABORTED:
            case rclcpp_action::ResultCode::CANCELED:
                state_ = STATE_FAILED;
                break;
            default:
                break;
        }
    }
};

} // namespace target_recognition_cpp

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<target_recognition_cpp::TargetDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
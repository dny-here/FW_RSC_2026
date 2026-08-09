// targetdetection.hpp
// Deklarasi awal dari variabel-variabel yang digunakan
// di targetdetection.cpp

#ifndef TARGET_RECOGNITION__TARGET_DETECTION_HPP_
#define TARGET_RECOGNITION__TARGET_DETECTION_HPP_

// OpenCV Dependencies
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

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

    // Payload has been dropped and a cooldown is added to add another waypoint
    STATE_COOLDOWN,

    // Both payload has been dropped
    STATE_DONE,
    
    // Planner aborted or target lost during gathering. Resetting to try again.
    STATE_FAILED 
};

// Single-scale retinex of an image
cv::Mat ssr(const cv::Mat& img, double sigma) {
    cv::Mat blur, log_img, log_blur, res;
    
    // cv::GaussianBlur computes ksize automatically when cv::Size(0,0) is passed
    cv::GaussianBlur(img, blur, cv::Size(0, 0), sigma);
    
    // F(x, y) = GaussianBlur(img) + 1.0
    cv::add(blur, cv::Scalar::all(1.0), blur);
    
    // cv::log computes natural log (ln). To get log10, divide by ln(10)
    cv::log(img, log_img);
    log_img /= std::log(10.0);
    
    cv::log(blur, log_blur);
    log_blur /= std::log(10.0);
    
    cv::subtract(log_img, log_blur, res);
    return res;
}

// Multi-scale retinex of an image
cv::Mat msr(const cv::Mat& img, const std::vector<double>& sigma_scales) {
    cv::Mat msr_sum = cv::Mat::zeros(img.size(), CV_32FC1);
    
    for (double sigma : sigma_scales) {
        cv::Mat s = ssr(img, sigma);
        cv::add(msr_sum, s, msr_sum);
    }
    
    msr_sum /= static_cast<float>(sigma_scales.size());
    
    cv::Mat msr_out;
    cv::normalize(msr_sum, msr_out, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    
    return msr_out;
}

// Contrast stretch img by histogram equalization with black and white cap
cv::Mat color_balance(const cv::Mat& img, float low_per, float high_per) {
    int tot_pix = img.total();
    float low_count = tot_pix * low_per / 100.0f;
    float high_count = tot_pix * (100.0f - high_per) / 100.0f;

    std::vector<cv::Mat> ch_list;
    if (img.channels() == 1) {
        ch_list.push_back(img);
    } else {
        cv::split(img, ch_list);
    }

    std::vector<cv::Mat> cs_img;
    for (size_t c = 0; c < ch_list.size(); ++c) {
        cv::Mat ch = ch_list[c];
        
        int histSize = 256;
        float range[] = { 0, 256 };
        const float* histRange = { range };
        cv::Mat hist;
        cv::calcHist(&ch, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

        std::vector<float> cum_hist(256, 0);
        cum_hist[0] = hist.at<float>(0);
        for (int i = 1; i < 256; ++i) {
            cum_hist[i] = cum_hist[i - 1] + hist.at<float>(i);
        }

        // Find indices for blacking and whiting out pixels
        auto li_it = std::lower_bound(cum_hist.begin(), cum_hist.end(), low_count);
        auto hi_it = std::lower_bound(cum_hist.begin(), cum_hist.end(), high_count);
        
        int li = std::distance(cum_hist.begin(), li_it);
        int hi = std::distance(cum_hist.begin(), hi_it);
        
        li = std::max(0, std::min(li, 255));
        hi = std::max(0, std::min(hi, 255));

        if (li == hi) {
            cs_img.push_back(ch);
            continue;
        }

        cv::Mat lut(1, 256, CV_8UC1);
        for (int i = 0; i < 256; ++i) {
            if (i < li) {
                lut.at<uchar>(i) = 0;
            } else if (i > hi) {
                lut.at<uchar>(i) = 255;
            } else {
                lut.at<uchar>(i) = cv::saturate_cast<uchar>(std::round(static_cast<float>(i - li) / (hi - li) * 255.0f));
            }
        }

        cv::Mat cs_ch;
        cv::LUT(ch, lut, cs_ch);
        cs_img.push_back(cs_ch);
    }

    cv::Mat out;
    if (cs_img.size() == 1) {
        out = cs_img[0];
    } else {
        cv::merge(cs_img, out);
    }
    return out;
}

// Multi-scale retinex with Color Preservation
cv::Mat msrcp(const cv::Mat& img, const std::vector<double>& sigma_scales = {15.0, 80.0, 250.0}, float low_per = 1.0f, float high_per = 1.0f) {
    cv::Mat img_f;
    img.convertTo(img_f, CV_32FC3);

    std::vector<cv::Mat> bgr;
    cv::split(img_f, bgr);

    // int_img = (sum(img, axis=2) / 3) + 1.0
    cv::Mat int_img = (bgr[0] + bgr[1] + bgr[2]) / 3.0f + 1.0f;

    // msr_int = msr(int_img, sigma_scales)
    cv::Mat msr_int = msr(int_img, sigma_scales); 

    // msr_cb = color_balance(msr_int, low_per, high_per)
    cv::Mat msr_cb = color_balance(msr_int, low_per, high_per);
    
    cv::Mat msr_cb_f;
    msr_cb.convertTo(msr_cb_f, CV_32FC1);

    // max_img = max(Ic(x,y)) + 1.0
    cv::Mat max_img = cv::max(cv::max(bgr[0], bgr[1]), bgr[2]) + 1.0f;

    // B = 256.0 / max(Ic)
    cv::Mat B = 256.0f / max_img;

    // A = min(B, MSR_Int / Int)
    cv::Mat A = cv::min(B, msr_cb_f / int_img);

    // MSRCP = A * I
    cv::Mat out_b = A.mul(bgr[0]);
    cv::Mat out_g = A.mul(bgr[1]);
    cv::Mat out_r = A.mul(bgr[2]);

    std::vector<cv::Mat> out_bgr = {out_b, out_g, out_r};
    cv::Mat out;
    cv::merge(out_bgr, out);

    // Convert back to 8-bit. convertTo handles clipping [0, 255] automatically.
    cv::Mat final_out;
    out.convertTo(final_out, CV_8UC3, 1.0, 0.0);
    
    return final_out;
}


} // namespace target_recognition_cpp

#endif
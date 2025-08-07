#ifndef LIDAR_TAPE_CORRECTION_INTENSITY_DISTANCE_CORRECTOR_HPP_
#define LIDAR_TAPE_CORRECTION_INTENSITY_DISTANCE_CORRECTOR_HPP_

#include <vector>
#include <map>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace lidar_tape_correction
{

struct CalibrationPoint
{
    float intensity;
    float measured_distance;
    float actual_distance;
    float error;
};

class IntensityDistanceCorrector
{
public:
    IntensityDistanceCorrector();
    
    void addCalibrationPoint(float intensity, float measured_distance, float actual_distance);
    
    void fitCorrectionModel();
    
    float correctDistance(float measured_distance, float intensity) const;
    
    bool saveCalibration(const std::string& filename) const;
    
    bool loadCalibration(const std::string& filename);
    
    void clearCalibrationData();
    
    bool isCalibrated() const { return calibrated_; }
    
    std::vector<CalibrationPoint> getCalibrationData() const { return calibration_data_; }

private:
    std::vector<CalibrationPoint> calibration_data_;
    
    std::vector<float> polynomial_coeffs_;
    
    bool calibrated_;
    
    float intensity_threshold_;
    
    void fitPolynomial(int degree = 2);
    
    float evaluatePolynomial(float x) const;
};

}

#endif
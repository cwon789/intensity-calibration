#include "lidar_tape_correction/intensity_distance_corrector.hpp"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <sstream>

namespace lidar_tape_correction
{

IntensityDistanceCorrector::IntensityDistanceCorrector()
    : calibrated_(false), intensity_threshold_(200.0)
{
    polynomial_coeffs_.resize(3, 0.0);
}

void IntensityDistanceCorrector::addCalibrationPoint(float intensity, float measured_distance, float actual_distance)
{
    CalibrationPoint point;
    point.intensity = intensity;
    point.measured_distance = measured_distance;
    point.actual_distance = actual_distance;
    point.error = actual_distance - measured_distance;
    
    calibration_data_.push_back(point);
}

void IntensityDistanceCorrector::fitCorrectionModel()
{
    if (calibration_data_.size() < 3)
    {
        std::cerr << "Need at least 3 calibration points to fit model" << std::endl;
        return;
    }
    
    fitPolynomial(2);
    calibrated_ = true;
}

void IntensityDistanceCorrector::fitPolynomial(int degree)
{
    int n = calibration_data_.size();
    int m = degree + 1;
    
    std::vector<std::vector<double>> A(m, std::vector<double>(m, 0.0));
    std::vector<double> b(m, 0.0);
    
    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            double sum = 0.0;
            for (const auto& point : calibration_data_)
            {
                if (point.intensity >= intensity_threshold_)
                {
                    sum += std::pow(point.intensity, i + j);
                }
            }
            A[i][j] = sum;
        }
        
        double sum_b = 0.0;
        for (const auto& point : calibration_data_)
        {
            if (point.intensity >= intensity_threshold_)
            {
                sum_b += point.error * std::pow(point.intensity, i);
            }
        }
        b[i] = sum_b;
    }
    
    for (int i = 0; i < m; ++i)
    {
        int max_row = i;
        for (int k = i + 1; k < m; ++k)
        {
            if (std::abs(A[k][i]) > std::abs(A[max_row][i]))
            {
                max_row = k;
            }
        }
        
        std::swap(A[i], A[max_row]);
        std::swap(b[i], b[max_row]);
        
        for (int k = i + 1; k < m; ++k)
        {
            double factor = A[k][i] / A[i][i];
            for (int j = i; j < m; ++j)
            {
                A[k][j] -= factor * A[i][j];
            }
            b[k] -= factor * b[i];
        }
    }
    
    polynomial_coeffs_.resize(m);
    for (int i = m - 1; i >= 0; --i)
    {
        polynomial_coeffs_[i] = b[i];
        for (int j = i + 1; j < m; ++j)
        {
            polynomial_coeffs_[i] -= A[i][j] * polynomial_coeffs_[j];
        }
        polynomial_coeffs_[i] /= A[i][i];
    }
}

float IntensityDistanceCorrector::evaluatePolynomial(float x) const
{
    float result = 0.0;
    for (size_t i = 0; i < polynomial_coeffs_.size(); ++i)
    {
        result += polynomial_coeffs_[i] * std::pow(x, i);
    }
    return result;
}

float IntensityDistanceCorrector::correctDistance(float measured_distance, float intensity) const
{
    if (!calibrated_)
    {
        return measured_distance;
    }
    
    if (intensity < intensity_threshold_)
    {
        return measured_distance;
    }
    
    float correction = evaluatePolynomial(intensity);
    
    correction = std::max(-0.5f, std::min(0.5f, correction));
    
    return measured_distance + correction;
}

bool IntensityDistanceCorrector::saveCalibration(const std::string& filename) const
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        return false;
    }
    
    file << "# Intensity Distance Correction Calibration Data\n";
    file << "# Format: intensity_threshold polynomial_coefficients calibration_points\n";
    
    file << intensity_threshold_ << "\n";
    
    file << polynomial_coeffs_.size() << "\n";
    for (float coeff : polynomial_coeffs_)
    {
        file << coeff << " ";
    }
    file << "\n";
    
    file << calibration_data_.size() << "\n";
    for (const auto& point : calibration_data_)
    {
        file << point.intensity << " " 
             << point.measured_distance << " " 
             << point.actual_distance << " "
             << point.error << "\n";
    }
    
    file.close();
    return true;
}

bool IntensityDistanceCorrector::loadCalibration(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        return false;
    }
    
    std::string line;
    
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        
        std::istringstream iss(line);
        iss >> intensity_threshold_;
        break;
    }
    
    size_t num_coeffs;
    if (std::getline(file, line))
    {
        std::istringstream iss(line);
        iss >> num_coeffs;
    }
    
    polynomial_coeffs_.clear();
    if (std::getline(file, line))
    {
        std::istringstream iss(line);
        for (size_t i = 0; i < num_coeffs; ++i)
        {
            float coeff;
            iss >> coeff;
            polynomial_coeffs_.push_back(coeff);
        }
    }
    
    size_t num_points;
    if (std::getline(file, line))
    {
        std::istringstream iss(line);
        iss >> num_points;
    }
    
    calibration_data_.clear();
    for (size_t i = 0; i < num_points; ++i)
    {
        if (std::getline(file, line))
        {
            CalibrationPoint point;
            std::istringstream iss(line);
            iss >> point.intensity >> point.measured_distance 
                >> point.actual_distance >> point.error;
            calibration_data_.push_back(point);
        }
    }
    
    file.close();
    calibrated_ = true;
    return true;
}

void IntensityDistanceCorrector::clearCalibrationData()
{
    calibration_data_.clear();
    polynomial_coeffs_.clear();
    polynomial_coeffs_.resize(3, 0.0);
    calibrated_ = false;
}

}
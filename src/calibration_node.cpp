#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "lidar_tape_correction/intensity_distance_corrector.hpp"
#include <vector>
#include <algorithm>
#include <numeric>

class CalibrationNode : public rclcpp::Node
{
public:
    CalibrationNode() : Node("lidar_calibration_node")
    {
        this->declare_parameter("scan_topic", "/bot_sensor/lidar_front/laser_scan");
        this->declare_parameter("calibration_file", "lidar_calibration.txt");
        this->declare_parameter("intensity_threshold", 200.0);
        this->declare_parameter("collection_window_size", 10);
        this->declare_parameter("high_intensity_angle_range", 5.0);
        
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
        collection_window_size_ = this->get_parameter("collection_window_size").as_int();
        angle_range_ = this->get_parameter("high_intensity_angle_range").as_double() * M_PI / 180.0;
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&CalibrationNode::scanCallback, this, std::placeholders::_1));
        
        actual_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "actual_distance", 10,
            std::bind(&CalibrationNode::actualDistanceCallback, this, std::placeholders::_1));
        
        high_intensity_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "high_intensity_point", 10);
        
        collect_service_ = this->create_service<std_srvs::srv::Trigger>(
            "collect_calibration_point",
            std::bind(&CalibrationNode::collectCalibrationPoint, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        fit_service_ = this->create_service<std_srvs::srv::Trigger>(
            "fit_calibration_model",
            std::bind(&CalibrationNode::fitCalibrationModel, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_calibration",
            std::bind(&CalibrationNode::saveCalibration, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        clear_service_ = this->create_service<std_srvs::srv::Trigger>(
            "clear_calibration",
            std::bind(&CalibrationNode::clearCalibration, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Calibration node initialized");
        RCLCPP_INFO(this->get_logger(), "Instructions:");
        RCLCPP_INFO(this->get_logger(), "1. Place high-reflectance tape at known distance");
        RCLCPP_INFO(this->get_logger(), "2. Publish actual distance to /actual_distance topic");
        RCLCPP_INFO(this->get_logger(), "3. Call /collect_calibration_point service");
        RCLCPP_INFO(this->get_logger(), "4. Repeat for multiple distances");
        RCLCPP_INFO(this->get_logger(), "5. Call /fit_calibration_model service");
        RCLCPP_INFO(this->get_logger(), "6. Call /save_calibration service");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = msg;
        
        std::vector<std::pair<float, float>> high_intensity_points;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->intensities[i] >= intensity_threshold_ && 
                !std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i]))
            {
                float angle = msg->angle_min + i * msg->angle_increment;
                high_intensity_points.push_back({msg->ranges[i], msg->intensities[i]});
                
                if (std::abs(angle) < angle_range_)
                {
                    recent_high_intensity_distances_.push_back(msg->ranges[i]);
                    recent_high_intensity_intensities_.push_back(msg->intensities[i]);
                    
                    if (recent_high_intensity_distances_.size() > collection_window_size_)
                    {
                        recent_high_intensity_distances_.erase(recent_high_intensity_distances_.begin());
                        recent_high_intensity_intensities_.erase(recent_high_intensity_intensities_.begin());
                    }
                }
            }
        }
        
        if (!high_intensity_points.empty())
        {
            auto min_it = std::min_element(high_intensity_points.begin(), high_intensity_points.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
            
            geometry_msgs::msg::PointStamped point_msg;
            point_msg.header = msg->header;
            point_msg.point.x = min_it->first;
            point_msg.point.y = 0.0;
            point_msg.point.z = min_it->second;
            high_intensity_point_pub_->publish(point_msg);
        }
    }
    
    void actualDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        actual_distance_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Actual distance set to: %.3f m", actual_distance_);
    }
    
    void collectCalibrationPoint(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (recent_high_intensity_distances_.empty())
        {
            response->success = false;
            response->message = "No high intensity points detected";
            return;
        }
        
        if (actual_distance_ <= 0)
        {
            response->success = false;
            response->message = "Please set actual distance first via /actual_distance topic";
            return;
        }
        
        float avg_distance = std::accumulate(recent_high_intensity_distances_.begin(),
                                            recent_high_intensity_distances_.end(), 0.0f) /
                           recent_high_intensity_distances_.size();
        
        float avg_intensity = std::accumulate(recent_high_intensity_intensities_.begin(),
                                             recent_high_intensity_intensities_.end(), 0.0f) /
                            recent_high_intensity_intensities_.size();
        
        corrector_.addCalibrationPoint(avg_intensity, avg_distance, actual_distance_);
        
        response->success = true;
        response->message = "Calibration point added: intensity=" + std::to_string(avg_intensity) +
                          ", measured=" + std::to_string(avg_distance) +
                          ", actual=" + std::to_string(actual_distance_) +
                          ", error=" + std::to_string(actual_distance_ - avg_distance) +
                          " | Total points: " + std::to_string(corrector_.getCalibrationData().size());
        
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    void fitCalibrationModel(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (corrector_.getCalibrationData().size() < 3)
        {
            response->success = false;
            response->message = "Need at least 3 calibration points. Current: " + 
                              std::to_string(corrector_.getCalibrationData().size());
            return;
        }
        
        corrector_.fitCorrectionModel();
        
        response->success = true;
        response->message = "Calibration model fitted with " + 
                          std::to_string(corrector_.getCalibrationData().size()) + " points";
        
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        
        for (const auto& point : corrector_.getCalibrationData())
        {
            float corrected = corrector_.correctDistance(point.measured_distance, point.intensity);
            RCLCPP_INFO(this->get_logger(), 
                       "Intensity: %.1f, Measured: %.3f, Actual: %.3f, Corrected: %.3f, Remaining error: %.3f",
                       point.intensity, point.measured_distance, point.actual_distance, 
                       corrected, point.actual_distance - corrected);
        }
    }
    
    void saveCalibration(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!corrector_.isCalibrated())
        {
            response->success = false;
            response->message = "Model not fitted yet. Call /fit_calibration_model first";
            return;
        }
        
        if (corrector_.saveCalibration(calibration_file_))
        {
            response->success = true;
            response->message = "Calibration saved to " + calibration_file_;
        }
        else
        {
            response->success = false;
            response->message = "Failed to save calibration to " + calibration_file_;
        }
        
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    void clearCalibration(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        corrector_.clearCalibrationData();
        recent_high_intensity_distances_.clear();
        recent_high_intensity_intensities_.clear();
        actual_distance_ = 0.0;
        
        response->success = true;
        response->message = "Calibration data cleared";
        
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actual_distance_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr high_intensity_point_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr collect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fit_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
    
    lidar_tape_correction::IntensityDistanceCorrector corrector_;
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    
    std::vector<float> recent_high_intensity_distances_;
    std::vector<float> recent_high_intensity_intensities_;
    
    float actual_distance_ = 0.0;
    float intensity_threshold_;
    float angle_range_;
    int collection_window_size_;
    
    std::string scan_topic_;
    std::string calibration_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
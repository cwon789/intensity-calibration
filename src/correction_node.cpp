#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "lidar_tape_correction/intensity_distance_corrector.hpp"

class CorrectionNode : public rclcpp::Node
{
public:
    CorrectionNode() : Node("lidar_correction_node")
    {
        this->declare_parameter("scan_topic", "/bot_sensor/lidar_front/laser_scan");
        this->declare_parameter("corrected_scan_topic", "/corrected_laser_scan");
        this->declare_parameter("calibration_file", "lidar_calibration.txt");
        this->declare_parameter("publish_markers", true);
        this->declare_parameter("intensity_threshold", 200.0);
        
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        corrected_scan_topic_ = this->get_parameter("corrected_scan_topic").as_string();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&CorrectionNode::scanCallback, this, std::placeholders::_1));
        
        corrected_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            corrected_scan_topic_, 10);
        
        if (publish_markers_)
        {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "correction_markers", 10);
        }
        
        if (corrector_.loadCalibration(calibration_file_))
        {
            RCLCPP_INFO(this->get_logger(), "Calibration loaded from %s", calibration_file_.c_str());
            RCLCPP_INFO(this->get_logger(), "Correction node is active");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration from %s", calibration_file_.c_str());
            RCLCPP_WARN(this->get_logger(), "Running in pass-through mode");
        }
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto corrected_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (!std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i]))
            {
                float original_range = msg->ranges[i];
                float intensity = msg->intensities[i];
                
                corrected_msg->ranges[i] = corrector_.correctDistance(original_range, intensity);
                
                if (publish_markers_ && intensity >= intensity_threshold_)
                {
                    float angle = msg->angle_min + i * msg->angle_increment;
                    
                    visualization_msgs::msg::Marker original_marker;
                    original_marker.header = msg->header;
                    original_marker.ns = "original";
                    original_marker.id = marker_id;
                    original_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    original_marker.action = visualization_msgs::msg::Marker::ADD;
                    original_marker.pose.position.x = original_range * cos(angle);
                    original_marker.pose.position.y = original_range * sin(angle);
                    original_marker.pose.position.z = 0.0;
                    original_marker.pose.orientation.w = 1.0;
                    original_marker.scale.x = 0.05;
                    original_marker.scale.y = 0.05;
                    original_marker.scale.z = 0.05;
                    original_marker.color.r = 1.0;
                    original_marker.color.g = 0.0;
                    original_marker.color.b = 0.0;
                    original_marker.color.a = 0.5;
                    original_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
                    marker_array.markers.push_back(original_marker);
                    
                    visualization_msgs::msg::Marker corrected_marker = original_marker;
                    corrected_marker.ns = "corrected";
                    corrected_marker.id = marker_id + 1000;
                    corrected_marker.pose.position.x = corrected_msg->ranges[i] * cos(angle);
                    corrected_marker.pose.position.y = corrected_msg->ranges[i] * sin(angle);
                    corrected_marker.color.r = 0.0;
                    corrected_marker.color.g = 1.0;
                    corrected_marker.color.b = 0.0;
                    marker_array.markers.push_back(corrected_marker);
                    
                    visualization_msgs::msg::Marker line_marker;
                    line_marker.header = msg->header;
                    line_marker.ns = "correction_lines";
                    line_marker.id = marker_id + 2000;
                    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    line_marker.action = visualization_msgs::msg::Marker::ADD;
                    line_marker.scale.x = 0.01;
                    line_marker.color.r = 0.0;
                    line_marker.color.g = 0.0;
                    line_marker.color.b = 1.0;
                    line_marker.color.a = 0.5;
                    line_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
                    
                    geometry_msgs::msg::Point p1, p2;
                    p1.x = original_range * cos(angle);
                    p1.y = original_range * sin(angle);
                    p1.z = 0.0;
                    p2.x = corrected_msg->ranges[i] * cos(angle);
                    p2.y = corrected_msg->ranges[i] * sin(angle);
                    p2.z = 0.0;
                    
                    line_marker.points.push_back(p1);
                    line_marker.points.push_back(p2);
                    marker_array.markers.push_back(line_marker);
                    
                    marker_id++;
                }
            }
        }
        
        corrected_scan_pub_->publish(*corrected_msg);
        
        if (publish_markers_ && !marker_array.markers.empty())
        {
            marker_pub_->publish(marker_array);
        }
        
        static int count = 0;
        if (++count % 100 == 0)
        {
            int high_intensity_count = 0;
            float total_correction = 0.0;
            
            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                if (msg->intensities[i] >= intensity_threshold_ && 
                    !std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i]))
                {
                    high_intensity_count++;
                    total_correction += corrected_msg->ranges[i] - msg->ranges[i];
                }
            }
            
            if (high_intensity_count > 0)
            {
                RCLCPP_INFO(this->get_logger(), 
                           "Processing scan: %d high-intensity points, avg correction: %.3f m",
                           high_intensity_count, total_correction / high_intensity_count);
            }
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr corrected_scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    lidar_tape_correction::IntensityDistanceCorrector corrector_;
    
    std::string scan_topic_;
    std::string corrected_scan_topic_;
    std::string calibration_file_;
    bool publish_markers_;
    float intensity_threshold_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorrectionNode>());
    rclcpp::shutdown();
    return 0;
}
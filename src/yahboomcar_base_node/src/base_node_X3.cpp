#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
//#include "turtlesim/turtlesim/msg/pose.hpp"


#include <memory>
#include <string>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class OdomPublisher:public rclcpp ::Node
{
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   rclcpp::TimerBase::SharedPtr keepalive_timer_;
   double linear_scale_x_ = 0.0 ;
   double linear_scale_y_ = 0.0;
   double angular_scale_ = 0.0;
   double vel_dt_ = 0.0;
   double x_pos_ = 0.0;
   double y_pos_ = 0.0;
   double heading_ = 0.0;
   double linear_velocity_x_ = 0.0;
   double linear_velocity_y_ = 0.0;
   double angular_velocity_z_ = 0.0;
   bool pub_odom_tf_ = false;
   rclcpp::Time last_vel_time_  ;
   std::string odom_frame = "odom";
   std::string base_footprint_frame = "base_footprint";
	public:
	  OdomPublisher()
	  : Node("base_node")
	  {            
        // Initialize time tracking
        last_vel_time_ = this->get_clock()->now();
            
            this->declare_parameter<std::string>("odom_frame","odom");
            this->declare_parameter<std::string>("base_footprint_frame","base_footprint"); 
            this->declare_parameter<double>("linear_scale_x",1.0);     // No scaling - calibration applied in hardware driver
            this->declare_parameter<double>("linear_scale_y",1.0);      // No scaling - calibration applied in hardware driver  
            this->declare_parameter<double>("angular_scale",1.0);       // No scaling - calibration applied in hardware driver
            this->declare_parameter<bool>("pub_odom_tf",false);

            this->get_parameter<double>("linear_scale_x",linear_scale_x_);
            this->get_parameter<double>("linear_scale_y",linear_scale_y_);
            this->get_parameter<double>("angular_scale",angular_scale_);
            this->get_parameter<bool>("pub_odom_tf",pub_odom_tf_);
            this->get_parameter<std::string>("odom_frame",odom_frame);
            this->get_parameter<std::string>("base_footprint_frame",base_footprint_frame);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize last_vel_time_ to prevent huge vel_dt_ on first call
        last_vel_time_ = this->get_clock()->now();

		subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("vel_raw",50,std::bind(&OdomPublisher::handle_vel,this,_1));
		odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 50);

		// Publish initial odometry immediately
		this->publish_odometry();
		
		// Publish odometry at 20Hz to ensure it's always available
		keepalive_timer_ = this->create_wall_timer(
			std::chrono::milliseconds(50), // 20Hz
			std::bind(&OdomPublisher::publish_odometry, this)
		);

        }
	  	private:
	  	  void publish_odometry()
	  	  {
	  	      rclcpp::Time current_time = this->get_clock()->now();
	  	      
	  	      tf2::Quaternion myQuaternion;
	  	      geometry_msgs::msg::Quaternion odom_quat;
	  	      myQuaternion.setRPY(0.00, 0.00, heading_);
	  	      
	  	      odom_quat.x = myQuaternion.x();
	  	      odom_quat.y = myQuaternion.y();
	  	      odom_quat.z = myQuaternion.z();
	  	      odom_quat.w = myQuaternion.w();
	  	      
	  	      nav_msgs::msg::Odometry odom;
	  	      odom.header.stamp = current_time;
	  	      odom.header.frame_id = odom_frame;
	  	      odom.child_frame_id = base_footprint_frame;
	  	      // Position from integration of actual hardware velocities
	  	      odom.pose.pose.position.x = x_pos_;
	  	      odom.pose.pose.position.y = y_pos_;
	  	      odom.pose.pose.position.z = 0.0;
	  	      
	  	      // Orientation from integrated heading
	  	      tf2::Quaternion q;
	  	      q.setRPY(0.0, 0.0, heading_);
	  	      odom.pose.pose.orientation.x = q.x();
	  	      odom.pose.pose.orientation.y = q.y();
	  	      odom.pose.pose.orientation.z = q.z();
	  	      odom.pose.pose.orientation.w = q.w();
	  	      odom.pose.covariance[0] = 0.001;
	  	      odom.pose.covariance[7] = 0.001;
	  	      odom.pose.covariance[35] = 0.001;
	  	      odom.twist.twist.linear.x = linear_velocity_x_;
	  	      odom.twist.twist.linear.y = linear_velocity_y_;
	  	      odom.twist.twist.linear.z = 0.0;
	  	      odom.twist.twist.angular.x = 0.0;
	  	      odom.twist.twist.angular.y = 0.0;
	  	      odom.twist.twist.angular.z = angular_velocity_z_;
	  	      odom.twist.covariance[0] = 0.0001;
	  	      odom.twist.covariance[7] = 0.0001;
	  	      odom.twist.covariance[35] = 0.0001;

	  	      odom_publisher_->publish(odom);

	  	      // Publish TF transform from odom to base_footprint if enabled
	  	      if (pub_odom_tf_) {
	  	          geometry_msgs::msg::TransformStamped odom_trans;
	  	          odom_trans.header.stamp = current_time;
	  	          odom_trans.header.frame_id = odom_frame;
	  	          odom_trans.child_frame_id = base_footprint_frame;

	  	          odom_trans.transform.translation.x = x_pos_;
	  	          odom_trans.transform.translation.y = y_pos_;
	  	          odom_trans.transform.translation.z = 0.0;
	  	          odom_trans.transform.rotation = odom_quat;

	  	          tf_broadcaster_->sendTransform(odom_trans);
	  	      }
	  	  }
	  	  
	  	  void handle_vel(const std::shared_ptr<geometry_msgs::msg::Twist > msg)
	  	  {
	  	  	rclcpp::Time current_time = this->get_clock()->now();

	  	  	// Store the actual hardware velocities (from vel_raw)
	  	  	// Negate angular velocity to match ROS coordinate convention
	  	  	linear_velocity_x_ = msg->linear.x;
    		linear_velocity_y_ = msg->linear.y;
    		angular_velocity_z_ = -msg->angular.z;

    		// Calculate time delta for position integration
			vel_dt_ = (current_time - last_vel_time_).seconds();
    		last_vel_time_ = current_time;

    		// Only integrate if we have a reasonable time delta and non-zero velocities
    		if (vel_dt_ > 0.0 && vel_dt_ < 1.0) // Sanity check: between 0 and 1 second
    		{
    			// Calculate position changes from ACTUAL hardware velocities
    			double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_ * linear_scale_x_;
    			double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_ * linear_scale_y_;
				x_pos_ += delta_x;
    			y_pos_ += delta_y;

				// Update heading from actual angular velocity
    			double delta_heading = angular_velocity_z_ * vel_dt_ * angular_scale_;
				heading_ += delta_heading;
			}
	  	  }
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomPublisher>());
	rclcpp::shutdown();
    return 0;
}


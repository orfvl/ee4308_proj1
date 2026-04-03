#include "ee4308_drone/controller.hpp"

namespace ee4308::drone
{
    Controller::Controller(
        const rclcpp::NodeOptions &options,
        const std::string &name = "controller") 
        : Node(name, options)
    {
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 20.0).as_double();
        this->enable_ = ee4308::getParameter<bool>(this, "enable", true).as_bool();
        this->lookahead_distance_ = ee4308::getParameter<double>(this, "lookahead_distance", 1.0).as_double();
        this->max_xy_vel_ = ee4308::getParameter<double>(this, "max_xy_vel", 1.0).as_double();
        this->max_z_vel_ = ee4308::getParameter<double>(this, "max_z_vel", 0.5).as_double();
        this->yaw_vel_ = ee4308::getParameter<double>(this, "yaw_vel", 0.3).as_double();
        this->kp_xy_ = ee4308::getParameter<double>(this, "kp_xy", 1.0).as_double();
        this->kp_z_ = ee4308::getParameter<double>(this, "kp_z", 1.0).as_double();
        

        this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubPlan_, this, std::placeholders::_1));

        this->pub_lookahead_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "lookahead_marker", rclcpp::SensorDataQoS());

        this->sub_true_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "true_odom", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry msg) { this->true_odom_ = msg; });

        this->received_odom_ = false;

        this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Controller::callbackTimer_, this));
    }

    void Controller::callbackSubOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->odom_ = msg;
        this->received_odom_ = true;
    }

    void Controller::callbackSubPlan_(const nav_msgs::msg::Path msg)
    {
        this->plan_ = msg;
    }

    void Controller::callbackTimer_()
    {
        if (!enable_)
            return;

        if (!this->received_odom_)
        {
            publishCmdVel_(0, 0, 0, 0);
            return;
        }

        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel_(0, 0, 0, 0);
            return;
        }

        // ==== make use of ====
        // plan_.poses
        // odom_
        // ee4308::getYawFromQuaternion()
        // std::hypot()
        // std::clamp()
        // std::cos(), std::sin() 
        // lookahead_distance_
        // kp_xy_
        // kp_z_
        // max_xy_vel_
        // max_z_vel_
        // yaw_vel_
        // publishCmdVel__()
        // =========

        //  Find the closest point along the path.
        size_t closest_idx = 0;
        double closest_dist = std::hypot(plan_.poses[0].pose.position.x - odom_.pose.pose.position.x,
                                        plan_.poses[0].pose.position.y - odom_.pose.pose.position.y,
                                        plan_.poses[0].pose.position.z - odom_.pose.pose.position.z);
        for (size_t i = 1; i < plan_.poses.size(); i++)
        {
            double dist = std::hypot(plan_.poses[i].pose.position.x - odom_.pose.pose.position.x,
                                    plan_.poses[i].pose.position.y - odom_.pose.pose.position.y,
                                    plan_.poses[i].pose.position.z - odom_.pose.pose.position.z);
            if (dist < closest_dist)            {
                closest_dist = dist;
                closest_idx = i;
            }
        }
        
        //  Find the lookahead point along the path that is at least lookahead_distance_ away from the closest point.
        //  From the lookahead point by searching from the closest point.
        size_t lookahead_idx = plan_.poses.size() - 1;
        for (size_t i = closest_idx; i < plan_.poses.size(); i++)
        {
            double dist = std::hypot(plan_.poses[i].pose.position.x - odom_.pose.pose.position.x,
                                    plan_.poses[i].pose.position.y - odom_.pose.pose.position.y,
                                    plan_.poses[i].pose.position.z - odom_.pose.pose.position.z);
            if (dist >= lookahead_distance_)
            {
                lookahead_idx = i;
                break;
            }
        }

                //  Determine the x and y velocities in the drone's frame to reach the lookahead point.
        //  PD control: proportional term drives toward target, derivative term damps current velocity.
        double xy_error = std::hypot(plan_.poses[lookahead_idx].pose.position.x - odom_.pose.pose.position.x,
                                     plan_.poses[lookahead_idx].pose.position.y - odom_.pose.pose.position.y);
        double vel_ = kp_xy_ * xy_error;
        vel_ = std::clamp(vel_, 0.0, max_xy_vel_);
 
        double path_yaw = std::atan2(plan_.poses[lookahead_idx].pose.position.y - odom_.pose.pose.position.y,
                                    plan_.poses[lookahead_idx].pose.position.x - odom_.pose.pose.position.x);
        double drone_yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);
        double angle_diff = ee4308::limitAngle(path_yaw - drone_yaw);
 
        // Proportional component in drone frame
        double x_vel_ = vel_ * std::cos(angle_diff);
        double y_vel_ = vel_ * std::sin(angle_diff);
 
        // Derivative damping: subtract a term proportional to the drone's current velocity (in drone frame).
        // odom twist is in the drone's body frame, so linear.x and linear.y can be used directly.
        x_vel_ -= kd_xy_ * odom_.twist.twist.linear.x;
        y_vel_ -= kd_xy_ * odom_.twist.twist.linear.y;
 
        //  Constrain the x and y velocities.
        x_vel_ = std::clamp(x_vel_, -max_xy_vel_, max_xy_vel_);
        y_vel_ = std::clamp(y_vel_, -max_xy_vel_, max_xy_vel_);
 
        //  Determine the z velocity in the drone's frame to reach the lookahead point.
        //  PD control: proportional on altitude error, derivative damps vertical velocity.
        double z_error = plan_.poses[lookahead_idx].pose.position.z - odom_.pose.pose.position.z;
        double z_vel_ = kp_z_ * z_error - kd_z_ * odom_.twist.twist.linear.z;
 
        //  Constrain the z velocity.
        z_vel_ = std::clamp(z_vel_, -max_z_vel_, max_z_vel_);

        //  Move the drone in x , y , and z , and at the required yaw velocity.
        // publish

        // Publish lookahead point marker for Foxglove visualization
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = plan_.poses[lookahead_idx].pose.position.x;
        marker.pose.position.y = plan_.poses[lookahead_idx].pose.position.y;
        marker.pose.position.z = plan_.poses[lookahead_idx].pose.position.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        pub_lookahead_marker_->publish(marker);
        publishCmdVel_(x_vel_, y_vel_, z_vel_, yaw_vel_);
    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel_(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        // RCLCPP_INFO(this->get_logger(), "%f,%f,%f,%f", x_vel, y_vel, z_vel, yaw_vel);
        if (!std::isfinite(x_vel) || !std::isfinite(y_vel) || !std::isfinite(z_vel) || !std::isfinite(yaw_vel))
        {
            RCLCPP_WARN(this->get_logger(), 
                "Cmd velocities are inf or nan. Controller or estimator problem. CmdVels(x,y,z,yaw): %6.3f, %6.3f, %6.3f, %6.3f", 
                x_vel, y_vel, z_vel, yaw_vel);
        }
        pub_cmd_vel_->publish(cmd_vel);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Controller);
#include "ee4308_turtle/controller.hpp"

#include <cmath>
#include <limits>

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_gain", this->yaw_gain_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".curvature_threshold", this->curvature_threshold_, 100.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".proximity_threshold", this->proximity_threshold_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".lookahead_gain", this->lookahead_gain_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".min_lookahead_dist", this->min_lookahead_dist_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_lookahead_dist", this->max_lookahead_dist_, 0.6);

        // initialize topics
        this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->scan_ranges_ = msg->ranges;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // -------------------------------- Check for Global Path -------------------------------
        // If no global path, then return (0,0) -> Zero linear and angular velocities

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // -------------------------------- Check for Goal Proximity -------------------------------
        // If the robot is close to the goal, then return (0,0) -> Goal is the last point on the path

        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        // If the robot is close to the goal Then return Zero velocities
        if (ee4308::getDistance(rbt_pose.pose.position, goal_pose.pose.position) < xy_goal_thres_)
        {
            const double rbt_yaw = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);
            const double goal_yaw = ee4308::getYawFromQuaternion(goal_pose.pose.orientation);
            const double yaw_error = ee4308::limitAngle(goal_yaw - rbt_yaw);
            if (std::abs(yaw_error) > yaw_goal_thres_){
                return writeCmdVel(0, std::clamp(yaw_error * this->yaw_gain_, -max_angular_vel_, max_angular_vel_));
            }

            RCLCPP_INFO_STREAM(node_->get_logger(), "Goal reached!");
            return writeCmdVel(0, 0);
        }

        // -------------------------------- Find Closest Point -------------------------------
        // Find the point along the path that is closest to the robot
        
        size_t closest_point_index = 0;
        double distance_to_closest = ee4308::getDistance(x_r, y_r, 
            global_plan_.poses[closest_point_index].pose.position.x,
            global_plan_.poses[closest_point_index].pose.position.y);
        double contending_distance;

        for (size_t i = 0; i < global_plan_.poses.size(); i++)
        {
            contending_distance = ee4308::getDistance(x_r, y_r, 
            global_plan_.poses[i].pose.position.x,
            global_plan_.poses[i].pose.position.y);

            if (distance_to_closest > contending_distance)
            {
                closest_point_index = i;
                distance_to_closest = contending_distance;
            }
        }

        // -------------------------------- Find Lookahead Point -------------------------------
        // From the closest point, find the lookahead point

        size_t lookahead_point_idx = closest_point_index;
        double dist = 0.0;
        for (size_t i = closest_point_index; i < global_plan_.poses.size(); ++i)
        {
            dist = ee4308::getDistance(rbt_pose.pose.position, global_plan_.poses[i].pose.position);
            if (dist >= desired_lookahead_dist_)
            {
                lookahead_point_idx = i;
                break;
            }
        }
        geometry_msgs::msg::PoseStamped lookahead_pose = global_plan_.poses[lookahead_point_idx];

        // -------------------------------- Transform lookahead point to robot frame (x_p, y_p) -------------------------------
        // Transform the lookahead point into the robot frame to get (x_dash, y_dash)
        double x_delta = lookahead_pose.pose.position.x - rbt_pose.pose.position.x;
        double y_delta = lookahead_pose.pose.position.y - rbt_pose.pose.position.y;

        double theta_rbt = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation); 

        double x_dash = x_delta * std::cos(theta_rbt) + y_delta * std::sin(theta_rbt);
        double y_dash = -x_delta * std::sin(theta_rbt) + y_delta * std::cos(theta_rbt);

        // -------------------------------- Check if lookahead is behind robot -------------------------------
        // If the lookahead point is behind the robot, then rotate in place to face it
        double alpha = std::atan2(y_dash, x_dash);  // bearing to lookahead in robot frame

        if (std::abs(alpha) > M_PI / 2.0) { // > 90 degrees threshold
        double omega = std::clamp(this->yaw_gain_ * alpha, -max_angular_vel_, max_angular_vel_);
        return writeCmdVel(0.0, omega);
        }

        // -------------------------------- Calculate Curvature c -------------------------------
        // Calculate the curvature.
        double d2 = x_dash*x_dash + y_dash*y_dash;
        double curvature = 2.0 * y_dash / d2;

        // -------------------------------- Calculate omega from v and c -------------------------------
        // Calculate ω from v and c.
        
        double linear_vel = this->desired_linear_vel_;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Curvature: " << curvature << ", Dist: " << dist);

        double desired_angular_vel = curvature * linear_vel;

        // -------------------------------- Constrain omega and v -------------------------------
        // Constrain ω to within the largest allowable angular speed.
        desired_angular_vel = std::clamp(desired_angular_vel, -max_angular_vel_, max_angular_vel_);

        // Constrain v to within the largest allowable linear speed.
        linear_vel = std::clamp(linear_vel, -max_linear_vel_, max_linear_vel_);

        // -------------------------------- Curvature heuristic -------------------------------
        // Calculate the curvature heuristic. 
        RCLCPP_INFO_STREAM(node_->get_logger(), "Before curvature adjustment, linear_vel: " << linear_vel);
        linear_vel = ( std::abs(curvature) > this->curvature_threshold_) ? linear_vel * this->curvature_threshold_/curvature : linear_vel; 
        RCLCPP_INFO_STREAM(node_->get_logger(), "After curvature adjustment, linear_vel: " << linear_vel);    
        
        // -------------------------------- Obstacle heuristic -------------------------------
        // Calculate the obstacle heuristic. 

        double obstacle_dist = std::numeric_limits<double>::infinity();
        for (const float r : this->scan_ranges_)
        {
            if (std::isfinite(r) && r > 0.0f)
            {
                obstacle_dist = std::min(obstacle_dist, static_cast<double>(r));
            }
        }

        if (std::isfinite(obstacle_dist))
        {
            linear_vel = (obstacle_dist < this->proximity_threshold_) ? linear_vel * obstacle_dist / this->proximity_threshold_ : linear_vel;
        }
        RCLCPP_INFO_STREAM(node_->get_logger(), "After proximity adjustment, linear_vel: " << linear_vel);
        
        // -------------------------------- Vary lookahead heuristic -------------------------------
        // Vary the lookahead heuristic.
        desired_lookahead_dist_ = std::clamp(this->lookahead_gain_ * std::abs(linear_vel), min_lookahead_dist_, max_lookahead_dist_);

        RCLCPP_INFO_STREAM(node_->get_logger(),
                             "Closest idx: " << closest_point_idx <<
                             ", Lookahead idx: " << lookahead_point_idx <<
                             ", x_delta: " << x_delta <<
                             ", y_delta: " << y_delta <<
                             ", dist: " << dist <<
                             ", curvature: " << curvature <<
                             ", linear_vel: " << linear_vel <<
                             ", desired_angular_vel: " << desired_angular_vel);
        return writeCmdVel(linear_vel, desired_angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "odom";
        cmd_vel.header.stamp = this->node_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    // ======================================== DO NOT TOUCH =================================

    void Controller::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { this->global_plan_ = path; }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)
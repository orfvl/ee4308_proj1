#include "ee4308_turtle/controller.hpp"

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

        // initialize topics
        // this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "scan", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    // void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    // {
    //     this->scan_ranges_ = msg->ranges;
    // }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        // If the robot is close to the goal Then return Zero velocities
        if (ee4308::getDistance(rbt_pose.pose.position, goal_pose.pose.position) < xy_goal_thres_ &&
            std::abs(ee4308::getYawFromQuaternion(rbt_pose.pose.orientation) - ee4308::getYawFromQuaternion(goal_pose.pose.orientation)) < yaw_goal_thres_)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Goal reached!");
            return writeCmdVel(0, 0);
        }

        // Find the point along the path that is closest to the robot.
        // TODO: optimize search by starting from last closest point/ last point
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_point_idx = 0;
        for (size_t i = 0; i < global_plan_.poses.size(); ++i)
        {
            double dist = ee4308::getDistance(rbt_pose.pose.position, global_plan_.poses[i].pose.position);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_point_idx = i;
            }
        }

        // From the closest point, find the lookahead point.
        size_t lookahead_point_idx = closest_point_idx;
        double dist = 0.0;
        for (size_t i = closest_point_idx; i < global_plan_.poses.size(); ++i)
        {
            dist = ee4308::getDistance(rbt_pose.pose.position, global_plan_.poses[i].pose.position);
            if (dist >= desired_lookahead_dist_)
            {
                lookahead_point_idx = i;
                break;
            }
        }
        geometry_msgs::msg::PoseStamped lookahead_pose = global_plan_.poses[lookahead_point_idx];

        // Transform the lookahead point into the robot frame to get (x_dash, y_dash)
        double x_delta = lookahead_pose.pose.position.x - rbt_pose.pose.position.x;
        double y_delta = lookahead_pose.pose.position.y - rbt_pose.pose.position.y;

        double theta_rbt = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation); 
        // double x_dash = x_delta * cos(theta_rbt) +
        //                 y_delta * sin(theta_rbt);
        double y_dash = -x_delta * sin(theta_rbt) +
                        y_delta * cos(theta_rbt);


        // Calculate the curvature.
        double curvature = (2 * y_dash) / (dist * dist);

        // Calculate ω from v and c .
        double desired_angular_vel =curvature * desired_linear_vel_;

        // Constrain ω to within the largest allowable angular speed.
        desired_angular_vel = std::clamp(desired_angular_vel, -max_angular_vel_, max_angular_vel_);

        // Constrain v to within the largest allowable linear speed.
        double desired_linear_vel = std::clamp(desired_linear_vel_, -max_linear_vel_, max_linear_vel_);

        RCLCPP_INFO_STREAM(node_->get_logger(),
                             "Closest idx: " << closest_point_idx <<
                             ", Lookahead idx: " << lookahead_point_idx <<
                             ", x_delta: " << x_delta <<
                             ", y_delta: " << y_delta <<
                             ", dist: " << dist <<
                             ", curvature: " << curvature <<
                             ", desired_linear_vel: " << desired_linear_vel <<
                             ", desired_angular_vel: " << desired_angular_vel);
        return writeCmdVel(desired_linear_vel, desired_angular_vel);
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
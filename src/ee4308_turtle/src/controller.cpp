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
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_gain", this->yaw_gain_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".curvature_threshold", this->curvature_threshold_, 100.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".proximity_threshold", this->proximity_threshold_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".lookahead_gain", this->lookahead_gain_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".min_lookahead", this->min_lookahead_, 0.2);
        // initialize topics
        this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));

        this->pub_lookahead_dist_ = this->node_->create_publisher<std_msgs::msg::Float64>(
            "lookahead_distance", 10);
        this->pub_lookahead_marker_ = this->node_->create_publisher<visualization_msgs::msg::Marker>(
            "lookahead_marker", 10);
        this->pub_curvature_ = this->node_->create_publisher<std_msgs::msg::Float64>(
            "curvature", 10);
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
        if (ee4308::getDistance(rbt_pose.pose.position, goal_pose.pose.position) < xy_goal_thres_)
        {
            isGoalReached_ = true;
        }
        if (ee4308::getDistance(rbt_pose.pose.position, goal_pose.pose.position) > 2*xy_goal_thres_)
        {
            isGoalReached_ = false;
        }
        if (isGoalReached_)
        {
            double yaw_error = ee4308::limitAngle(ee4308::getYawFromQuaternion(goal_pose.pose.orientation) - ee4308::getYawFromQuaternion(rbt_pose.pose.orientation)) ;
            if (std::abs(yaw_error) > yaw_goal_thres_){
                RCLCPP_INFO_STREAM(node_->get_logger(), "Close to goal: yaw error " << yaw_error << " omega " << std::clamp(yaw_error* this->yaw_gain_, -max_angular_vel_, max_angular_vel_));
                return writeCmdVel(0, std::clamp(yaw_error* this->yaw_gain_, -max_angular_vel_, max_angular_vel_));
            }

            RCLCPP_INFO_STREAM(node_->get_logger(), "Goal reached!");
            return writeCmdVel(0, 0);
        }
        

        // Find the point along the path that is closest to the robot.
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
        size_t lookahead_point_idx = global_plan_.poses.size() -1;
        for (size_t i = closest_point_idx; i < global_plan_.poses.size(); ++i)
        {
            double temp_dist = ee4308::getDistance(rbt_pose.pose.position, global_plan_.poses[i].pose.position);
            if (temp_dist >= desired_lookahead_dist_)
            {
                lookahead_point_idx = i;
                break;
            }
        }
        geometry_msgs::msg::PoseStamped lookahead_pose = global_plan_.poses[lookahead_point_idx];

        // Publish lookahead distance as Float64
        {
            std_msgs::msg::Float64 dist_msg;
            dist_msg.data = desired_lookahead_dist_;
            pub_lookahead_dist_->publish(dist_msg);
        }

        // Publish lookahead point as a marker
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = node_->now();
            marker.ns = "lookahead";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = lookahead_pose.pose.position;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3f;
            marker.scale.x = 0.1;  // diameter in meters
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            pub_lookahead_marker_->publish(marker);
        }

        // NOW calculate dist to the actual lookahead point
        double dist = ee4308::getDistance(rbt_pose.pose.position, lookahead_pose.pose.position);

        // Transform the lookahead point into the robot frame to get (x_dash, y_dash)
        double x_delta = lookahead_pose.pose.position.x - rbt_pose.pose.position.x;
        double y_delta = lookahead_pose.pose.position.y - rbt_pose.pose.position.y;

        double theta_rbt = ee4308::limitAngle(ee4308::getYawFromQuaternion(rbt_pose.pose.orientation)); 

        double x_dash = x_delta * std::cos(theta_rbt) + y_delta * std::sin(theta_rbt);
        double y_dash = -x_delta * std::sin(theta_rbt) +
                        y_delta * std::cos(theta_rbt);
        double theta_dash = ee4308::limitAngle(atan2(y_dash, x_dash));
        //If point is behind robot, let robot turn
        if (std::abs(theta_dash) > 4*M_PI/5)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(),"Point behind robot: x_dash"  << x_dash << " theta_dash " << theta_dash);
            return writeCmdVel(0, std::clamp( theta_dash*this->yaw_gain_,  -max_angular_vel_, max_angular_vel_));
        }

        


        // Calculate the curvature.
        double curvature = (2 * y_dash) / (dist * dist);

        // Publish curvature as Float64
        {
            std_msgs::msg::Float64 curvature_msg;
            curvature_msg.data = curvature;
            pub_curvature_->publish(curvature_msg);
        }

        // Calculate ω from v and c .
        
        double desired_linear_vel = this->desired_linear_vel_;
        double desired_angular_vel =curvature * desired_linear_vel;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "Curvature: " << curvature << ", Dist: " << dist << "lookahead_dist_: " << desired_lookahead_dist_  );
        // Calculate the curvature heuristic. 
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Before curvature adjustment, desired_linear_vel: " << desired_linear_vel);
        desired_linear_vel = ( std::abs(curvature) > this->curvature_threshold_) ? desired_linear_vel * this->curvature_threshold_/std::abs(curvature) : desired_linear_vel; 

        // RCLCPP_INFO_STREAM(node_->get_logger(), "After curvature adjustment, desired_linear_vel: " << desired_linear_vel << " curvature " << curvature);    
        // Calculate the obstacle heuristic.
        double d_obstacle = getMinObstacleDistance_(); 
        desired_linear_vel = (d_obstacle < this->proximity_threshold_) ? desired_linear_vel * d_obstacle/this->proximity_threshold_ : desired_linear_vel;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "After proximity adjustment, desired_linear_vel: " << desired_linear_vel);
        
        // Vary the lookahead.
        desired_lookahead_dist_ = std::max(this->min_lookahead_, this->lookahead_gain_ * std::abs(desired_linear_vel));

        
        // Constrain ω to within the largest allowable angular speed.
        desired_angular_vel = std::clamp(desired_angular_vel, -max_angular_vel_, max_angular_vel_);

        // Constrain v to within the largest allowable linear speed.
        desired_linear_vel = std::clamp(desired_linear_vel, -max_linear_vel_, max_linear_vel_);

        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //                      "Closest idx: " << closest_point_idx <<
        //                      ", Lookahead idx: " << lookahead_point_idx <<
        //                      ", x_delta: " << x_delta <<
        //                      ", y_delta: " << y_delta <<
        //                      ", dist: " << dist);
        //                      ", curvature: " << curvature <<
        //                      ", desired_linear_vel: " << desired_linear_vel <<
        //                      ", desired_angular_vel: " << desired_angular_vel);
        return writeCmdVel(desired_linear_vel, desired_angular_vel);
    }

    double Controller::getMinObstacleDistance_()
    {
        if (scan_ranges_.empty())
        {
            return std::numeric_limits<double>::max();  // No scan data, assume no obstacles
        }

        double min_dist = std::numeric_limits<double>::max();
        for (const auto& range : scan_ranges_)
        {
            // Filter out invalid readings (inf, nan, or zero)
            if (std::isfinite(range) && range > 0.0)
            {
                min_dist = std::min(min_dist, static_cast<double>(range));
            }
        }
        
        return min_dist;
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
#include "ee4308_drone/behavior.hpp"

namespace ee4308::drone
{

    Behavior::Behavior(
        const rclcpp::NodeOptions &options,
        const std::string &name = "behavior")
        : Node(name, options)
    {
        // parameters
        this->reached_thres_ = ee4308::getParameter<double>(this, "reached_thres", 0.2).as_double();
        this->cruise_height_ = ee4308::getParameter<double>(this, "cruise_height", 4.0).as_double();
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 5.0).as_double();
        this->topic_turtle_plan_ = ee4308::getParameter<std::string>(this, "topic_turtle_plan", "/turtle/plan").as_string();
        this->topic_turtle_stop_ = ee4308::getParameter<std::string>(this, "topic_turtle_stop", "/turtle/stop").as_string();
        this->frame_id_map_ = ee4308::getParameter<std::string>(this, "map_frame_id", "map").as_string();
        this->initial_x_ = ee4308::getParameter<double>(this, "initial_x", -2.0).as_double();
        this->initial_y_ = ee4308::getParameter<double>(this, "initial_y", -2.0).as_double();
        this->initial_z_ = ee4308::getParameter<double>(this, "initial_z", 0.05).as_double();

        // topics
        this->sub_est_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
                                                                                 std::bind(&Behavior::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_turtle_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            this->topic_turtle_stop_, rclcpp::ServicesQoS(),
            std::bind(&Behavior::callbackSubTurtleStop_, this, std::placeholders::_1));
        this->sub_turtle_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            this->topic_turtle_plan_, rclcpp::SensorDataQoS(),
            std::bind(&Behavior::callbackSubTurtlePlan_, this, std::placeholders::_1));
        this->pub_enable_ = this->create_publisher<std_msgs::msg::Bool>("enable", rclcpp::ServicesQoS());

        // services
        this->cli_plan_ = this->create_client<nav_msgs::srv::GetPlan>("get_plan");

        // states
        this->turtle_stop_ = false;
        this->plan_requested_ = false;
        this->state_ = BEGIN;
        this->transition_(TAKEOFF);
        this->timer_ = this->create_timer(1s / frequency_, std::bind(&Behavior::callbackTimer_, this));

        // intercept prediction
        this->turtle_speed_ = 0.0;
        this->prev_turtle_x_ = 0.0;
        this->prev_turtle_y_ = 0.0;
        this->prev_turtle_time_ = this->now();
        this->turtle_speed_initialized_ = false;
        this->drone_cruise_speed_ = 0.75;  // conservative estimate of avg drone horizontal speed

    }

    void Behavior::callbackSubOdom_(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->odom_ = *msg;
    }

    void Behavior::callbackSubTurtleStop_(std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        this->turtle_stop_ = true;
    }

    void Behavior::callbackSubTurtlePlan_(nav_msgs::msg::Path::SharedPtr msg)
    {
        this->turtle_plan_ = *msg;
    }

    void Behavior::callbackTimer_()
    {
        // ==== make use of ====
        // state_
        // TAKEOFF, TURTLE_POSITION, TURTLE_WAYPOINT, INITIAL, LANDING, END
        // odom_
        // waypoint_x_, waypoint_y_, waypoint_z_
        // ==== ====
        // Update turtle-tracking waypoints every tick
        double alpha_drone = 0.3;
        drone_cruise_speed_= alpha_drone * std::hypot(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y) + (1.0 - alpha_drone) * drone_cruise_speed_;
        
        if (!turtle_plan_.poses.empty())
        {
            updateTurtleSpeed_();
            
        }

        if (state_ == TURTLE_POSITION && !turtle_plan_.poses.empty())
        {
            double intercept_x, intercept_y;
            computeInterceptPoint_(intercept_x, intercept_y);
            setWaypoint_(intercept_x, intercept_y, cruise_height_);
        }
        // if (state_ == TURTLE_POSITION && !turtle_plan_.poses.empty())
        // {
        //     setWaypoint_(turtle_plan_.poses[0].pose.position.x,
        //                 turtle_plan_.poses[0].pose.position.y,
        //                 cruise_height_);
        // }

        if (reachedWaypoint_())  // change the 1: if (reached waypoint)
        {
            //   If state is taking-off Then
            //    Transition to either initial state or turtle-position state.
            if (state_ == TAKEOFF)
            {
                if (turtle_stop_)
                    transition_(INITIAL);
                else
                    transition_(TURTLE_POSITION);
            }
            
            //   Else If state is initial Then
            //    Transition to landing or turtle-position state depending on whether the turtlebot has stopped.
            else if (state_ == INITIAL)
            {
                if (turtle_stop_)
                {
                    transition_(LANDING);
                }
                else
                {
                    transition_(TURTLE_POSITION);
                }
            }

            else if (state_ == TURTLE_POSITION)
            {
                intercept_initialized_ = false;
                transition_(TURTLE_WAYPOINT);
            }
            
            //   Else If state is turtle-waypoint Then
            //    Transition to initial.
            else if (state_ == TURTLE_WAYPOINT)
            {
                transition_(INITIAL);
            }

            //   Else If state is landing Then
            //    Transition to end.           
            else if (state_ == LANDING)
            {
                transition_(END);
            }
        }

        // request a plan with requestPlan(). This is done every time cbTimer() is called.
        requestPlan_(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z,
                     waypoint_x_, waypoint_y_, waypoint_z_);
    }

    void Behavior::transition_(int new_state)
    {
        // std::cout << "transition_ from " << state_ << " To " << new_state << std::endl;

        // ==== make use of ====
        // new_state (function argument)
        // state_
        // TAKEOFF, TURTLE_POSITION, TURTLE_WAYPOINT, INITIAL, LANDING, END
        // initial_x_, initial_y_, initial_z_,
        // cruise_height_
        // turtle_plan_.poses
        // turtle_stop_
        // =========

        // modify the following
        state_ = new_state;

        if (state_ == TAKEOFF)
        {
            // turn on the robot
            std_msgs::msg::Bool msg_enable;
            msg_enable.data = true;
            this->pub_enable_->publish(msg_enable);

            // set the waypoint.
            setWaypoint_(initial_x_, initial_y_, cruise_height_);
        }
        else if (state_ == END)
        {
            // turn off the robot
            std_msgs::msg::Bool msg_enable;
            msg_enable.data = false;
            this->pub_enable_->publish(msg_enable);

            // delete the timer to stop the state machine.
            this->timer_->cancel();
            this->timer_ = nullptr; 
        }
        else if (state_ == TURTLE_POSITION)
        {
            if (turtle_stop_ || turtle_plan_.poses.empty())
            {
                setWaypoint_(initial_x_, initial_y_, cruise_height_);
            }
            else
            {
                // Compute intercept point: predict where turtle will be when drone arrives
                double intercept_x, intercept_y;
                computeInterceptPoint_(intercept_x, intercept_y);
                setWaypoint_(intercept_x, intercept_y, cruise_height_);
                // setWaypoint_(turtle_plan_.poses[0].pose.position.x,
                //             turtle_plan_.poses[0].pose.position.y,
                //             cruise_height_);
            }
        }
        else if (state_ == TURTLE_WAYPOINT)
        {
            setWaypoint_(turtle_plan_.poses.back().pose.position.x, turtle_plan_.poses.back().pose.position.y, cruise_height_);
        }
        else if (state_ == INITIAL)
        {
            setWaypoint_(initial_x_, initial_y_, cruise_height_);
        }
        else if (state_ == LANDING)
        {
            setWaypoint_(initial_x_, initial_y_, initial_z_);
        }
    }

    void Behavior::setWaypoint_(double waypoint_x, double waypoint_y, double waypoint_z)
    {
        // you may choose to not use this function.
        // ==== make use of ====
        // waypoint_x_, waypoint_y_, waypoint_z_
        // waypoint_x, waypoint_y, waypoint_z
        // =========
        
        // remove or rewrite the following
        this->waypoint_x_ = waypoint_x; // remove or .
        this->waypoint_y_ = waypoint_y; // write to private class property.
        this->waypoint_z_ = waypoint_z; // write to private class property.
    }

    bool Behavior::reachedWaypoint_()
    {
        // you may choose to not use this function.
        // returns true if the current waypoint is reached.
        
        // remove or rewrite the following.

        return std::hypot(odom_.pose.pose.position.x - waypoint_x_,
                            odom_.pose.pose.position.y - waypoint_y_,
                            odom_.pose.pose.position.z - waypoint_z_) < reached_thres_;
    }

        void Behavior::updateTurtleSpeed_()
    {
        // Estimate turtle speed from consecutive position readings
        double turtle_x = turtle_plan_.poses[0].pose.position.x;
        double turtle_y = turtle_plan_.poses[0].pose.position.y;
 
        if (!turtle_speed_initialized_)
        {
            prev_turtle_x_ = turtle_x;
            prev_turtle_y_ = turtle_y;
            prev_turtle_time_ = this->now();
            turtle_speed_initialized_ = true;
            return;
        }
 
        double dt = (this->now() - prev_turtle_time_).seconds();
        if (dt < 0.05)  // avoid division by very small dt
            return;
 
        double dist = std::hypot(turtle_x - prev_turtle_x_, turtle_y - prev_turtle_y_);
        double instant_speed = dist / dt;
 
        // Exponential moving average for smooth speed estimate
        double alpha = 0.3;
        turtle_speed_ = alpha * instant_speed + (1.0 - alpha) * turtle_speed_;
 
        prev_turtle_x_ = turtle_x;
        prev_turtle_y_ = turtle_y;
        prev_turtle_time_ = this->now();
    }


    void Behavior::computeInterceptPoint_(double &intercept_x, double &intercept_y)
    {
        // Predict where the turtle will be when the drone arrives,
        // by walking along the turtle's planned path.
 
        double turtle_x = turtle_plan_.poses[0].pose.position.x;
        double turtle_y = turtle_plan_.poses[0].pose.position.y;
 
        // If we have no meaningful speed estimate, just target current position
        if (turtle_speed_ < 0.01)
        {
            intercept_x = turtle_x;
            intercept_y = turtle_y;
            return;
        }
 
        // Iterative refinement: compute intercept, then refine arrival time
        double target_x = turtle_x;
        double target_y = turtle_y;
        double arrival_time_turtle = 0.0;
        for (size_t i = 0; i + 1 < turtle_plan_.poses.size(); i++)
        {
            // Estimate drone travel time to current target
            double drone_dist = std::hypot(
                turtle_plan_.poses[i].pose.position.x - odom_.pose.pose.position.x,
                turtle_plan_.poses[i].pose.position.y - odom_.pose.pose.position.y);
            double arrival_time_drone = drone_dist / drone_cruise_speed_;


            double seg_dx = turtle_plan_.poses[i + 1].pose.position.x - turtle_plan_.poses[i].pose.position.x;
            double seg_dy = turtle_plan_.poses[i + 1].pose.position.y - turtle_plan_.poses[i].pose.position.y;
            double seg_len = std::hypot(seg_dx, seg_dy);
            arrival_time_turtle += seg_len / turtle_speed_;

   
            if (arrival_time_turtle >= arrival_time_drone)
            {
                target_x = turtle_plan_.poses[i].pose.position.x;
                target_y = turtle_plan_.poses[i].pose.position.y;
                break;
            }
        }
        // Smooth the intercept point to prevent jumping
        if (!intercept_initialized_)
        {
            smooth_intercept_x_ = target_x;
            smooth_intercept_y_ = target_y;
            intercept_initialized_ = true;
        }
        else
        {
            double alpha = 0.1;  // lower = smoother but slower to react
            smooth_intercept_x_ = alpha * target_x + (1.0 - alpha) * smooth_intercept_x_;
            smooth_intercept_y_ = alpha * target_y + (1.0 - alpha) * smooth_intercept_y_;
        }

        intercept_x = smooth_intercept_x_;
        intercept_y = smooth_intercept_y_;
    }

 
    // void Behavior::computeInterceptPoint_(double &intercept_x, double &intercept_y)
    // {
    //     // Predict where the turtle will be when the drone arrives,
    //     // by walking along the turtle's planned path.
 
    //     double turtle_x = turtle_plan_.poses[0].pose.position.x;
    //     double turtle_y = turtle_plan_.poses[0].pose.position.y;
 
    //     // If we have no meaningful speed estimate, just target current position
    //     if (turtle_speed_ < 0.01)
    //     {
    //         intercept_x = turtle_x;
    //         intercept_y = turtle_y;
    //         return;
    //     }
 
    //     // Iterative refinement: compute intercept, then refine arrival time
    //     double target_x = turtle_x;
    //     double target_y = turtle_y;
        
    //     for (int iteration = 0; iteration < 3; iteration++)
    //     {
    //         // Estimate drone travel time to current target
    //         double drone_dist = std::hypot(
    //             target_x - odom_.pose.pose.position.x,
    //             target_y - odom_.pose.pose.position.y);
    //         double arrival_time = drone_dist / drone_cruise_speed_;
 
    //         // Distance the turtle will travel along its path in that time
    //         double turtle_travel_dist = turtle_speed_ * arrival_time;
 
    //         // Walk along turtle_plan_.poses accumulating distance
    //         double accumulated = 0.0;
    //         target_x = turtle_plan_.poses.back().pose.position.x;
    //         target_y = turtle_plan_.poses.back().pose.position.y;
 
    //         for (size_t i = 0; i + 1 < turtle_plan_.poses.size(); i++)
    //         {
    //             double seg_dx = turtle_plan_.poses[i + 1].pose.position.x - turtle_plan_.poses[i].pose.position.x;
    //             double seg_dy = turtle_plan_.poses[i + 1].pose.position.y - turtle_plan_.poses[i].pose.position.y;
    //             double seg_len = std::hypot(seg_dx, seg_dy);
 
    //             if (accumulated + seg_len >= turtle_travel_dist)
    //             {
    //                 // Intercept point lies within this segment
    //                 double remaining = turtle_travel_dist - accumulated;
    //                 double frac = (seg_len > 1e-6) ? remaining / seg_len : 0.0;
    //                 target_x = turtle_plan_.poses[i].pose.position.x + frac * seg_dx;
    //                 target_y = turtle_plan_.poses[i].pose.position.y + frac * seg_dy;
    //                 break;
    //             }
    //             accumulated += seg_len;
    //         }
    //         // If turtle_travel_dist exceeds path length, target stays at path end (turtle's goal)
    //     }
 
    //     intercept_x = target_x;
    //     intercept_y = target_y;
    // }

    void Behavior::requestPlan_(double drone_x, double drone_y, double drone_z,
                                double waypoint_x, double waypoint_y, double waypoint_z)
    { // Sends a non-blocking service request to publish a path from the planner node.
        if (plan_requested_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No request is made as there is no response yet from previous request.");
            return;
        }
        plan_requested_ = true;
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->goal.header.frame_id = this->frame_id_map_;
        request->goal.header.stamp = this->now();
        request->goal.pose.position.x = waypoint_x;
        request->goal.pose.position.y = waypoint_y;
        request->goal.pose.position.z = waypoint_z;
        request->start.header.frame_id = this->frame_id_map_;
        request->start.header.stamp = this->now();
        request->start.pose.position.x = drone_x;
        request->start.pose.position.y = drone_y;
        request->start.pose.position.z = drone_z;
        request_plan_future_ = cli_plan_->async_send_request(request, std::bind(&Behavior::callbackCliReceivePlan_, this, std::placeholders::_1)).future;
    }

    void Behavior::callbackCliReceivePlan_(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
    {
        (void)future;
        plan_requested_ = false;
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Behavior);

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

        if (reachedWaypoint_())  // change the 1: if (reached waypoint)
        {
            //   If state is taking-off Then
            //    Transition to either initial state or turtle-position state.
            if (state_ == TAKEOFF)
            {
                transition_(INITIAL);
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
            if (turtle_stop_)
            {
                setWaypoint_(initial_x_, initial_y_, cruise_height_);
            }
            else
            {
                setWaypoint_(turtle_plan_.poses[0].pose.position.x, turtle_plan_.poses[0].pose.position.y, cruise_height_);
            }
        }
        else if (state_ == TURTLE_WAYPOINT)
        {
            setWaypoint_(turtle_plan_.poses[-1].pose.position.x, turtle_plan_.poses[-1].pose.position.y, cruise_height_);
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

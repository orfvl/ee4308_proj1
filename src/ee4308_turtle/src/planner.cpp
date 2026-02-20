#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{

    // ====================== Planner Node ===================
    AStarNode::AStarNode(int new_c, int new_r) : c(new_c), r(new_r) {}

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;
        this->costmap_ = costmap_ros->getCostmap();
        this->global_frame_id_ = costmap_ros->getGlobalFrameID();

        // declare parameters to let the node know we are using these params.
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_access_cost", this->max_access_cost_, 254);
        ee4308::initParam(this->node_, this->plugin_name_ + ".sg_half_cost", this->sg_half_cost_, 4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".sg_order", this->sg_order_, 3);
        ee4308::initParam(this->node_, this->plugin_name_ + ".interpolation_distance", this->interpolation_distance_, 0.05);
    
    }

    // Converts world coordinates to cell column and cell row.
    std::pair<int, int> Planner::XYToCR_(double x, double y)
    {
        double resolution = costmap_->getResolution();
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();
        
        int c = static_cast<int>(std::floor((x - origin_x) / resolution));
        int r = static_cast<int>(std::floor((y - origin_y) / resolution));
        
        return {c, r};
    }
    
    // Converts cell column and cell row to world coordinates.
    std::pair<double, double> Planner::CRToXY_(int c, int r)
    {
        double resolution = costmap_->getResolution();
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();
        
        // +0.5 to get the center of the cell
        double x = origin_x + (c + 0.5) * resolution;
        double y = origin_y + (r + 0.5) * resolution;
        
        return {x, y};
    }

    std::pair<bool, double> Planner::LOS(int c1, int r1, int c2, int r2)
    {
        auto [start_x, start_y] = this->CRToXY_(c1, r1);
        auto [end_x, end_y] = this->CRToXY_(c2, r2);

        double dx = (end_x - start_x);
        double dy = (end_y - start_y);

        double step_size = costmap_->getResolution()*0.5;
        double steps = std::max(std::abs(dx), std::abs(dy)) / step_size;
        double step_x = dx/steps;
        double step_y = dy/steps;
        double los_cost = 0.0;

        double cur_x = start_x;
        double cur_y = start_y;

        for (int i = 1; i <= steps; ++i)
        {
            cur_x += step_x;
            cur_y += step_y;
            auto [check_c, check_r] = this->XYToCR_(cur_x, cur_y);
            if (costmap_->getCost(check_c, check_r) > this->max_access_cost_)
            {
                return {false, los_cost};
            }
            los_cost += (1+costmap_->getCost(check_c, check_r)) * std::hypot(step_x, step_y); 
        }
        return {true, los_cost};
    }



    // Converts cell column and cell row to flattened array index.
    int Planner::CRToIndex_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getSizeInCellsX()
        //   this->costmap_->getSizeInCellsY()

        // Row-major order: index = row * num_columns + column
        return r * costmap_->getSizeInCellsX() + c;
    }

    // Returns true if out of map, false otherwise.
    bool Planner::outOfMap_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getSizeInCellsX()
        //   this->costmap_->getSizeInCellsY()

        return c < 0 || r < 0 || 
           c >= static_cast<int>(costmap_->getSizeInCellsX()) || 
           r >= static_cast<int>(costmap_->getSizeInCellsY());
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> /*cancel_checker*/)
    {
        // =========== DELETE / COMMENT LINES IN {} ONCE READY TO CODE PLANNER ===================
        // { // Start (for lab 1 and testing)
        //     nav_msgs::msg::Path path;
        //     path.poses.clear();
        //     path.header.frame_id = this->global_frame_id_;
        //     path.header.stamp = this->node_->now();
            
        //     double dx = start.pose.position.x - goal.pose.position.x;
        //     double dy = start.pose.position.y - goal.pose.position.y;
        //     int num_steps = std::floor(std::hypot(dx, dy) / 0.05);
        //     std::vector<AStarNode> nodes;
        //     for (int s = 0; s < num_steps; ++s)
        //     {
        //         geometry_msgs::msg::PoseStamped pose; 
        //         pose.pose.position.x = dx * s / num_steps + goal.pose.position.x;
        //         pose.pose.position.y =  dy * s / num_steps + goal.pose.position.y;
        //         path.poses.push_back(pose);
        //     }

        //     std::reverse(path.poses.begin(), path.poses.end());

        //     geometry_msgs::msg::PoseStamped goal_ = goal;
        //     goal_.header.frame_id = "";
        //     goal_.header.stamp = rclcpp::Time(); 
        //     path.poses.push_back(goal_);

        //     return path;
        // } // End (for lab 1 and testing)

        // =========== Initializations ===================

        // Create a vector of nodes (modify accordingly)
        // TODO: use costmap_->getSizeInCellsX() and costmap_->getSizeInCellsY() to get the actual map size and initialize the nodes vector accordingly.
        std::vector<AStarNode> nodes;
        for (size_t r = 0; r < costmap_->getSizeInCellsY(); ++r)
        {
            for (size_t c = 0; c < costmap_->getSizeInCellsX(); ++c)
            {
                nodes.emplace_back(c, r);
            }
        }

        // Create an open list
        OpenList<AStarNode *> open_list;

        // get the c,r map coordinates of the start and goal points
        auto [start_c, start_r] = this->XYToCR_(start.pose.position.x, start.pose.position.y);
        auto [goal_c, goal_r] = this->XYToCR_(goal.pose.position.x, goal.pose.position.y);

        // do some start node initialization (modify accordingly)
        int start_idx = this->CRToIndex_(start_c, start_r);
        AStarNode *start_node = &nodes[start_idx];
        start_node->g = 0.0; //Initialize start node with 0 g -cost.
        start_node->h = 0.0;
        start_node->f = 0.0;
        open_list.push(start_node); //Queue start node into open-list.

        // ================ Expansion loop ========================
        while (rclcpp::ok() && !open_list.empty())
        {
            // pop the cheapest
            AStarNode *node = open_list.top();
            open_list.pop();

            // RCLCPP_INFO_STREAM(this->node_->get_logger(), "Expanding node at c: " << node->c << ", r: " << node->r << " with f: " << node->f);

            // If n was previously expanded Then Continue 
            if (node->expanded)
            {
                continue;
            }

            // do something if goal found
            if (goal_c == node->c && goal_r == node->r)
            {   
            
                auto preliminary_path = this->writeToPath_(node, goal);
                
                // Apply Savitsky Golay smoothing to the path.
                auto smoothed_path = this->savitsky_golay_smoothing_(preliminary_path);

                return smoothed_path;
            }

            // Mark n as expanded.
            node->expanded = true;

            // do stuff in expansion loop

            // ================ Neighbor loop ========================
            for (auto [dc, dr] : std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}})
            {
                
                int nb_c = node->c + dc;
                int nb_r = node->r + dr;

                // RCLCPP_INFO_STREAM(this->node_->get_logger(), "  Checking neighbor at c: " << nb_c << ", r: " << nb_r);

                // (void) nb_c; // avoids unused variable warnings. Can be deleted.
                // (void) nb_r; // avoids unused variable warnings. Can be deleted.

                // do stuff in neighbor loop

                // outofmap check
                if (this->outOfMap_(nb_c, nb_r)) {
                    continue;
                }

                // Skip if cost too high (inaccessible)
                unsigned char cost = costmap_->getCost(nb_c, nb_r);
                if (cost > this->max_access_cost_) {
                    continue;
                }

                if(node->parent != nullptr){
                  auto [los_bool, los_cost] = this->LOS(node->parent->c, node->parent->r, nb_c, nb_r);
                  if (los_bool){
                    auto g_tilde = node->parent->g + los_cost;
                    auto nb_idx = this->CRToIndex_(nb_c, nb_r);
                    if (g_tilde < nodes[nb_idx].g) {
                        // update node info
                        nodes[nb_idx].g = g_tilde;
                        nodes[nb_idx].h = this->calculateHeuristic_(nb_c, nb_r, goal_c, goal_r);
                        nodes[nb_idx].f = nodes[nb_idx].g + nodes[nb_idx].h;
                        nodes[nb_idx].parent = node->parent;

                        // push to open list if not expanded
                        if (!nodes[nb_idx].expanded) {
                            open_list.push(&nodes[nb_idx]);
                        }
                        continue;
                       }
                    }
                }

                //Fall back to Astar 
                auto [nb_x, nb_y] = this->CRToXY_(nb_c, nb_r);
                auto nb_idx = this->CRToIndex_(nb_c, nb_r);
                auto [node_x, node_y] = this->CRToXY_(node->c, node->r);
                auto distance_nb = std::hypot(nb_x - node_x, nb_y - node_y);
                auto g_tilde = node->g + distance_nb * (this->costmap_->getCost(nb_c, nb_r)+1); //TODO: check if there is Euclidean cost weight or something
                
                if (g_tilde < nodes[nb_idx].g) {
                    // update node info
                    nodes[nb_idx].g = g_tilde;
                    nodes[nb_idx].h = this->calculateHeuristic_(nb_c, nb_r, goal_c, goal_r);
                    nodes[nb_idx].f = nodes[nb_idx].g + nodes[nb_idx].h;
                    nodes[nb_idx].parent = node;

                    // push to open list if not expanded
                    if (!nodes[nb_idx].expanded) {
                        open_list.push(&nodes[nb_idx]);
                    }
                }
                
            }       
            
        }

        // If we reach here, then there is no path found.
        RCLCPP_WARN(this->node_->get_logger(), "No path found!!!!!");
        return this->writeToPath_(nullptr, goal); // no path
    }

    double Planner::calculateHeuristic_(int c, int r, int goal_c, int goal_r)
    {
        // You may use std::hypot() function.
        return std::hypot(goal_c - c, goal_r - r);

        // Chebyshev distance heuristic (admissible for 8-connected grid)
        //return std::max(std::abs(goal_c - c), std::abs(goal_r - r));
    }

    nav_msgs::msg::Path Planner::savitsky_golay_smoothing_(
        const nav_msgs::msg::Path &preliminary_path)
    {
        nav_msgs::msg::Path smoothed_path = preliminary_path;
       
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(
            2 * this->sg_half_cost_ + 1,
            this->sg_order_ + 1);
        
        for (int i = 0; i < this->sg_order_ + 1; i++){
            for (int j = 0; j < 2 * this->sg_half_cost_ + 1; j++){
                if(i == 0) {
                    J(j, i) = 1;
                    continue;
                }
                J(j, i) = std::pow((-this->sg_half_cost_ + j), i);
            }
        }

        Eigen::RowVectorXd A = ((J.transpose() * J).inverse() * J.transpose()).row(0);
        
        for (int k=0; k < static_cast<int>(preliminary_path.poses.size()); k++){
            smoothed_path.poses[k].pose.position.x = 0.0;
            smoothed_path.poses[k].pose.position.y = 0.0;
            for (int point=0; point < 2 * this->sg_half_cost_ + 1; point++){
                int idx = k - this->sg_half_cost_ + point;
                if (idx < 0) {
                    idx = 0;
                }
                if (idx >= static_cast<int>(preliminary_path.poses.size())){
                    idx = preliminary_path.poses.size() - 1;
                }
                smoothed_path.poses[k].pose.position.x += A(point) * preliminary_path.poses[idx].pose.position.x;
                smoothed_path.poses[k].pose.position.y += A(point) * preliminary_path.poses[idx].pose.position.y;
            }
        }
        return smoothed_path;
    }

    nav_msgs::msg::Path Planner::writeToPath_(
        AStarNode *goal_node,
        geometry_msgs::msg::PoseStamped goal)
    {
        // setup the path message
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = this->global_frame_id_;
        path.header.stamp = this->node_->now();

        // do whatever is required to get the path
        AStarNode* node = goal_node;
        while (node != nullptr)
        { 
            // convert map coordinates to world coordinates
            auto [wx, wy] = this->CRToXY_(node->c, node->r);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);

            // go to the next node
            node = node->parent;
        }
        
        // don't forget to reverse the path!
        std::reverse(path.poses.begin(), path.poses.end());

        nav_msgs::msg::Path interpolated_path;
        interpolated_path.header = path.header;

        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            if (i == 0)
            {
                interpolated_path.poses.push_back(path.poses[i]);
                continue;
            }

            double x0 = path.poses[i - 1].pose.position.x;
            double y0 = path.poses[i - 1].pose.position.y;
            double x1 = path.poses[i].pose.position.x;
            double y1 = path.poses[i].pose.position.y;
            double dist = std::hypot(x1 - x0, y1 - y0);
            int num_interp = static_cast<int>(std::ceil(dist / this->interpolation_distance_));

            for (int j = 1; j <= num_interp; ++j)
            {
                double t = static_cast<double>(j) / (num_interp + 1);
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = x0 + t * (x1 - x0);
                pose.pose.position.y = y0 + t * (y1 - y0);
                pose.pose.orientation.w = 1.0;
                interpolated_path.poses.push_back(pose);
            }

            interpolated_path.poses.push_back(path.poses[i]);
        }

        // push the original goal (contains the final yaw angle of the robot)
        goal.header.frame_id = "";
        goal.header.stamp = rclcpp::Time(); // possible bug: prevents nav2 and tf2 from having time extrapolation issues.
        interpolated_path.poses.push_back(goal);

        // return path;
        return interpolated_path;
    }

    // ======================================== DO NOT TOUCH =================================

    void Planner::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)
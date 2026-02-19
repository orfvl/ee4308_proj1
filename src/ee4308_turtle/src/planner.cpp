#include "ee4308_turtle/planner.hpp"

#include <algorithm>
#include <cmath>

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
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_access_cost", this->max_access_cost_, 254); // default 254
        ee4308::initParam(this->node_, this->plugin_name_ + ".los_max_access_cost", this->los_max_access_cost_, 40); // default 40 
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

    bool Planner::hasLineOfSight_(
        const geometry_msgs::msg::PoseStamped &a,
        const geometry_msgs::msg::PoseStamped &b)
    {
        if (!costmap_) return false;

        // Treat inflation as blocked as well (tune this!)
        // 200 is a common starting point: blocks deep inflation but not all of it.
        const unsigned char LOS_BLOCK_COST = static_cast<unsigned char>(los_max_access_cost_); // TODO: tune this!

        auto [c0, r0] = XYToCR_(a.pose.position.x, a.pose.position.y);
        auto [c1, r1] = XYToCR_(b.pose.position.x, b.pose.position.y);

        if (outOfMap_(c0, r0) || outOfMap_(c1, r1)) return false;

        // Endpoint sanity (if endpoints are in high cost, reject)
        if (costmap_->getCost(c0, r0) >= LOS_BLOCK_COST) return false;
        if (costmap_->getCost(c1, r1) >= LOS_BLOCK_COST) return false;

        // Supercover-style sampling in cell space (robust, hard to "miss" walls)
        const int dc = c1 - c0;
        const int dr = r1 - r0;

        const int steps = std::max(std::abs(dc), std::abs(dr));
        if (steps == 0) return true;

        // Oversample to reduce aliasing (tune: 2 is usually plenty)
        const int samples = steps * 2;

        int prev_c = c0;
        int prev_r = r0;

        for (int i = 0; i <= samples; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(samples);
            const int c = static_cast<int>(std::lround(c0 + t * dc));
            const int r = static_cast<int>(std::lround(r0 + t * dr));

            if (outOfMap_(c, r)) return false;

            // Blocked cell?
            if (costmap_->getCost(c, r) >= LOS_BLOCK_COST) return false;

            // Corner-cut prevention: if we moved diagonally, check both side cells too
            if (c != prev_c && r != prev_r)
            {
                if (!outOfMap_(prev_c, r) && costmap_->getCost(prev_c, r) >= LOS_BLOCK_COST) return false;
                if (!outOfMap_(c, prev_r) && costmap_->getCost(c, prev_r) >= LOS_BLOCK_COST) return false;
            }

            prev_c = c;
            prev_r = r;
        }

        return true;
    }



    //     nav_msgs::msg::Path Planner::lineOfSightPrune_(const nav_msgs::msg::Path &in)
    // {
    //     nav_msgs::msg::Path out;
    //     out.header = in.header;
    //     out.poses.clear();

    //     const size_t n = in.poses.size();
    //     if (n == 0) return out;
    //     if (n <= 2) { out.poses = in.poses; return out; }

    //     size_t i = 0;
    //     out.poses.push_back(in.poses[i]);

    //     while (i < n - 1)
    //     {
    //         size_t best = i + 1;

    //         // Grow forward until LOS breaks, keep farthest visible
    //         for (size_t j = i + 1; j < n; ++j)
    //         {
    //             if (hasLineOfSight_(in.poses[i], in.poses[j]))
    //             {
    //                 best = j;
    //             }
    //             else
    //             {
    //                 break; // LOS usually won't come back after it breaks
    //             }
    //         }

    //         out.poses.push_back(in.poses[best]);
    //         i = best;
    //     }

    //     return out;
    // }



    nav_msgs::msg::Path Planner::lineOfSightPrune_(const nav_msgs::msg::Path &in)
    {
        // Line-of-sight pruning to remove unnecessary intermediate waypoints
        // from an anchor pose i, connect directly to the farthest
        // pose j we can still "see"; then set i=j and repeat.
        nav_msgs::msg::Path out;
        out.header = in.header;
        out.poses.clear();

        const size_t n = in.poses.size();
        if (n == 0)
        {
            return out;
        }
        if (n <= 2)
        {
            out.poses = in.poses;
            return out;
        }

        size_t anchor_index = 0;
        out.poses.push_back(in.poses[anchor_index]);

        while (anchor_index < n - 1)
        {
            // Start by trying to connect to the end of the path,
            // and walk backwards until line-of-sight is satisfied.
            size_t candidate_index = n - 1;
            while (candidate_index > anchor_index + 1 &&
                   !this->hasLineOfSight_(in.poses[anchor_index], in.poses[candidate_index]))
            {
                --candidate_index;
            }

            out.poses.push_back(in.poses[candidate_index]);
            anchor_index = candidate_index;
        }

        return out;
    }

        nav_msgs::msg::Path Planner::interpolatePath_(const nav_msgs::msg::Path &in, double step)
    {
        nav_msgs::msg::Path out;
        out.header = in.header;
        out.poses.clear();

        if (in.poses.empty()) return out;
        if (in.poses.size() == 1) { out.poses = in.poses; return out; }

        out.poses.push_back(in.poses.front());

        for (size_t k = 0; k + 1 < in.poses.size(); ++k)
        {
            const auto &p0 = in.poses[k].pose.position;
            const auto &p1 = in.poses[k + 1].pose.position;

            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            const double L  = std::hypot(dx, dy);

            if (L < 1e-9)
            {
                continue;
            }

            const int num = std::max(1, static_cast<int>(std::floor(L / step)));

            for (int i = 1; i <= num; ++i)
            {
                const double t = static_cast<double>(i) / static_cast<double>(num);

                geometry_msgs::msg::PoseStamped pose = in.poses[k]; // copy header/frame
                pose.pose.position.x = p0.x + t * dx;
                pose.pose.position.y = p0.y + t * dy;
                pose.pose.position.z = 0.0;

                // leave orientation alone (controller usually ignores it for intermediate points)
                out.poses.push_back(pose);
            }
        }

        // Ensure exact last pose matches input last pose
        out.poses.back() = in.poses.back();
        return out;
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

                // Then prune (clearance-aware + forward greedy)
                auto pruned_path = this->lineOfSightPrune_(preliminary_path);

                // Then re-interpolate so controller has enough points
                auto interpolated_path = this->interpolatePath_(pruned_path, this->interpolation_distance_); //TODO: tune the interpolation distance (tradeoff: too dense -> more smoothing but more computation, too sparse -> less smoothing but less computation). You can also make this a parameter if you want.

                // Smooth first (dense points -> safe smoothing)
                auto smoothed_path = this->savitsky_golay_smoothing_(interpolated_path);

        
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
                if (cost >= this->max_access_cost_) {
                    continue;
                }

                auto [nb_x, nb_y] = this->CRToXY_(nb_c, nb_r);
                auto nb_idx = this->CRToIndex_(nb_c, nb_r);
                auto [node_x, node_y] = this->CRToXY_(node->c, node->r);
                auto distance_nb = std::hypot(nb_x - node_x, nb_y - node_y);
                auto g_tilde = node->g + distance_nb * this->costmap_->getCost(nb_c, nb_r); //TODO: check if there is Euclidean cost weight or something
                
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
        // return std::hypot(goal_c - c, goal_r - r);

        // Chebyshev distance heuristic (admissible for 8-connected grid)
        return std::max(std::abs(goal_c - c), std::abs(goal_r - r));
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

        // push the original goal (contains the final yaw angle of the robot)
        goal.header.frame_id = "";
        goal.header.stamp = rclcpp::Time(); // possible bug: prevents nav2 and tf2 from having time extrapolation issues.
        path.poses.push_back(goal);

        // return path;
        return path;
    }

    // ======================================== DO NOT TOUCH =================================

    void Planner::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)
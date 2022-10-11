#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    Node* goalNode = new Node(goalCell.x, goalCell.y);

    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node* startNode = new Node(startCell.x, startCell.y);
    startNode->g_cost = 0.0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);

    PriorityQueue openList;
    openList.push(startNode);
    std::vector<Node*> closedList;

    bool found_path = false;
    while (!openList.empty())
    {
        Node* nextNode = openList.pop();
        closedList.push_back(nextNode);
        if (*nextNode == *goalNode) 
        {
            found_path = true;
            goalNode = nextNode;
            break;
        }
        auto children = expand_node(nextNode, distances, params);

        for (auto &&child : children)
        {
            // Check if any of those is in the closed list
            Node* childNode = NULL;
            auto childNode_closed = get_from_list(child, closedList);
            auto childNode_open = openList.get_member(child);
            if (childNode_closed != NULL)
                childNode = childNode_closed;
            else if (childNode_open != NULL) 
                childNode = childNode_open;

            if (childNode != NULL)
            {
                // Update the g cost if newer is better
                auto new_g_cost = g_cost(nextNode, child, distances, params);
                
                if (new_g_cost < childNode->g_cost)
                {
                    childNode->g_cost = new_g_cost;
                    childNode->parent = nextNode;
                }
                continue;
            }
            else if (!openList.is_member(child))
            {
                child->g_cost = g_cost(nextNode, child, distances, params);
                child->h_cost = h_cost(child, goalNode, distances);
                child->parent = nextNode;
                openList.push(child);
            }
        }
    }
    mbot_lcm_msgs::robot_path_t path;
    path.utime = start.utime;
    if (found_path) 
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }
     
    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // Diagonal distance: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
    int dx = std::abs(goal->cell.x - from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);
    double diag_distance = 1.414;

    double h_cost = (dx + dy) + (diag_distance - 2 ) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // We want to add the obstacles distances to the g_cost, to penalize paths that go very near the obstacles

    // For now, just add the cost of traveling from from to goal to the from's g cost.
    double obstacle_penalization = 0.0;
    // If the distance to the obstacle is inside the range, set the penalization
    if (distances(goal->cell.x, goal->cell.y) <= params.maxDistanceWithCost)
    {
        obstacle_penalization = std::pow(params.maxDistanceWithCost - distances(goal->cell.x, goal->cell.y), params.distanceCostExponent) ;
    }

    int dx = std::abs(goal->cell.x - from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);

    double g_cost = from->g_cost + obstacle_penalization;
    if (dx == 1 && dy == 1)
        g_cost += 1.414;
    else 
        g_cost += 1.0;
    
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // return all the neighbors of the current node, given certain conditions
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1 };

    std::vector<Node*> children;
    for (int n = 0; n < 8; ++n)
    {
        int cell_x = node->cell.x + xDeltas[n];
        int cell_y = node->cell.y + yDeltas[n];
        Node* childNode = new Node(cell_x, cell_y);
        // Check that the child is inside the map
        if (!distances.isCellInGrid(cell_x, cell_y))
            continue;
        // Check that the child is not an obstacle (or close enough to one)
        if (distances(cell_x, cell_y) <= params.minDistanceToObstacle) 
            continue;
        children.push_back(childNode);
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    auto curr_node = goal_node;

    while (!(*curr_node == *start_node))
    {
        path.push_back(curr_node);
        curr_node = curr_node->parent;
    }
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // For now, use every cell.
    auto new_node_path = prune_node_path(nodes);
    // Efficiently prune every point that lies in a straight line between two other waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    for (auto &&node : new_node_path)
    {
        auto global_pose = grid_position_to_global_position(node->cell, distances);
        mbot_lcm_msgs::pose_xyt_t pose;
        pose.x = global_pose.x;
        pose.y = global_pose.y;
        // Set the angle to the direction of the movement from the previous point to this one
        if (path.size() > 0)
        {
            auto last_pose = path.back();
            pose.theta = std::atan2(pose.y -  last_pose.y, pose.x - last_pose.x);
        }
        else pose.theta = 0;
        
        pose.utime = 0;

        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;

    if (nodePath.size() < 3) return nodePath;

    auto ref_node = nodePath[0];
    new_node_path.push_back(ref_node);

    for (size_t i = 1; i < nodePath.size() - 1; i++)
    {
        ref_node = nodePath[i - 1];
        auto second_node = nodePath[i];
        auto third_node = nodePath[i + 1];
        // I assume there are only 8 possible directions (to the neighboring cells)
        Point<int> ref_direction(second_node->cell.x - ref_node->cell.x, second_node->cell.y - ref_node->cell.y);
        Point<int> new_direction(third_node->cell.x - second_node->cell.x, third_node->cell.y - second_node->cell.y);

        // Nothing has changed
        if (ref_direction == new_direction) continue;

        // Add the pivot node and change it to be the reference node
        new_node_path.push_back(second_node);
        ref_node = second_node;
    }
    // Add the goal node
    new_node_path.push_back(nodePath.back());

    return new_node_path;

}

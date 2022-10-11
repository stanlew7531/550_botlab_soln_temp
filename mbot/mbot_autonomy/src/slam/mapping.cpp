#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    // Update log odds
    //  Cells the laser hit
    for (auto& ray : movingScan) 
        scoreEndpoint(ray, map);
    
    // Cells the laser went through
    for (auto& ray : movingScan) 
        scoreRay(ray, map);
    
    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    if (ray.range <= kMaxLaserDistance_)
    {
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        // Check if on map
        if (map.isCellInGrid(rayCell.x, rayCell.y))
        {
            if (map(rayCell.x, rayCell.y)  + kHitOdds_ < 127)
            {
                map(rayCell.x, rayCell.y) += kHitOdds_;
            }
            else 
            {
                map(rayCell.x, rayCell.y) = 127;
            }
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    auto cells_touched = bresenham(ray, map);
    // auto cells_touched = divideAndStepAlongRay(ray, map);

    for (auto &&cell : cells_touched)
    {
        if (map.isCellInGrid(cell.x, cell.y))
        {
            if (map(cell.x, cell.y) - kMissOdds_ > -127)
            {
                map(cell.x, cell.y) -= kMissOdds_;
            }
            else 
            {
                map(cell.x, cell.y) = -127;
            }
        }
    }
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    // Point<float> f_start = global_position_to_grid_position(ray.origin, map);
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);

    // Algorithm starts
    int dx = abs(end_cell.x - start_cell.x);
    int dy = abs(end_cell.y - start_cell.y);
    // Direction of movement
    int sx = start_cell.x < end_cell.x ? 1 : -1;
    int sy = start_cell.y < end_cell.y ? 1 : -1;
    int error = dx - dy;

    Point<int> curr_point(start_cell.x, start_cell.y);
    std::vector<Point<int>> cells_touched;
    while (curr_point.x != end_cell.x || curr_point.y != end_cell.y) 
    {
        // Push a copy of current point
        cells_touched.push_back(Point<int>(curr_point.x, curr_point.y));
        int e2 = 2 * error;
        if (e2 >= -dy)
        {
            error -= dy;
            curr_point.x += sx;
        }
        if (e2 <= dx)
        {
            error += dx;
            curr_point.y += sy;
        }
    }

    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    double c_t = std::cos(ray.theta);
    double s_t = std::sin(ray.theta);
    float step_x = map.metersPerCell() / 2 * c_t;
    float step_y = map.metersPerCell() / 2 * s_t;

    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * c_t,
        ray.origin.y + ray.range * s_t
        ), map);
    
    double curr_x = ray.origin.x;
    double curr_y = ray.origin.y;
    Point<int> last_cell_seen;
    bool initialized = false;
    std::vector<Point<int>> cells_touched;
    while (true) 
    {
        // Get cell of current position
        auto cell = global_position_to_grid_cell(Point<double>(curr_x,curr_y), map);
        if (cell == end_cell) break;
        // Check if I went pass the end cell
        float dy = curr_y - ray.origin.y;
        float dx = curr_x - ray.origin.x;
        if (std::sqrt(dx * dx + dy * dy) >= ray.range) break;

        // If not already pushed, push cell
        if (!initialized)
        {
            initialized = true;
            cells_touched.push_back(cell);
        }
        else if (last_cell_seen != cell)
        {
            cells_touched.push_back(cell);
        }

        curr_x += step_x;
        curr_y += step_y;

        last_cell_seen = cell;
    }

    return cells_touched;
}

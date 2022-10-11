#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    if (search_range == 0) 
    {
        bfs_offsets_.push_back(Point<int>{0, 0});
    }
    else
    {
        for(int i =-search_range; i < search_range; i++)
        {
            for(int j=-search_range; j < search_range; j++)
            {
                bfs_offsets_.push_back(Point<int>{i, j});
            }
        }
        auto less_than = [&](Point<int>& a, Point<int>& b) { return a.norm() < b.norm(); };
        std::sort(bfs_offsets_.begin(), bfs_offsets_.end(), less_than);
    }
    max_offset_norm = Point<float>(search_range+0.05, search_range+0.05).norm();
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double scanScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);

    // Update maximum score expected
    int scanCount = std::floor(movingScan.size()/ray_stride_);
    max_scan_score = scanCount * max_offset_norm;

    // Score rays
    for (const auto& ray : movingScan)
	{
		if(ray.range < max_ray_range_)
		{ 
            auto ray_score = scoreRay(ray, map);
            scanScore += ray_score;
		}
    }

    // return particle likelihood
    return scanScore / max_scan_score;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    double rayScore = 0;
	Point<float> end_point 			= getRayEndPointOnMap(ray, map);
    Point<int> grid_end_point       = global_position_to_grid_cell(end_point, map);
	Point<int> occupied_cell_offset	= gridBFS(grid_end_point, map);
    
    float offset = Point<float>(occupied_cell_offset).norm();
    float offset_quality = std::max<float>(0.0, max_offset_norm - offset);  
    return offset_quality * offset_quality_weight;
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    for(Point<int> offset : bfs_offsets_)
	{
		Point<int> check_loc = {end_point.x + offset.x, end_point.y + offset.y};
		if (map.isCellInGrid(check_loc.x, check_loc.y))
        {
            if(map.logOdds(check_loc.x, check_loc.y) > occupancy_threshold_)
            {
                return offset;
            }
        }
	}
    if (search_range == 0) { return Point<int>(1, 0); }
    else { return Point<int>(search_range, search_range); }

}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
	Point<float> ray_end; 
    ray_end.x = (ray.range * std::cos(ray.theta)) + ray.origin.x; 
    ray_end.y = (ray.range * std::sin(ray.theta)) + ray.origin.y;
    return ray_end;
}

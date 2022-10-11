#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;

    posteriorPose_ = pose;

    for (auto &&p : posterior_)
    {
        p.pose.x = pose.x;
        p.pose.y = pose.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;

        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;

    randomPoseGen = RandomPoseSampler(&map);
    
    for (auto &&p : posterior_)
    {
        auto pose = randomPoseGen.get_pose();
        p.pose = pose;
        p.parent_pose = pose;

        p.weight = sampleWeight;
    }
    posteriorPose_ = posterior_[0].pose;
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(&map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    // TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;

    // Low variance sampler from Probabilistic Robotics
    std::random_device rd;
    std::default_random_engine generator(rd());
    auto uniform_distribution = std::uniform_real_distribution<double>(0.0, 1.0/(static_cast<double>(kNumParticles_)));
    double r = uniform_distribution(generator);

    double cum_sum = posterior_[0].weight;
    int particle_index = 0;
    for (size_t m = 0; m < kNumParticles_; m++)
    {
        // Augmentation: randomly sample if average weights worsening
        if(samplingAugmentation.sample_randomly()) 
        {
            prior.push_back(randomPoseGen.get_particle());
        }
        else
        {  
            double navigator_u = r + static_cast<double>(m) / kNumParticles_;
            while (navigator_u > cum_sum)
            {
                particle_index++;
                cum_sum += posterior_[particle_index].weight;
                
            }
            prior.push_back(posterior_[particle_index]);

        }
    }

    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate 
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());
        
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {  
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) || 
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", " 
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen.get_particle();
    //         }
    //     }
    // }

    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
     // Take the proposal, and apply the actions
    for (auto &&p : prior)
    {
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double sumWeights = 0.0;

    for (auto &&p : proposal)
    {
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }
    auto avg_weight = sumWeights/proposal.size();
    samplingAugmentation.insert_average_weight(avg_weight);
    distribution_quality = avg_weight / sensorModel_.max_scan_score;
    randomPoseGen = RandomPoseSampler(&map);

    // Renormalize
    for (auto &&p : posterior)
    {
        p.weight /= sumWeights;
    }

    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.
    mbot_lcm_msgs::pose_xyt_t pose;
    ParticleList posterior_copy = posterior;
    // Sort posterior vector of particles by weight in descending order
    std::sort( posterior_copy.begin( ), posterior_copy.end( ), [ ]( const mbot_lcm_msgs::particle_t& lhs, const mbot_lcm_msgs::particle_t& rhs )
    {
        return lhs.weight > rhs.weight;
    });
    // Get a certain number of particles to be averaged
    // Best X%
    double best_x_percent = 0.03;
    int num_particles_of_interest = static_cast<int>(best_x_percent * kNumParticles_);
    ParticleList to_be_averaged = ParticleList(posterior_copy.begin(), posterior_copy.begin() + num_particles_of_interest);
    // Average particles in the to_be_averaged vector
    // Get the pose from the resulting particle
    pose = computeParticlesAverage(to_be_averaged);
    // pose = posterior_copy[0].pose;

    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}

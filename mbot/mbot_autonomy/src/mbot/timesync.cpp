#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/timestamp_t.hpp>
#include <common_utils/lcm_config.h>
#include <common_utils/time_util.h>
#include <mbot/mbot_channels.h>
#include <iostream>

/**
	A program that gets the current system time and publishes an lcm message of the current time
**/
int main(){

	//sleep duration between time samplings
	const int sleep_usec = 1000000;

	lcm::LCM lcmConnection(MULTICAST_URL);
	if(!lcmConnection.good()) return 1;

	mbot_lcm_msgs::timestamp_t now;

	while(true){

		now.utime = utime_now();

		lcmConnection.publish(MBOT_TIMESYNC_CHANNEL, &now);

		usleep(sleep_usec);
	}

	return 0;
}

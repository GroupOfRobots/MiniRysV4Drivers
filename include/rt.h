#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include <string>

#ifndef RT_H_
#define RT_H_

#define MAX_SAFE_STACK (8*1024)

void setRTPriority(std::string name = "Main") {
	int policy = SCHED_RR;
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(policy)-1;
	printf("[%s] Setting RT scheduling, priority %d.\n", (name + ":rt_set").c_str(), schedulerParams.sched_priority);

	if (sched_setscheduler(0, policy, &schedulerParams) == -1) {
		printf("[%s] WARNING: Setting RT scheduling failed: %s.\n", (name + ":rt_set").c_str(), strerror(errno));
		return;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		printf("[%s] WARNING: Failed to lock memory: %s.\n", (name + ":rt_set").c_str(), strerror(errno));
	}

	unsigned char dummy[MAX_SAFE_STACK];
	memset(dummy, 0, MAX_SAFE_STACK);
}

#endif
/*
 * MyExecutor.hpp
 *
 *  Created on: Oct 16, 2019
 *      Author: dangield
 */

#ifndef MYEXECUTOR_MYEXECUTOR_HPP_
#define MYEXECUTOR_MYEXECUTOR_HPP_

#include <chrono>
#include <mutex>

struct Exec
{
	std::mutex *m;
	bool *activate;
	std::chrono::milliseconds delay;
	std::chrono::time_point<std::chrono::high_resolution_clock> nextActivationTime;
	std::string name;
	Exec *next;
	Exec();
};

class MyExecutor {
	private:
		Exec *firstExec;
		bool *destroy;
	public:
		MyExecutor(bool&);
		~MyExecutor();
		void addExec(std::mutex& mut, bool& activationBool, std::chrono::milliseconds threadDelay, std::string name);
		void spin();
		void list();
};

#endif /* MYEXECUTOR_MYEXECUTOR_HPP_ */

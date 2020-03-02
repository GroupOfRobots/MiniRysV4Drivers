#include "FrequencyCounter/FrequencyCounter.hpp"
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

unsigned int FrequencyCounter::id_counter = 0;

FrequencyCounter::FrequencyCounter() {

	this->previous = std::chrono::high_resolution_clock::now();
	this->timeNow = std::chrono::high_resolution_clock::now();

	this->numOfRuns = 0;
	this->frequency = 0.0;

	this->id = id_counter;
	++id_counter;

	printf("[Frequency Counter %d] Ready\n", id);
}

void FrequencyCounter::count() {
	this->numOfRuns++;
	// std::this_thread::sleep_for(2ms);
	// rclcpp::sleep_for(2ms);
	if (this->numOfRuns > 999) {
		this->previous = this->timeNow;
		this->timeNow = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		float loopTime = loopTimeSpan.count();
		this->frequency = this->numOfRuns/loopTime;
		printf("[Frequency Counter %d] Frequency %f Hz after %d messages\n", id, frequency, numOfRuns);
		// std::cout <<"[FrequencyCounter] Frequency " << this->frequency << "Hz after " << numOfRuns << " messages." << std::endl;
		this->numOfRuns = 0;
	}
}

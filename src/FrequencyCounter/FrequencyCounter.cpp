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

	this->id = FrequencyCounter::id_counter;
	FrequencyCounter::id_counter++;

	printf("[Frequency Counter %d] Ready\n", id);
	// printf("%u %u\n", this->id, this->numOfRuns);

	clock_gettime(CLOCK_REALTIME, &this->previous2);
	this->timeNow2 = this->previous2;
	// std::cout << previous2.tv_sec << ' ' << previous2.tv_nsec << std::endl;
	// std::cout << timeNow2.tv_sec << ' ' << timeNow2.tv_nsec << std::endl;
}

void FrequencyCounter::count() {
	this->numOfRuns++;
	// printf("%u %u\n", this->id, this->numOfRuns);
	if (this->numOfRuns > 999) {
		this->previous = this->timeNow;
		this->timeNow = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		float loopTime = loopTimeSpan.count();
		this->frequency = this->numOfRuns/loopTime;
		printf("[Frequency Counter %u] Frequency %f Hz after %u messages\n", id, frequency, numOfRuns);
		this->numOfRuns = 0;
	}
}

void FrequencyCounter::count2() {
	this->numOfRuns++;
	// printf("%u %u\n", this->id, this->numOfRuns);
	if (this->numOfRuns > 999) {
		this->previous2.tv_sec = this->timeNow2.tv_sec;
		this->previous2.tv_nsec = this->timeNow2.tv_nsec;
		clock_gettime(CLOCK_REALTIME, &this->timeNow2);
		// auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		// float loopTime = loopTimeSpan.count();
		float loopTime = this->timeNow2.tv_sec - this->previous2.tv_sec + float(this->timeNow2.tv_nsec - this->previous2.tv_nsec)/1000000000;
		this->frequency = this->numOfRuns/loopTime;
		printf("[Frequency Counter %u] Frequency %f Hz after %u messages\n", this->id, this->frequency, this->numOfRuns);
		// std::cout << previous2.tv_sec << ' ' << previous2.tv_nsec << std::endl;
		// std::cout << timeNow2.tv_sec << ' ' << timeNow2.tv_nsec << std::endl;
		this->numOfRuns = 0;
	}
}

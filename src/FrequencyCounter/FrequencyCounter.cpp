#include "FrequencyCounter/FrequencyCounter.hpp"
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

unsigned int FrequencyCounter::id_counter = 0;

FrequencyCounter::FrequencyCounter(std::string customName) {

	this->previous = std::chrono::steady_clock::now();
	this->timeNow = std::chrono::steady_clock::now();

	this->numOfRuns = 0;
	this->frequency = 0.0;

	this->id = FrequencyCounter::id_counter;
	if (customName.compare("") == 0) sprintf(name, "Frequency Counter %d", id);
	else sprintf(name, "Frequency Counter %d: %s", id, customName.c_str());
	FrequencyCounter::id_counter++;

	RCLCPP_INFO(rclcpp::get_logger(name), "Ready.");


	clock_gettime(CLOCK_REALTIME, &this->previous2);
	this->timeNow2 = this->previous2;
}

FrequencyCounter::~FrequencyCounter() {
		RCLCPP_INFO(rclcpp::get_logger(name), "Frequency %f Hz after %u messages", frequency, numOfRuns);
}

void FrequencyCounter::count() {
	this->numOfRuns++;
	if (this->numOfRuns > 999) {
		this->previous = this->timeNow;
		this->timeNow = std::chrono::steady_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		float loopTime = loopTimeSpan.count();
		this->frequency = this->numOfRuns/loopTime;
		// printf("[Frequency Counter %u] Frequency %f Hz after %u messages\n", id, frequency, numOfRuns);
		RCLCPP_INFO(rclcpp::get_logger(name), "Frequency %f Hz after %u messages", frequency, numOfRuns);
		this->numOfRuns = 0;
	}
}

void FrequencyCounter::count2() {
	this->numOfRuns++;
	if (this->numOfRuns > 999) {
		this->previous2.tv_sec = this->timeNow2.tv_sec;
		this->previous2.tv_nsec = this->timeNow2.tv_nsec;
		clock_gettime(CLOCK_REALTIME, &this->timeNow2);
		float loopTime = this->timeNow2.tv_sec - this->previous2.tv_sec + float(this->timeNow2.tv_nsec - this->previous2.tv_nsec)/1000000000;
		this->frequency = this->numOfRuns/loopTime;
		RCLCPP_INFO(rclcpp::get_logger(name), "Frequency %f Hz after %u messages", frequency, numOfRuns);
		this->numOfRuns = 0;
	}
}

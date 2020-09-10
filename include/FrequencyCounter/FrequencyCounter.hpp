#include <chrono>
#include <string>
#include <time.h>

class FrequencyCounter {
	private:
		std::chrono::time_point<std::chrono::steady_clock> previous;
		std::chrono::time_point<std::chrono::steady_clock> timeNow;

		unsigned int numOfRuns;
		float frequency;

		unsigned int id;
		static unsigned int id_counter; 

		timespec previous2;
		timespec timeNow2;

	public:
		void count();
		void count2();
		FrequencyCounter();
		~FrequencyCounter();
};

#include <chrono>
#include <string>

class FrequencyCounter {
	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> previous;
		std::chrono::time_point<std::chrono::high_resolution_clock> timeNow;

		int numOfRuns;
		float frequency;

		int id;
		static unsigned int id_counter; 

	public:
		void count();
		FrequencyCounter();
};

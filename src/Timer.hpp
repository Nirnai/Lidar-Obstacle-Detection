#include <iostream>
#include <chrono>


using namespace std::chrono;

class Timer
{
    public:
        Timer()
        {
            m_startTime = high_resolution_clock::now();
        }

        ~Timer()
        {
            stop();
        }
        
        void stop()
        {
            auto endTime = high_resolution_clock::now();
            auto start = time_point_cast<milliseconds>(m_startTime).time_since_epoch().count();
            auto end = time_point_cast<milliseconds>(endTime).time_since_epoch().count();
            auto duration = end - start;
            std::cout <<  "Duration : " << duration << " ms" << std::endl;
        }

    private:
        time_point<high_resolution_clock> m_startTime;
};
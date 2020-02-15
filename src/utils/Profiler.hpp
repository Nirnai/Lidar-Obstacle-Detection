#pragma once

#include <string>
#include <fstream>
#include <algorithm>
#include <chrono>   

struct ProfileResult
{
    std::string name;
    long long start, end;
};

struct ProfilerSession
{
    std::string Name;
};

class Profiler
{
public:
    Profiler() : m_currentSession(nullptr), m_profileCount(0){}
    
    void beginSession(const std::string& name, const std::string& filepath = "../profile/profile.json")
    {
        m_outputStream.open(filepath);
        writeHeader();
        m_currentSession = new ProfilerSession{name};
    }

    void writeProfile(const ProfileResult& result)
    {
        if (m_profileCount++ > 0)
            m_outputStream << ",";

        std::string name = result.name;
        std::replace(name.begin(), name.end(), '"', '\'');

        m_outputStream << "{";
        m_outputStream << "\"cat\":\"function\",";
        m_outputStream << "\"dur\":" << (result.end - result.start) << ',';
        m_outputStream << "\"name\":\"" << name << "\",";
        m_outputStream << "\"ph\":\"X\",";
        m_outputStream << "\"pid\":0,";
        m_outputStream << "\"tid\":0,";
        m_outputStream << "\"ts\":" << result.start;
        m_outputStream << "}";

        m_outputStream.flush();

    }

    void endSession()
    {
        writeFooter();
        m_outputStream.close();
        delete m_currentSession;
        m_currentSession = nullptr;
        m_profileCount = 0;
    }

    void writeHeader()
    {
        m_outputStream << "{\"otherData\": {},\"traceEvents\":[";
        m_outputStream.flush();
    }

    void writeFooter()
    {
        m_outputStream << "]}";
        m_outputStream.flush();
    }

    static Profiler& get()
    {
        static Profiler instance;
        return instance;
    }

private:
    ProfilerSession* m_currentSession;
    std::ofstream m_outputStream;
    int m_profileCount;

};


class ProfilerTimer
{
public:
    ProfilerTimer(std::string name)
        :m_name(name)
    {
        m_startTime = std::chrono::high_resolution_clock::now();
    }

    ~ProfilerTimer()
    {
        stop();
    }
    
    void stop()
    {
        auto endTime = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTime).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTime).time_since_epoch().count();
        auto duration = end - start;
        Profiler::get().writeProfile({m_name, start, end});
    }

private:
    std::string m_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <locale>
#include <codecvt>
#include <thread>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include "utils.h"

namespace fs = boost::filesystem;

bool usleep(int64_t usec);

namespace ilsr {

    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;

    double clamp_loop(double q, double lb, double ub)
    {
        const double origin = (ub + lb) / 2;
        const double range = std::abs(ub - lb);
        assert(range > DBL_EPSILON);
        // limit to (origin-range, origin+range)
        double residual = std::fmod(q - origin, range);
        // limit to (origin-range/2, origin+range/2)
        residual += (residual > (range / 2)) * (-range) + (residual < (-range / 2)) * (range);
        return residual + origin;
    }

    double Time::wall_time()
    {
        static const boost::posix_time::ptime t0(boost::gregorian::date(1970, 1, 1));
        const boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration delta(t1 - t0);
        return delta.total_microseconds() / 1000000.0;
    }

    void Time::sleep_for(double dt)
    {
        using namespace std::chrono;
        using clock = high_resolution_clock;
        using elapse = duration<int64_t, nanoseconds>;
        steady_clock::time_point t = clock::now() + nanoseconds(int64_t(dt * 1e9));

        while (t - clock::now() > microseconds(100)) {
            if (!usleep(100)) {
                std::this_thread::sleep_for(microseconds(100));
            }
        }
        while (clock::now() < t) {
            if (!usleep(1)) {
                std::this_thread::sleep_for(microseconds(1));
            }
        }
    }

    std::string Time::timestamp()
    {
        using namespace boost::posix_time;
        // set your formatting
        std::stringstream is;
        is.imbue(std::locale(
            is.getloc(), new boost::posix_time::time_facet("%H:%M:%S.%f"))); // HHmmSS.xxxxxx
        is << microsec_clock::local_time();
        return is.str();
    }

    std::string Time::logtime()
    {
        using namespace boost::posix_time;
        // set your formatting
        std::stringstream is;
        is.imbue(std::locale(is.getloc(), new boost::posix_time::time_facet("%Y%m%d_%H%M%S")));
        is << second_clock::local_time();
        return is.str();
    }

    std::wstring convert(const std::string &input) { return converter.from_bytes(input); }

    std::string convert(const std::wstring &input) { return converter.to_bytes(input); }

    std::string join(const std::vector<std::string> &parts)
    {
        std::string s;
        for (const auto &piece : parts)
            s += piece;
        return s;
    }

    std::vector<std::string> split(const std::string &s, const std::string &delimiters)
    {
        std::vector<std::string> tokens;
        std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
        std::string::size_type pos = s.find_first_of(delimiters, lastPos);
        while (std::string::npos != pos || std::string::npos != lastPos) {
            tokens.push_back(s.substr(lastPos, pos - lastPos));
            lastPos = s.find_first_not_of(delimiters, pos);
            pos = s.find_first_of(delimiters, lastPos);
        }
        return tokens;
    }

#pragma region LogFile

    Logger::Logger(std::string file, std::string folder)
    {
        const fs::path dir = fs::current_path();
        fs::path fd(folder);
        if (!folder.empty() && fs::is_directory(fd)) {
            if (!fd.is_absolute()) {
                fd = dir / fd;
            }
        } else {
            // 使用默认路径
            fd = dir / "log";
        }
        if (!fs::exists(fd)) {
            fs::create_directories(fd);
        }

        m_FilePath = (fd / file.c_str()).string();
        CreateLogFile(m_FilePath);
        m_Thread = std::make_shared<boost::thread>(boost::bind(&Logger::Process, this));
    }

    Logger::~Logger()
    {
        if (m_Thread) {
            m_Thread->interrupt();
            m_Thread->join();
        }
        m_LogFile.close();
    }

    int Logger::Process()
    {
        while (!boost::this_thread::interruption_requested()) {
            if (!m_LogFile.is_open()) {
                CreateLogFile(m_FilePath);
            }
            WriteLogFile();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(250));
            boost::this_thread::interruption_point();
        }
        // 将尚未存储的记录写入文件
        WriteLogFile();
        return 0;
    }

    std::string Logger::GetFilePath() const { return m_FilePath; }

    void Logger::AddLog(std::string log)
    {
#if (defined(USING_LOG) && USING_LOG)
        // boost::lock_guard<decltype(m_lock)> lock(m_lock);
        m_LogList.push(log);
#endif // #if (defined(USING_LOG) && USING_LOG)
    }

    bool Logger::CreateLogFile(const std::string &file)
    {
#if (defined(USING_LOG) && USING_LOG)
        // boost::lock_guard<decltype(m_lock)> lock(m_lock);
        if (!m_LogFile.is_open()) {
            m_LogFile.open(file, std::ios::in | std::ios::out | std::ios::app | std::ios::ate);
            return m_LogFile.is_open();
        }
#endif // #if (defined(USING_LOG) && USING_LOG)
        return false;
    }

    void Logger::WriteLogFile()
    {
        try {

#if (defined(USING_LOG) && USING_LOG)
            // 记录程序信息
            // boost::lock_guard<decltype(m_lock)> lock(m_lock);
            std::string msg;
            while (m_LogList.try_pop(msg)) {
                m_LogFile << msg;
            }
            m_LogFile.flush();
#else
            m_LogList.clear();
#endif // #if (defined(USING_LOG) && USING_LOG)

        } catch (std::exception e) {
            std::cout << e.what() << std::endl;
        }
    }

#pragma endregion LogFile

} // namespace ilsr

#if defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <windows.h>
#include <iomanip>

// From: https://gist.github.com/GoaLitiuM/aff9fbfa4294fa6c1680

unsigned long setHighestTimerResolution(unsigned long timer_res_us)
{
    static const HINSTANCE ntdll = LoadLibraryA("ntdll.dll");
    static unsigned long timer_current_res = ULONG_MAX;
    if (ntdll != NULL) {
        typedef long(NTAPI * pNtSetTimerResolution)(
            unsigned long RequestedResolution, BOOLEAN Set, unsigned long *ActualResolution);

        pNtSetTimerResolution NtSetTimerResolution
            = (pNtSetTimerResolution)GetProcAddress(ntdll, "NtSetTimerResolution");
        if (NtSetTimerResolution != NULL) {
            // bounds are validated and set to the highest allowed resolution
            NtSetTimerResolution(timer_res_us, TRUE, &timer_current_res);
        }
        // we can decrement the internal reference count by one
        // and NTDLL.DLL still remains loaded in the process
        FreeLibrary(ntdll);
    }
    return timer_current_res;
}

bool usleep(int64_t usec)
{
    static size_t currentResolution;
    if (currentResolution == 0)
        currentResolution = setHighestTimerResolution(1);

    HANDLE timer;
    LARGE_INTEGER period;

    // negative values are for relative time
    period.QuadPart = -(10 * usec);

    if (!(timer = CreateWaitableTimerW(NULL, TRUE, NULL))) {
        return false;
    }
    if (SetWaitableTimer(timer, &period, 0, NULL, NULL, 0)) {
        WaitForSingleObject(timer, INFINITE);
        CloseHandle(timer);
        return false;
    }
    CloseHandle(timer);
    return true;
}
#endif

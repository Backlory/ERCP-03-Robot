#ifndef __UTILS__
#define __UTILS__

#pragma once

#include <assert.h>
#include <stdint.h>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <boost/thread/thread.hpp>

#if defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <sdkddkver.h>
#endif

#if defined(USING_TBB)
#include <tbb/concurrent_queue.h>
template <typename _Ty>
using concurrent_queue = tbb::concurrent_queue<_Ty>;
#elif defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <ppl.h>
#include <concurrent_queue.h>
template <typename _Ty>
using concurrent_queue = concurrency::concurrent_queue<_Ty>;
#endif

#ifdef USING_LOGURU
#include "loguru/loguru.hpp"
#endif

#if __cplusplus < 201703L && _MSVC_LANG < 201703L
namespace std {

    // FUNCTION TEMPLATE clamp
    template <class _Ty, class _Pr>
    _NODISCARD constexpr const _Ty &clamp(
        const _Ty &_Val, const _Ty &_Min_val, const _Ty &_Max_val, _Pr _Pred)
    {
        // returns _Val constrained to [_Min_val, _Max_val]
#if _ITERATOR_DEBUG_LEVEL == 2
        if (_DEBUG_LT_PRED(_Pred, _Max_val, _Min_val)) {
            _STL_REPORT_ERROR("invalid bounds arguments passed to std::clamp");
            return _Val;
        }
#endif // _ITERATOR_DEBUG_LEVEL == 2
        if (_DEBUG_LT_PRED(_Pred, _Max_val, _Val)) {
            return _Max_val;
        }
        if (_DEBUG_LT_PRED(_Pred, _Val, _Min_val)) {
            return _Min_val;
        }
        return _Val;
    }

    template <class _Ty>
    _NODISCARD constexpr const _Ty &clamp(const _Ty &_Val, const _Ty &_Min_val, const _Ty &_Max_val)
    {
        // returns _Val constrained to [_Min_val, _Max_val]
        return _STD clamp(_Val, _Min_val, _Max_val, less<_Ty>{});
    }

} // namespace std
#endif

class not_implemented_error : public std::exception {
public:
    not_implemented_error(const std::string &name)
        : object(name)
    {
        message = "Object `" + object + "` not implemented.";
    }

    const char *what() const throw() { return message.c_str(); }

    std::string message;
    const std::string object;
};

namespace ilsr {

    /**
     * @brief 限制数值在区间内，溢出则循环补足
     * @param q 输入数值
     * @param lb 下界
     * @param ub 上界
     * @return double
     * @note [例] 限制16在-1~5之间，返回4
     */
    double clamp_loop(double q, double lb, double ub);

    std::wstring convert(const std::string &input);

    std::string convert(const std::wstring &input);

    std::string join(const std::vector<std::string> &parts);

    std::vector<std::string> split(const std::string &s, const std::string &delimiters = " ");

    /**
     * @brief 获取程序当前时间，单位[s]
     */
    class Time {
    public:
        static const Time &GetInstance()
        {
            static Time _time;
            return _time;
        }

        static double wall_time();

        static void sleep_for(double dt);

        const double get_start_time() const { return m_start_time; }

        const double software_time() const { return wall_time() - m_start_time; }

        /// Generate time in "[%H%M%S.%ms]" format.
        static std::string timestamp();

        /// Generate time in "%Y%m%d_%H%M%S" format.
        static std::string logtime();

    private:
        double m_start_time;

        Time() { m_start_time = wall_time(); }

        Time(const Time &) = delete;
    };

    class Logger {

    public:
        /**
         * @brief 记录器构造器
         * @param file 记录文件名，无需路径，存放在程序目录log文件夹下面对应启动时间的目录下
         */
        Logger(std::string file, std::string folder = "");
        Logger(const Logger &) = delete;
        ~Logger();

    public:
        // 添加记录
        void AddLog(std::string log);

        std::string GetFilePath() const;

    protected:
        // 线程处理函数
        int Process();
        // 创建LOG文件
        bool CreateLogFile(const std::string &file);
        // 将缓存队列的信息写入LOG文件
        void WriteLogFile();

    private:
        // 记录文件路径
        std::string m_FilePath;
        // 程序LOG文件
        std::ofstream m_LogFile;
        // 文件读写线程
        std::shared_ptr<boost::thread> m_Thread;
        // 程序LOG缓存队列，定时写入文件
        concurrent_queue<std::string> m_LogList;
    };

    /**
     * @brief 加锁的数据类型
     */
    template <typename T>
    class mutex_data {
    public:
        using data_type = T;

        struct stamped_data {
            double time = -1;
            T data;
        };

    public:
        mutex_data() = default;

        mutex_data(const T &default_value)
            : m_data(default_value)
        {
        }

        mutex_data(const mutex_data &) = delete;

        void Set(const T &data)
        {
            double t = ilsr::Time::wall_time();
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            m_time = t;
            m_data = data;
        }

        void Set(T &&data)
        {
            double t = ilsr::Time::wall_time();
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            m_time = t;
            m_data = std::move(data);
        }

        double GetTime() const { return m_time; }

        T Get(double overtime) const
        {
            double t = ilsr::Time::wall_time();
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            if (m_time > 0 && t - m_time < overtime) {
                return m_data;
            }
            throw std::runtime_error("MutexData failed.");
        }

        T UnsafeGet() const
        {
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            return m_data;
        }

        bool TryGet(T &data, double overtime) const
        {
            double t = ilsr::Time::wall_time();
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            if (m_time > 0 && t - m_time < overtime) {
                data = m_data;
                return true;
            }
            return false;
        }

        bool TryGet(stamped_data &data, double overtime) const
        {
            double t = ilsr::Time::wall_time();
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            if (m_time > 0 && t - m_time < overtime) {
                data.data = m_data;
                data.time = m_time;
                return true;
            }
            return false;
        }

        bool TryMove(T &data, double overtime)
        {
            double t = ilsr::Time::wall_time();
            std::lock_guard<decltype(m_mutex)> lock(m_mutex);
            if (m_time > 0 && t - m_time < overtime) {
                data = std::move(m_data);
                m_time = -1;
                return true;
            }
            return false;
        }

    private:
        mutable std::mutex m_mutex;
        double m_time = -1;
        T m_data;
    };

} // namespace ilsr

#endif // __UTILS__

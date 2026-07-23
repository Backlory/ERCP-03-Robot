#pragma once
#include <stdint.h>
#include <map>

#if defined(_WIN32) || defined(WIN32) || defined(_WINDOWS) || defined(WINDOWS)
    #ifdef ROBOT_EXPORT
    #define ROBOT_API_MEMBER __declspec(dllexport)
    #define ROBOT_API        extern "C" ROBOT_API_MEMBER
    #else
    #define ROBOT_API_MEMBER __declspec(dllimport)
    #define ROBOT_API        extern "C" ROBOT_API_MEMBER
    #endif
#else
    #define ROBOT_API_MEMBER
    #define ROBOT_API
#endif

#if defined(USING_LOG) && USING_LOG
#ifdef USING_LOGURU
#include "loguru/loguru.hpp"
#define ROBOT_INFO(cond, x)    VLOG_IF_S(loguru::Verbosity_INFO, cond) << x;
#define ROBOT_ERROR(cond, x)   VLOG_IF_S(loguru::Verbosity_ERROR, cond) << x;
#define ROBOT_THREADNAME(name) loguru::set_thread_name(name);
#else
#include <iostream>
#define ROBOT_INFO(cond, x)    ((cond) ? (std::cout << x << std::endl) : (std::cout));
#define ROBOT_ERROR(cond, x)   ((cond) ? (std::cout << x << std::endl) : (std::cout));
#define ROBOT_THREADNAME(name) (void)0;
#endif
#else
#define ROBOT_INFO(cond, x)    (void)0;
#define ROBOT_ERROR(cond, x)   (void)0;
#define ROBOT_THREADNAME(name) (void)0;
#endif

/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * Very trivial logger implementation
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include <iostream>
#include <sstream>

#define SIMPLE_LOGGER simple_logger::Logger

#define LOG_LEVEL_INFO simple_logger::LogLevel::INFO
#define LOG_LEVEL_DEBUG simple_logger::LogLevel::DEBUG
#define LOG_LEVEL_WARNING simple_logger::LogLevel::WARNING
#define LOG_LEVEL_ERROR simple_logger::LogLevel::ERROR

#define SLOG_DEBUG SIMPLE_LOGGER(LOG_LEVEL_DEBUG)
#define SLOG_INFO SIMPLE_LOGGER(LOG_LEVEL_INFO)
#define SLOG_WARNING SIMPLE_LOGGER(LOG_LEVEL_WARNING)
#define SLOG_ERROR SIMPLE_LOGGER(LOG_LEVEL_ERROR)

#define LOG_DEBUG(_msg) SIMPLE_LOGGER(LOG_LEVEL_INFO) << _msg;
#define LOG_INFO(_msg) SIMPLE_LOGGER(LOG_LEVEL_INFO) << _msg;
#define LOG_WARNING(_msg) SIMPLE_LOGGER(LOG_LEVEL_WARNING) << _msg;
#define LOG_ERROR(_msg) SIMPLE_LOGGER(LOG_LEVEL_ERROR) << _msg;

namespace simple_logger
{
enum class LogLevel
{
    ERROR,
    WARNING,
    INFO,
    DEBUG
};

struct Logger
{
    Logger(LogLevel level) : m_level(level){};
    ~Logger();

    std::stringstream m_os;

  private:
    LogLevel m_level;
};

template <typename T> Logger& operator<<(Logger& logger, T&& t)
{
    logger.m_os << std::forward<T>(t);
    return logger;
}

template <typename T> Logger& operator<<(Logger&& logger, T&& t)
{
    return logger << std::forward<T>(t);
}
} // namespace SIMPLE_LOGGER
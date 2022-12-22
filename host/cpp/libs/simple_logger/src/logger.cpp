/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * Very trivial logger implementation
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "simple_logger/logger.hpp"

namespace simple_logger
{
Logger::~Logger()
{
    switch (m_level)
    {
    case LogLevel::DEBUG:
        std::cout << "DEBUG_INFO: ";
        break;
    case LogLevel::INFO:
        std::cout << "INFO: ";
        break;
    case LogLevel::WARNING:
        std::cout << "WARNING: ";
        break;
    case LogLevel::ERROR:
        std::cout << "ERROR: ";
        break;
    }
    std::cout << m_os.str() << std::endl;
}
} // namespace simple_logger

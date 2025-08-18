/**
 * @file logger.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2025-06-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "logger.hpp"

void Logger::log(LogLevel level, const std::string& msg) {

    std::string logLevel;
    std::string colorCode;

    // Determine log level and color code
    switch (level) {
        case LogLevel::DEBUG:
          logLevel = "DEBUG";
          colorCode = "\033[36m";
          break;
        case LogLevel::INFO:
          logLevel = "INFO";
          colorCode = "\033[37m";
          break;
        case LogLevel::WARN:
          logLevel = "WARN";
          colorCode = "\033[33m";
          break;
        case LogLevel::ERROR:
          logLevel = "ERROR";
          colorCode = "\033[31m";
          break;
        case LogLevel::SUCCESS:
          logLevel = "SUCCESS";
          colorCode = "\033[1;32m";
          break;
        case LogLevel::FAILURE:
          logLevel = "FAILURE";
          colorCode = "\033[1;31m";
          break;
    }

    // Create a timestamp for the log entry
    std::ostringstream timeStream;
    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&t_c);
    timeStream << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

    // Format the log message
    std::string log = "[" + timeStream.str() + "] [" + logLevel + "] " + msg;

    std::cout << colorCode << log << "\033[0m\n";
}

LogStream Logger::createLogStream(LogLevel level) {
    return LogStream(level);
}
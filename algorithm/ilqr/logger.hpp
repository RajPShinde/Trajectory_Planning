/**
 * @file logger.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2025-06-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <string>

/**
* @brief Enumeration for log levels
*/
enum class LogLevel {DEBUG, INFO, WARN, ERROR, SUCCESS, FAILURE};

/**
* @brief Forward Declararion of LogStream class
*/
struct LogStream;

/**
* @brief Logger class for handling logging functionality
*/
class Logger {
 public:
	/**
	* @brief Logs a message with a specified log level
	* @param level Log level for the message (DEBUG, INFO, WARN, ERROR)
	* @param msg Message to be logged
	*/
    static void log(LogLevel level, const std::string& msg);

	/**
	* @brief Creates a LogStream object for streaming log messages
    * 
    * This function allows for more flexible logging by enabling the use of stream operators
    * to build log messages incrementally.
    *
	* @param level Log level for the stream (DEBUG, INFO, WARN, ERROR)
    * @return LogStream object for streaming log messages
	*/
    static LogStream createLogStream(LogLevel level);
};

/**
* @brief LogStream class for streaming log messages
*/
struct LogStream {
 public:
    /**
    * @brief Constructor for LogStream
    * @param level Log level for this stream (DEBUG, INFO, WARN, ERROR)
    */
    LogStream(LogLevel level) : level_(level){
    }

    /**
    * @brief Destructor for LogStream
    *
    * This destructor logs the accumulated message in the buffer to the log file
    */
    ~LogStream(){
        Logger::log(level_, buffer_.str());
    }

    /**
    * @brief Move constructor for LogStream
    */
    LogStream(LogStream&&) = default;
    
    /**
    * @brief Move assignment operator for LogStream
    */
    LogStream& operator=(LogStream&&) = default;

    /**
    * @brief Overloaded operator<< for streaming log messages
    * @param value Value to be added to the log message
    * @return Reference to the current LogStream object
    */
    template<typename T>
    LogStream& operator<<(const T& value) {
        buffer_ << value;
        return *this;
    }

 private:
    LogLevel level_;  //! Log level for this stream
    std::ostringstream buffer_;  //! Buffer to hold the log message
};

/**
* @brief Macro definitions for logging
*/
#define LOG_INIT(logFilePath, msg, terminal)  Logger::init(logFilePath, msg, terminal)  //! Initialize the logger with a file path, logger name, and terminal output option
#define LOG_SHUTDOWN()  Logger::shutdown()  //! Shutdown the logger and close the log file

#define LOG_MACRO_STREAM(LEVEL, STREAM_EXPR) \
    do { \
        Logger::createLogStream(LEVEL) << STREAM_EXPR; \
    } while(0) //! Macro for logging with a stream expression

#define LOG_DEBUG_STREAM(STREAM_EXPR) LOG_MACRO_STREAM(LogLevel::DEBUG, STREAM_EXPR) //! Debug level logging with a stream expression
#define LOG_INFO_STREAM(STREAM_EXPR) LOG_MACRO_STREAM(LogLevel::INFO, STREAM_EXPR) //! Info level logging with a stream expression
#define LOG_WARN_STREAM(STREAM_EXPR) LOG_MACRO_STREAM(LogLevel::WARN, STREAM_EXPR)  //! Warn level logging with a stream expression
#define LOG_ERROR_STREAM(STREAM_EXPR) LOG_MACRO_STREAM(LogLevel::ERROR, STREAM_EXPR)  //! Error level logging with a stream expression
#define LOG_SUCCESS_STREAM(STREAM_EXPR) LOG_MACRO_STREAM(LogLevel::SUCCESS, STREAM_EXPR)  //! Success level logging with a stream expression
#define LOG_FAILURE_STREAM(STREAM_EXPR) LOG_MACRO_STREAM(LogLevel::FAILURE, STREAM_EXPR)  //! Failure level logging with a stream expression

#define LOG_DEBUG(msg) Logger::log(LogLevel::DEBUG, msg) //! Debug level logging with a string
#define LOG_INFO(msg) Logger::log(LogLevel::INFO, msg)  //! Info level logging with a string
#define LOG_WARN(msg) Logger::log(LogLevel::WARN, msg)  //! Warn level logging with a string
#define LOG_ERROR(msg) Logger::log(LogLevel::ERROR, msg)  //! Error level logging with a string
#define LOG_SUCCESS(msg) Logger::log(LogLevel::SUCCESS, msg)  //! Success level logging with a string
#define LOG_FAILURE(msg) Logger::log(LogLevel::FAILURE, msg)  //! Failure level logging with a string


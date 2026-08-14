#ifndef MYBREP_FOUNDATION_LOGGER_H
#define MYBREP_FOUNDATION_LOGGER_H

#include <chrono>
#include <functional>
#include <sstream>
#include <string>
#include <thread>

namespace MyBRep
{
namespace Foundation
{

// 日志等级。
enum class LogLevel
{
    Debug = 0,
    Info,
    Warning,
    Error,
    Fatal
};

// 保存一条完整日志记录。
struct LogRecord
{
    LogLevel level = LogLevel::Info; // 日志等级。
    std::string message; // 日志正文。
    const char* file = nullptr; // 日志产生位置对应的源文件。
    int line = 0; // 日志产生位置对应的源文件行号。
    std::chrono::system_clock::time_point timestamp; // 日志产生时间。
    std::thread::id threadId; // 日志产生线程。
};

using LogSink = std::function<void(const LogRecord&)>;

// 提供线程安全的全局日志入口。
class Logger
{
public:
    /// 日志配置

    // 设置最低输出日志等级。
    static void setMinimumLevel(LogLevel level);

    // 返回当前最低输出日志等级。
    static LogLevel minimumLevel();

    // 判断指定等级当前是否允许输出。
    static bool isEnabled(LogLevel level);

    // 设置自定义日志输出目标，空目标恢复默认输出。
    static void setSink(const LogSink& sink);

    // 恢复默认标准错误输出。
    static void resetSink();

    /// 日志写入

    // 写入一条日志。
    static void write(LogLevel level, const std::string& message, const char* file = nullptr, int line = 0);

    // 返回日志等级名称。
    static const char* levelName(LogLevel level);

private:
    Logger() = delete;
};

}
}

#define MYBREP_LOG(level, expression) \
    do \
    { \
        if (MyBRep::Foundation::Logger::isEnabled(level)) \
        { \
            std::ostringstream myBRepLogStream; \
            myBRepLogStream << expression; \
            MyBRep::Foundation::Logger::write(level, myBRepLogStream.str(), __FILE__, __LINE__); \
        } \
    } while (false)

#define MYBREP_LOG_DEBUG(expression) MYBREP_LOG(MyBRep::Foundation::LogLevel::Debug, expression)
#define MYBREP_LOG_INFO(expression) MYBREP_LOG(MyBRep::Foundation::LogLevel::Info, expression)
#define MYBREP_LOG_WARNING(expression) MYBREP_LOG(MyBRep::Foundation::LogLevel::Warning, expression)
#define MYBREP_LOG_ERROR(expression) MYBREP_LOG(MyBRep::Foundation::LogLevel::Error, expression)
#define MYBREP_LOG_FATAL(expression) MYBREP_LOG(MyBRep::Foundation::LogLevel::Fatal, expression)

#endif // MYBREP_FOUNDATION_LOGGER_H
#pragma once
#ifndef __LOG_H__
#define __LOG_H__

#include <sstream>
#include <string>
#include <stdio.h>

inline std::string NowTime();

enum TLogLevel {ERROR, WARNING, INFO, DEBUG1, DEBUG2, DEBUG3, DEBUG4};

class Log
{
public:
    Log();
    virtual ~Log();
    std::ostringstream& Get(TLogLevel level = INFO);
public:
    static TLogLevel& ReportingLevel();
    static std::string ToString(TLogLevel level);
    static TLogLevel FromString(const std::string& level);
    static void SetLevel(TLogLevel level);
protected:
    std::ostringstream os;
private:
    Log(const Log&);
    Log& operator =(const Log&);

    static TLogLevel reportingLevel;
};

TLogLevel Log::reportingLevel = DEBUG4;

Log::Log() {}

std::ostringstream& Log::Get(TLogLevel level)
{
  os << NowTime();
    os << " " << ToString(level) << ": ";
    os << std::string(level > DEBUG1 ? level - DEBUG1 : 0, '\t');
    return os;
}

Log::~Log()
{
    os << std::endl;
    fprintf(stderr, "%s", os.str().c_str());
    fflush(stderr);
}

void Log::SetLevel(TLogLevel level)
{
    Log::reportingLevel = level;
}

TLogLevel& Log::ReportingLevel()
{
    return Log::reportingLevel;
}

std::string Log::ToString(TLogLevel level)
{
	static const char* const buffer[] = {"ERROR", "WARNING", "INFO", "DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4"};
    return buffer[level];
}

TLogLevel Log::FromString(const std::string& level)
{
    if (level == "DEBUG4")
        return DEBUG4;
    if (level == "DEBUG3")
        return DEBUG3;
    if (level == "DEBUG2")
        return DEBUG2;
    if (level == "DEBUG1")
        return DEBUG1;
    if (level == "INFO")
        return INFO;
    if (level == "WARNING")
        return WARNING;
    if (level == "ERROR")
        return ERROR;
    Log().Get(WARNING) << "Unknown logging level '" << level << "'. Using INFO level as default.";
    return INFO;
}

#define LOGGER(level) \
    if (level > Log::ReportingLevel()) ; \
    else Log().Get(level)

#include <sys/time.h>

inline std::string NowTime()
{
    char buffer[30];
    time_t t;
    time(&t);
    tm r = {0};
    strftime(buffer, sizeof(buffer), "%F %X", localtime_r(&t, &r));
    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[100] = {0};
    std::sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000); 
    return result;
}

#endif //__LOG_H__

#include <sstream>
#include <string>
#include <stdio.h>
#include <iostream.h>
#include <time.h>
#include <frc/Timer.h>

class Log {
 public:
  int LogInit(){
    frc::Timer::Timer	logTimer;
    logTimer.Start();
  }
  int Log(string logLevel, string message,) {
    currentTime = LogInit.logTimer.Get();
    
  }
}

#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

class Timer
{
public:
  void start() { start_time_ = std::chrono::steady_clock::now(); }

  int64_t elapsed_milli_seconds() const
  {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  }

  double elapsed_seconds() const
  {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return std::chrono::duration<double>(elapsed).count();
  }

  std::string elapsed_time() const
  {
    int64_t seconds = elapsed_seconds();
    int64_t hours = seconds / 3600;
    seconds %= 3600;
    int64_t minutes = seconds / 60;
    seconds %= 60;

    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hours << ":" << std::setw(2) << std::setfill('0')
        << minutes << ":" << std::setw(2) << std::setfill('0') << seconds;
    return oss.str();
  }

private:
  std::chrono::steady_clock::time_point start_time_;
};

#endif

#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <ostream>
#include <string>
#include <algorithm>

/*Usage:
  DPO_TIMER process_timer("process 1");
  process_timer.start_timer();
  // do something for processing...
  process_timer.end_timer();
  std::cout << process_timer << std::endl;

  **Output :
  process 1 takes (x) us. 

*/

class TIMER
{
  public:
    // critical message for timer description...
    TIMER(const std::string &timer_name) : timer_name_(timer_name) {}
    virtual ~TIMER() = default;
    TIMER() = delete;

    void start_timer() { start_ = std::chrono::steady_clock::now(); }
    void end_timer() { end_ = std::chrono::steady_clock::now(); }

    // return type of duration.count() is long long
    double duration_timer() const { return std::chrono::duration_cast<std::chrono::microseconds>(end_ - start_).count() * 1e-6; }
    friend std::ostream &operator<<(std::ostream &os, const TIMER &timer);

  private:
    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point end_;
    const std::string timer_name_;
};

inline std::ostream &operator<<(std::ostream &os, const TIMER &timer)
{
    os << timer.timer_name_ << " takes " << std::chrono::duration_cast<std::chrono::microseconds>(timer.end_ - timer.start_).count() * 1e-6 << "s. ";
    return os;
}


inline std::string current_time()
{
    std::time_t result = std::time(nullptr);
    std::string time_str(std::asctime(std::localtime(&result)));

    time_str.erase(std::find(time_str.begin(), time_str.end(), ' '));
    time_str.erase(std::find(time_str.begin(), time_str.end(), ' '));
    time_str.erase(std::find(time_str.begin(), time_str.end(), ' '));
    time_str.erase(std::find(time_str.begin(), time_str.end(), ' '));
    time_str.erase(std::find(time_str.begin(), time_str.end(), ':'));
    time_str.erase(std::find(time_str.begin(), time_str.end(), ':'));
    time_str.erase(std::find(time_str.begin(), time_str.end(), '\n'));
    return time_str;
}

#endif // TIMER_H
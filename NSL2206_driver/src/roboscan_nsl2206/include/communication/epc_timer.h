#ifndef EPC_TIMER_H
#define EPC_TIMER_H

#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>

namespace ComLib
{

class EpcTimer {
public:
  explicit EpcTimer(const std::chrono::milliseconds &period, const std::function<void ()> &func): m_period(period), m_func(func), flag(false), active(false)
  {    
    m_thread = std::thread([this] {
    std::unique_lock<std::mutex> lck(mutex_);
      while(true){
        condVar.wait(lck, [&]{ return flag == true; }  );
        active = true;
        std::this_thread::sleep_for(m_period);
        active = false;
        m_func();
      }
    });
  }

  ~EpcTimer() {
    m_thread.join();    
  }

  void start(const std::chrono::milliseconds &period){
    m_period = period;
    flag = true;
    condVar.notify_one();
  }

  void stop(){
    flag = false;
  }

  bool isActive(){
    return active;
  }


private:
  std::chrono::milliseconds m_period;
  std::function<void ()> m_func;
  std::thread m_thread;
  std::mutex mutex_;
  std::condition_variable condVar;
  std::atomic<bool> flag;
  std::atomic<bool> active;


};

}

#endif

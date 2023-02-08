#ifndef ASYNC_TIMER_H
#define ASYNC_TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <iostream>

using namespace std;
using namespace chrono_literals;
using namespace chrono;
using namespace std::this_thread;

class AsyncTimer {
public:
    AsyncTimer() = default;
    AsyncTimer(function<void()> callable, milliseconds duration)
    {
        cout << "Init Timer" << endl;
        runner = thread([](auto f, auto loop_time){
            for (;;) {
                cout << "Thread is running" << endl;
                if (f == nullptr)
                    break;
                f();
                sleep_for(loop_time);
            }
        }, callable, duration);
    };
private:
    thread runner;
};


#endif //ASYNC_TIMER_H

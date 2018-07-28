// 
// 
// 
#include "Thread.h"

Thread::Thread(void(*callback)(void), unsigned long _interval)
    {
    enabled = true;
    onRun(callback);
    //_cached_next_run = 0;
    last_run = millis();

    ThreadID = (int)this;

#ifdef USE_THREAD_NAMES
    ThreadName = "Thread ";
    ThreadName = ThreadName + ThreadID;
#endif

    setInterval(_interval);
    };

void Thread::runned(unsigned long time)
    {
    // Saves last_run
    last_run = time;
    }

void Thread::setInterval(unsigned long _interval)
    {
    // Save interval
    interval = _interval;
    }

bool Thread::shouldRun(unsigned long time)
    {

    if (time - last_run >= interval)
        {
        return (true && enabled);
        }

    return false;
    }

void Thread::onRun(void(*callback)(void))
    {
    _onRun = callback;
    }

void Thread::run()
    {
    // Update last_run 
    runned();

    if (_onRun != NULL)
        _onRun();

    }


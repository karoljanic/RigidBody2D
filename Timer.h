/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef TIMER_H
#define TIMER_H


// Timer class
class Timer
{
private:
    LARGE_INTEGER frequency;
    LARGE_INTEGER startTime, stopTime, currentTime;

    void Query(LARGE_INTEGER& query);

public:
    // constructor
    Timer()
    {
        QueryPerformanceFrequency(&frequency);

        Start();
        Stop();
    }

    //desctructor
    ~Timer()
    {
    }

    // saves start time to varaiable
    void Start()
    {
        QueryPerformanceCounter(&startTime);
    }

    // saves stop time to variable
    void Stop()
    {
        QueryPerformanceCounter(&stopTime);
    }

    // returns current time = timer.now - timer.start
    float Time()
    {
        QueryPerformanceCounter(&currentTime);
        return (currentTime.QuadPart - startTime.QuadPart) / (float)frequency.QuadPart;
    }

    // returns time elapsed from timer.start to timer.stop timer.stop - timer.start
    float Elapsed()
    {
        return (stopTime.QuadPart - startTime.QuadPart) / (float)frequency.QuadPart;
    }

    // return current clock count
    unsigned long long int Now()
    {
        QueryPerformanceCounter(&currentTime);
        return currentTime.QuadPart;
    }

};

#endif // TIMER_H
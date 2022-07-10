#pragma once
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include "Falcor.h"
using namespace Falcor;

class Timer
{
public:
    typedef std::chrono::high_resolution_clock Clock;
    //typedef std::chrono::milliseconds milliseconds;
    typedef std::chrono::duration<double> duration;

    Clock::time_point t0;
    Clock::time_point t1;
    std::string process;
    void begin(std::string name)
    {
        process = name;
        t0 = Clock::now();
        logInfo(name);
    }

    void end()
    {
        t1 = Clock::now();
        duration time_span = std::chrono::duration_cast<duration>(t1 - t0);
        logInfo(process + " uses time about  " + std::to_string(time_span.count()) + " seconds.");
        logInfo(process + " finished");
        //std::cout << process << " uses time about  " << time_span.count() << " seconds. \n";
        /*milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);
        std::cout << process << " uses time about  " << ms.count() << "ms\n";*/
    }
};

#pragma once

#include <string>
#include <chrono>
#include <boost/shared_ptr.hpp>
#include <fstream>

using std::string;
using std::ofstream;
using Clock = std::chrono::steady_clock;
using std::chrono::time_point;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

class MeasureExecutionTime{
public:
    MeasureExecutionTime(string logfile_name){
        log_file.open(logfile_name + ".log");
    }

    ~MeasureExecutionTime(){
        log_file.close();
    }

    void start(){
        start_point = Clock::now();
    }

    double stop(const char *name){
        stop_point = Clock::now();
        milliseconds diff = duration_cast<milliseconds>(stop_point - start_point);
        double ms = diff.count();
        log_file << name << " " << ms << "ms" << std::endl;
        return ms;
    }

private:
    time_point<Clock> start_point, stop_point;
    ofstream log_file;
};

typedef boost::shared_ptr<MeasureExecutionTime> MeasureExecutionTimePtr;
#ifndef EROSMATH_H
#define EROSMATH_H
#include <stdio.h>
#include <iostream>
#include <math.h>
class Compute_Average
{
public:
     Compute_Average();
    ~Compute_Average();
    void init();
    double compute(double new_measurement);
private:
    uint64_t update_count;
    double mean;

};
/*
class PID
{
public:
    struct PIDStruct {
        double v;
    };
    PID();
    ~PID();
    void init();
    PIDStruct update(double t_command,uint64_t t_command_rxcount,double t_sense,uint64_t t_sense_rxcount);
    PIDStruct get_piddata() { return data; }

private:
    PIDStruct data;
}
*/

#endif
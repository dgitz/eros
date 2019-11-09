#include "../include/eros_math.h"
Compute_Average::Compute_Average()
{

}
Compute_Average::~Compute_Average()
{
}
void Compute_Average::init()
{
    update_count = 0;
    mean = 0;
}
double Compute_Average::compute(double new_measurement)
{
    if(update_count == 0)
	{
		mean = pow(new_measurement,2.0);
	}
	else
	{
		double a = (pow(new_measurement,2.0)/(double)((update_count+1)));
		double b = (mean/(double)(update_count+1));
		mean += a-b;
	}
    double out = pow(mean,0.5);
}
/*
PID::PID()
{

}
PID::~PID()
{

}
PIDStruct PID::update(double t_command,uint64_t t_command_rxcount,double t_sense,uint64_t t_sense_rxcount)
{
	PIDStruct m_data = data;

	data = m_data;
	return data;
}
*/
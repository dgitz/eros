#ifndef POSEHELPER_H
#define POSEHELPER_H
#include <map>
#include <eros/pose.h>
#include "eROS_Definitions.h"
class PoseHelper
{
public:
	const double SIMPLEHEADING_STRING_RESOLUTION_DEG = 22.5;
	const double YAWROTATE_THRESHOLD_DEGS = 2.0;
	const double PITCHROTATE_THRESHOLD_DEGS = 2.0;
	const double ROLLROTATE_THRESHOLD_DEGS = 2.0;
	const double FORWARDVELOCITY_THRESHOLD_MPS = 0.1;
	enum class HeadingReference
	{
		UNKNOWN=0,
		COMMON_YAW=1
	};
	PoseHelper()
	{
		uint8_t index=0;
		double start_angle = -191.25;
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "W"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "WSW"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "SW"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "SSW"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "S"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "SSE"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "SE"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "ESE"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "E"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "ENE"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "NE"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "NNE"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "N"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "NNW"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "NW"; //Angle value is start range
		simple_headingstring_map[start_angle+(double)(index++)*SIMPLEHEADING_STRING_RESOLUTION_DEG] = "WNW"; //Angle value is start range
		
	}
	~PoseHelper()
	{

	}
	std::string get_simplepose_string(eros::pose pose)
	{
		std::string simple_string = "POSE:";
		bool anything_moving = false;
		// Check Yaw Rate
		if(pose.yawrate.value > YAWROTATE_THRESHOLD_DEGS)
		{
			anything_moving = true;
			simple_string += " ROTATING LEFT";
		}
		else if(pose.yawrate.value < -YAWROTATE_THRESHOLD_DEGS)
		{
			anything_moving = true;
			simple_string += " ROTATING RIGHT";
		}

		// Check Pitch Rate
		if(pose.pitchrate.value > PITCHROTATE_THRESHOLD_DEGS)
		{
			anything_moving = true;
			simple_string += " PITCHING FORWARD";
		}
		else if(pose.pitchrate.value < -PITCHROTATE_THRESHOLD_DEGS)
		{
			anything_moving = true;
			simple_string += " PITCHING BACKWARDS";
		}

		// Check Roll Rate
		if(pose.rollrate.value > ROLLROTATE_THRESHOLD_DEGS)
		{
			anything_moving = true;
			simple_string += " ROLLING LEFT";
		}
		else if(pose.rollrate.value < -ROLLROTATE_THRESHOLD_DEGS)
		{
			anything_moving = true;
			simple_string += " ROLLING RIGHT";
		}

		// Check Forward Velocity
		if(pose.forward_velocity.value > FORWARDVELOCITY_THRESHOLD_MPS)
		{
			anything_moving = true;
			simple_string += " DRIVING FORWARDS";
		}
		else if(pose.forward_velocity.value < -FORWARDVELOCITY_THRESHOLD_MPS)
		{
			anything_moving = true;
			simple_string += " DRIVING BACKWARDS";
		}
		if(anything_moving == false)
		{
			simple_string += " STATIONARY.";
		}
		return simple_string;
	}
	// HEADING STUFF
	double get_simple_headingstring_map_resolution() { return SIMPLEHEADING_STRING_RESOLUTION_DEG; }
	std::map<double,std::string> get_simple_headingstring_map() { return simple_headingstring_map; }
	bool is_angleequal(HeadingReference sourcea_heading_ref,double a_deg,HeadingReference sourceb_heading_ref,double b_deg,double precision_deg)
	{
		a_deg = convert_heading_deg(sourcea_heading_ref,HeadingReference::COMMON_YAW,a_deg);
		b_deg = convert_heading_deg(sourceb_heading_ref,HeadingReference::COMMON_YAW,b_deg);
		double d = a_deg-b_deg;
		if(d > 180.0) { d -= 360.0; }
		if(d < -180.0) { d += 360.0; }
		if(fabs(d) < precision_deg)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}
	double wrapAngle(HeadingReference source_heading_ref, double x)
	{
		if(source_heading_ref == HeadingReference::COMMON_YAW)
		{
			x = fmod(x + 180.0,360.0);
			if (x < 0.0)
				x += 360.0;
			return x - 180.0;
		}
		else
		{
			return 0.0;
		}
	}
	double convert_heading_deg(HeadingReference source_heading_ref,HeadingReference target_heading_ref,double angle_deg)
	{
		if(source_heading_ref == target_heading_ref)
		{
			return angle_deg;
		}
		else
		{
			return 0.0;
		}
	}
	std::string compute_heading_simplestring(HeadingReference heading_ref,double angle_deg)
	{
		angle_deg = convert_heading_deg(heading_ref,HeadingReference::COMMON_YAW,angle_deg);
		std::map<double, std::string>::iterator prev_it = simple_headingstring_map.begin();
		std::map<double, std::string>::iterator it = simple_headingstring_map.begin();
		while(it != simple_headingstring_map.end())
		{	
			if(angle_deg  < it->first)
			{
				return prev_it->second;
			}
			prev_it = it;
			it++;
		}
		return "W";

	}
	double get_examplarheading_simplestring(HeadingReference target_heading_ref, std::string direction)
	{
		if(target_heading_ref == HeadingReference::COMMON_YAW)
		{
			std::map<double, std::string>::iterator it = simple_headingstring_map.begin();
			while(it != simple_headingstring_map.end())
			{
				if(it->second == direction)
				{
					return it->first+SIMPLEHEADING_STRING_RESOLUTION_DEG/2.0;
				}
				it++;
			}
		}
		else
		{
			return 0.0;
		}
		return 0.0;
	}
	void print_simple_headingstring_map()
	{
		printf("--- HEADING MAP: REFERENCE: COMMON YAW ---\n");
		std::map<double, std::string>::iterator it = simple_headingstring_map.begin();
		while(it != simple_headingstring_map.end())
		{
			printf("\t%s:\tStart Angle: %4.2f Center Angle: %4.2f End Angle: %4.2f\n",
				it->second.c_str(),
				it->first,
				it->first+SIMPLEHEADING_STRING_RESOLUTION_DEG/2.0,
				it->first+SIMPLEHEADING_STRING_RESOLUTION_DEG);
			it++;
		}
		printf("---\n");
	}
	

private:
	std::map<double,std::string> simple_headingstring_map; // In COMMON YAW REFERENCE
};

#endif

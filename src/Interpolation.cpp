/**
 * \file Interpolation.cpp
 *
 * \brief 	Contains methods for position 
 * 			and angle interpolation.
 *
 * One of the 4 methods needs to be defined in the header 
 * file `Interpolation.h`. The CONTINUOUS_REPRESENTATION 
 * is defined by default.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 09 2018
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath> // std::fmod
#include <tf/transform_listener.h> // tf::Quaternion
#include "spline.h"

#include "Interpolation.h"

#define PI 3.14159265359

#if defined(COMPUTE_DIRECTION)
	int get_interpolated_positions(char* path)
	{
		// load position from file
		std::string line;

		char pos_path[100];
		snprintf(pos_path, sizeof pos_path, "%s/positions.txt", path);

		std::ifstream position_file (pos_path);
		if( ! position_file )	
		{
			std::cout << "Error opening file for reading\n" << std::endl ;
			return -1 ;
		}

		std::vector<double> sec_vec;
		std::vector<double> pos_x_vec;
		std::vector<double> pos_y_vec;

		double sec, pos_x, pos_y, yaw;
		while (position_file >> sec >> pos_x >> pos_y >> yaw)
		{
			sec_vec.push_back(sec);
			pos_x_vec.push_back(pos_x);
			pos_y_vec.push_back(pos_y);
		}
		position_file.close();

		// interpolate
		tk::spline s_x;
		tk::spline s_y;
		s_x.set_points(sec_vec,pos_x_vec);
		s_y.set_points(sec_vec,pos_y_vec);

		// write into a file
		char interpolated_positions_path[100];
		snprintf(interpolated_positions_path, sizeof interpolated_positions_path, "%s/interpolated_positions.txt", path);

		std::ofstream interpolated_positions_file(interpolated_positions_path);
		if( ! interpolated_positions_file )	
		{
			std::cout << "Error opening file for writing\n" << std::endl ;
			return -1 ;
		}

		for ( int i = sec_vec[0]; i<sec_vec[sec_vec.size()-1];i++)
		{	
			interpolated_positions_file <<  i << " " << s_x(double(i)) << " " << s_y(double(i)) << " " << atan2(s_y(double(i+1)) - s_y(double(i)), s_x(double(i+1)) - s_x(double(i))) << std::endl;
		}

		return 0;
	}

#elif defined(QUATERNION_INTERPOLATION)
	int get_interpolated_positions(char* path)
	{
		// load position from file
		std::string line;

		char pos_path[100];
		snprintf(pos_path, sizeof pos_path, "%s/positions.txt", path);

		std::ifstream position_file (pos_path);
		if( ! position_file )	
		{
			std::cout << "Error opening file for reading\n" << std::endl ;
			return -1 ;
		}

		std::vector<double> sec_vec;
		std::vector<double> pos_x_vec;
		std::vector<double> pos_y_vec;
		std::vector<double> q_z_vec;
		std::vector<double> q_w_vec;

		double sec, pos_x, pos_y, yaw;
		while (position_file >> sec >> pos_x >> pos_y >> yaw)
		{
			sec_vec.push_back(sec);
			pos_x_vec.push_back(pos_x);
			pos_y_vec.push_back(pos_y);
			tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
			q_z_vec.push_back(q[2]);
			q_w_vec.push_back(q[3]);
		}
		position_file.close();

		// interpolate
		tk::spline s_x;
		tk::spline s_y;
		tk::spline s_qz;
		tk::spline s_qw;
		s_x.set_points(sec_vec,pos_x_vec);
		s_y.set_points(sec_vec,pos_y_vec);
		s_qz.set_points(sec_vec,q_z_vec);
		s_qw.set_points(sec_vec,q_w_vec);

		// write into a file
		char interpolated_positions_path[100];
		snprintf(interpolated_positions_path, sizeof interpolated_positions_path, "%s/interpolated_positions.txt", path);

		std::ofstream interpolated_positions_file(interpolated_positions_path);
		if( ! interpolated_positions_file )	
		{
			std::cout << "Error opening file for writing\n" << std::endl ;
			return -1 ;
		}

		for ( int i = 0; i<sec_vec[sec_vec.size()-1];i++)
		{	
			tf::Quaternion q(0,0,s_qz(double(i)),s_qw(double(i)));

			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			interpolated_positions_file <<  i << " " << s_x(double(i)) << " " << s_y(double(i)) << " " << yaw << std::endl;
		}

		return 0;
	}

#elif defined(CONTINUOUS_REPRESENTATION)
	int rotations = 0;

	int rotational_overflow(double value, double prev_value)
	{
		if ( abs(value-prev_value)>PI && (value >= 0) && (prev_value < 0) )
		{
			if ( value < 1 )
			{
				return 1; //left turn
			} else {
				return -1; //right turn
			}
		} else {

			if ( abs(value-prev_value)>PI && (value < 0) && (prev_value >= 0) )
			{
				if( prev_value < 1 )
				{
					return -1; //right turn
				} else {
					return 1; //left turn
				}
			}
		}

		return 0; //not 180 deg turn
	}

	double convert_euler2continuous(double value, double previous_value)
	{
		int rot = rotational_overflow(value, previous_value);
		rotations += rot;

		if ( rotations > 0 ) //left rotation
		{
			return 2*rotations*PI+value;
		} else {
			if ( rotations < -1 ) //right rotation
			{
				return 2*rotations*PI+value;
			} else {
				return value;
			}
		}
	}

	double convert_continuous2euler(double value)
	{
		if ( value >= 0 ) // left rotation
		{
			if ( std::fmod(value,2*PI) > PI )
			{
				return -2*PI+std::fmod(value,2*PI);
			} else {
				return std::fmod(value,2*PI);
			}
		} else { //right rotation
			if ( std::fmod(value,2*PI) < -PI )
			{
				return 2*PI+std::fmod(value,2*PI);
			} else {
				return std::fmod(value,2*PI);
			}

		}
		return 0;
	}

	int get_interpolated_positions(char* path)
	{
		// load position from file
		std::string line;

		char pos_path[100];
		snprintf(pos_path, sizeof pos_path, "%s/positions.txt", path);

		std::ifstream position_file (pos_path);
		if( ! position_file )	
		{
			std::cout << "Error opening file for reading\n" << std::endl ;
			return -1 ;
		}

		std::vector<double> sec_vec;
		std::vector<double> pos_x_vec;
		std::vector<double> pos_y_vec;
		std::vector<double> yaw_vec;
		std::vector<double> yaw_converted_vec;

		double sec, pos_x, pos_y, yaw;
		while (position_file >> sec >> pos_x >> pos_y >> yaw)
		{
			sec_vec.push_back(sec);
			pos_x_vec.push_back(pos_x);
			pos_y_vec.push_back(pos_y);
			yaw_vec.push_back(yaw);
			if(yaw_vec.size()>1)
			{
				yaw_converted_vec.push_back(convert_euler2continuous(yaw, yaw_vec[yaw_converted_vec.size()-1]));
			}else {
				yaw_converted_vec.push_back(yaw);
			}
			
		}
		position_file.close();

		// interpolate
		tk::spline s_x;
		tk::spline s_y;
		tk::spline s_yaw;
		s_x.set_points(sec_vec,pos_x_vec);
		s_y.set_points(sec_vec,pos_y_vec);
		s_yaw.set_points(sec_vec,yaw_converted_vec);

		// write into a file
		char interpolated_positions_path[100];
		snprintf(interpolated_positions_path, sizeof interpolated_positions_path, "%s/interpolated_positions.txt", path);

		std::ofstream interpolated_positions_file(interpolated_positions_path);
		if( ! interpolated_positions_file )	
		{
			std::cout << "Error opening file for writing\n" << std::endl ;
			return -1 ;
		}

		for ( int i = 0; i<sec_vec[sec_vec.size()-1];i++)
		{	
			interpolated_positions_file <<  i << " " << s_x(double(i)) << " " << s_y(double(i)) << " " << convert_continuous2euler(s_yaw(double(i))) << std::endl;
		}

		return 0;
	}

#elif defined(SINUS_REPRESENTATION)

	int get_interpolated_positions(char* path)
	{
		// load position from file
		std::string line;

		char pos_path[100];
		snprintf(pos_path, sizeof pos_path, "%s/positions.txt", path);

		std::ifstream position_file (pos_path);
		if( ! position_file )	
		{
			std::cout << "Error opening file for reading\n" << std::endl ;
			return -1 ;
		}

		std::vector<double> sec_vec;
		std::vector<double> pos_x_vec;
		std::vector<double> pos_y_vec;
		std::vector<double> sin_vec;
		std::vector<double> cos_vec;

		double sec, pos_x, pos_y, yaw;
		while (position_file >> sec >> pos_x >> pos_y >> yaw)
		{
			sec_vec.push_back(sec);
			pos_x_vec.push_back(pos_x);
			pos_y_vec.push_back(pos_y);
			sin_vec.push_back(sin(yaw));
			cos_vec.push_back(cos(yaw));
			
			
		}
		position_file.close();

		// interpolate
		tk::spline s_x;
		tk::spline s_y;
		tk::spline s_sin;
		tk::spline s_cos;
		s_x.set_points(sec_vec,pos_x_vec);
		s_y.set_points(sec_vec,pos_y_vec);
		s_sin.set_points(sec_vec,sin_vec);
		s_cos.set_points(sec_vec,cos_vec);

		// write into a file
		char interpolated_positions_path[100];
		snprintf(interpolated_positions_path, sizeof interpolated_positions_path, "%s/interpolated_positions.txt", path);

		std::ofstream interpolated_positions_file(interpolated_positions_path);
		if( ! interpolated_positions_file )	
		{
			std::cout << "Error opening file for writing\n" << std::endl ;
			return -1 ;
		}

		for ( int i = 0; i<sec_vec[sec_vec.size()-1];i++)
		{	
			interpolated_positions_file <<  i << " " << s_x(double(i)) << " " << s_y(double(i)) << " " << atan2(s_sin(double(i)), s_cos(double(i+1))) << std::endl;
		}

		return 0;
	}
#else 

	int get_interpolated_positions(char* path)
	{
		std::cout << "ERROR: No interpolation method defined" << std::endl;
		return -1;
	}

#endif
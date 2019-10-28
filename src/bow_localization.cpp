/**
 * \file bow_localization.cpp
 *
 * \brief 	Testing BoW localization method.
 *
 * \param 1 path to test dataset bow descriptor file
 * \param 2 path to train dataset bow descriptor file
 * \param 3 output path for position estimation file
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 18 2018
 */

#include "ros/ros.h"
#include <fstream>
#include "Utils.h"
#include "Data_processing.h"

int main(int argc, char **argv) 
{
	std::ofstream test_results_file(argv[3]);
	if( ! test_results_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}


	std::ifstream descriptor_file (argv[1]);
	if( ! descriptor_file )	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	} 

	std::vector<std::string> vec;
	std::string row;
	std::string element;
	
	std::vector<float> descriptor;
	
	
	while( std::getline(descriptor_file,row) )
	{
		vec.clear();
		descriptor.clear();

		std::stringstream ss ( row );
		while ( getline( ss, element, ' ') ) 
		{
			vec.push_back(element);
		}
		
		for (int i = 3; i<vec.size();i++)
		{
			descriptor.push_back(std::strtof(vec[i].c_str(),0));
		}

		//get score for test data
		std::stringstream result;
		find_high_scoring_position(descriptor, argv[2], &result);

		test_results_file << vec[0] << " " << vec[1] << " " << vec[2] << " " << result.str() << std::endl;

	}

	descriptor_file.close();

	test_results_file.close();

	return 0;


}

/**
 * \file naive_localization.cpp
 *
 * \brief Demonstrates naive localization method with 
 *		  descriptors consisting of raw image pixels.
 * 
 * \param 1 path to test dataset naive descriptor file
 * \param 2 path to train dataset naive descriptor file
 * \param 3 output path for position estimation file
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: February 23 2018
 */

#include <fstream>
#include "Utils.h"
#include "Interpolation.h"
#include "Data_processing.h"


int main(int argc, char **argv)
{
	// check if arguments ok
	if ( ! (argc>3 && file_exists(argv[1]) && file_exists(argv[2]))  )
	{
		std::cout << "Error: missing arguments or invalid file path.\n";
		return -1;
	}

	std::ofstream test_results_file( argv[3] );
	if( ! test_results_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}


	// read the descriptor file
	std::ifstream descriptor_file ( argv[1] );
	if( ! descriptor_file )	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	} 

	std::vector<std::string> vec;
	std::string row;
	std::string element;
	
	std::vector<unsigned int> descriptor;
	
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
			descriptor.push_back(std::strtoul(vec[i].c_str(),0,10));
		}

		//get score for test data
		std::stringstream result;
		find_high_scoring_position(descriptor, argv[2], &result);

		test_results_file << vec[0] << " " << vec[1] << " " << vec[2] << " " << result.str() << std::endl;

	}

	return 0;
}
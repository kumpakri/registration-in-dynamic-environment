/**
 * \file make_naive_descriptor_file.cpp
 *
 * \brief Creates naive descriptor file.
 * 
 * \param 1 path to positions file
 * \param 2 path to image folder
 * \param 3 output path for descriptors file
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: July 01 2018
 */

#include <opencv2/opencv.hpp>
#include "Data_processing.h"
#include "Utils.h"

int main(int argc, char **argv)
{

	// check if arguments ok
	if ( ! (argc>3 && file_exists(argv[1]) && file_exists(argv[2]))  )
	{
		std::cout << "Error: missing arguments or invalid file path.\n";
		return -1;
	}

	make_naive_descriptor_file(argv[1], argv[2], argv[3]);

	return 0;
}
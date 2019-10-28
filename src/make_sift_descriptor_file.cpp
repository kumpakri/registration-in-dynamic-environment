/**
 * \file make_sift_descriptor_file.cpp
 *
 * \brief Creates SIFT descriptor file.
 * 
 * \param 1 nFeatures
 * \param 2 nOctaveLayers
 * \param 3 contrastThreshold
 * \param 4 edgeThreshold
 * \param 5 sigma
 * \param 6 path to positions file
 * \param 7 path to image folder
 * \param 8 output path for used positions file
 * \param 9 output path for descriptors file
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: July 02 2018
 */

#include <opencv2/opencv.hpp>
#include "Data_processing.h"
#include "Utils.h"

int main(int argc, char **argv)
{

	// check if arguments ok
	if ( ! (argc>9 && file_exists(argv[6]) && file_exists(argv[7]) )  )
	{
		std::cout << "Error: missing arguments or invalid file path.\n";
		return -1;
	}

	//SIFT detector setting
	float nFeatures = atof(argv[1]);
	float nOctaveLayers = atof(argv[2]);
	float contrastThreshold = atof(argv[3]);
	float edgeThreshold = atof(argv[4]);
	float sigma = atof(argv[5]);

	cv::Mat all_descriptors; 
	cv::FeatureDetector* detector;
	detector = new cv::SiftFeatureDetector( nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

	make_descriptor_file(argv[6], argv[7], argv[9], argv[8], &all_descriptors, detector);

	return 0;
}
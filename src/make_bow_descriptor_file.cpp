/**
 * \file make_bow_descriptor_file.cpp
 *
 * \brief Creates BoW descriptor file.
 * 
 * \param 1 nFeatures
 * \param 2 nOctaveLayers
 * \param 3 contrastThreshold
 * \param 4 edgeThreshold
 * \param 5 sigma
 * \param 6 path to train dataset SIFT descriptor file
 * \param 7 path to positions file
 * \param 8 path to image folder
 * \param 9 output path for descriptors file
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: July 02 2018
 */

#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include "Data_processing.h"
#include "Utils.h"

int main(int argc, char **argv)
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	// check if arguments ok
	if ( ! (argc>9 )  )
	{
		std::cout << "Error: missing arguments.\n";
		return -1;
	}

	if ( ! (file_exists(argv[6]) && file_exists(argv[7]) && file_exists(argv[8]) )  )
	{
		std::cout << "Error: invalid file path.\n";
		return -1;
	}

	//SIFT detector setting
	int nFeatures = atoi(argv[1]);
	int nOctaveLayers = atoi(argv[2]);
	float contrastThreshold = atof(argv[3]);
	float edgeThreshold = atof(argv[4]);
	float sigma = atof(argv[5]);

	int dictionarySize = 1000; // < number of descriptors
	cv::TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
	int retries=1;
	int flags=cv::KMEANS_PP_CENTERS;
	cv::BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);

	cv::Mat temp;
	cv::FileStorage descriptor_file(argv[6],cv::FileStorage::READ);
	descriptor_file["matDescriptors"] >> temp;

	// cutting the id column (id of the corresponding image)
	bowTrainer.add(temp(cv::Rect(1,0,temp.cols-1, temp.rows)));
	descriptor_file.release();

	cv::DescriptorExtractor* extractor;
	extractor = new cv::SiftDescriptorExtractor();	

	cv::DescriptorMatcher* matcher;
	matcher = new cv::BFMatcher();//(cv::NORM_L2,false); 

	cv::BOWImgDescriptorExtractor bowDE(extractor,matcher);
	bowDE.setVocabulary(bowTrainer.cluster());

	make_BoW_descriptor_file(argv[7], argv[8], argv[9], &bowDE, nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

	return 0;
}
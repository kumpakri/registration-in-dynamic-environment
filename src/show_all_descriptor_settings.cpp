/**
 * \file show_all_descriptor_settings.cpp
 *
 * \brief 	Shows matches obtained by using 
 *   		different feature detectors, descriptor 
 *			extractors and matchers.
 *
 * \param 1 source image
 * \param 2 target image
 * \param 3 image pair ID, a string appearing at the beggining 
 *			of the created maches image (may define a path)
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 28 2018
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp> 

#include "Utils.h"


int main(int argc, char **argv) 
{

	//SIFT detector setting
	float nFeatures = 0;
	float nOctaveLayers = 4;
	float contrastThreshold = .03;
	float edgeThreshold = 3;
	float sigma = 1.6;

	//SURF detector setting
	double 	hessianThreshold = 400.;
	int  	octaves = 4;
	int 	octaveLayers = 7;

	//STAR detector setting
	int  	maxSize = 30;
	int  	responseThreshold = 5;
	int  	lineThresholdProjected = 16;
	int  	lineThresholdBinarized = 25;
	int  	suppressNonmaxSize = 8;

	//MSER detector setting
	int  	delta = 5;
	int  	minArea = 40;
	int  	maxArea = 14400;
	double  maxVariation = 1;
	double  minDiversity = .1;
	int  	maxEvolution = 300;
	double  areaThreshold = 1.01;
	double  minMargin = .003;
	int  	edgeBlurSize = 3;

	//FAST detector setting
	int  	threshold = 20;
	bool  	nonmaxSuppression = false;


	
// ==== DETECTORS =======================

	const int number_of_detectors = 5;
	cv::FeatureDetector* SIFTdetector, *SURFdetector, *STARdetector, *MSERdetector, *FASTdetector;

	SIFTdetector = new cv::SiftFeatureDetector( nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);


	SURFdetector = new cv::SurfFeatureDetector( hessianThreshold, octaves, octaveLayers	);

	STARdetector = new cv::StarFeatureDetector( maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize);


	MSERdetector = new cv::MserFeatureDetector( delta, minArea, maxArea, maxVariation, minDiversity, maxEvolution, areaThreshold, minMargin,edgeBlurSize );

	FASTdetector = new cv::FastFeatureDetector( threshold, nonmaxSuppression );

	cv::FeatureDetector* detectors[number_of_detectors] = {SIFTdetector, SURFdetector, STARdetector, MSERdetector, FASTdetector};

	char* detector_names[number_of_detectors] = {"SIFT", "SURF", "STAR", "MSER", "FAST"};

// ==== EXTRACTORS ======================
// NOTE! in case of re-ordering extractors incompatibilities might occur.

	const int number_of_extractors = 3;
	cv::DescriptorExtractor* SIFTextractor, *SURFextractor, *BRIEFextractor;

	SIFTextractor = new cv::SiftDescriptorExtractor();

	SURFextractor = new cv::SurfDescriptorExtractor();

	BRIEFextractor = new cv::BriefDescriptorExtractor();

	cv::DescriptorExtractor* extractors[number_of_extractors] = {SIFTextractor, SURFextractor,BRIEFextractor};

	char* extractor_names[number_of_extractors] = {"SIFT", "SURF", "BRIEF"};

// ==== MATCHERS ========================
// NOTE! in case of re-ordering matchers incompatibilities might occur.

	const int number_of_matchers = 2;
	cv::DescriptorMatcher* FLANNmatcher, *BFmatcher;

	FLANNmatcher = new cv::FlannBasedMatcher(); 
	//! FLANNmatcher2 must be used in case of BRIEF descriptor extractor and FLANN matcher combination
	cv::FlannBasedMatcher FLANNmatcher2(new cv::flann::LshIndexParams(20,10,2));
	BFmatcher = new cv::BFMatcher();

	cv::DescriptorMatcher * matchers[number_of_matchers] = { BFmatcher, FLANNmatcher };
	char* matcher_names[number_of_matchers] = { "BruteForce", "FLANN" };

// ======================================

	cv::Mat image1, image2, image_result;

	// check if arguments ok
	if ( argc>2 && file_exists(argv[1]) && file_exists(argv[2]) )
	{
		image1 = cv::imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
 		image2 = cv::imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );
	} else {
		std::cout << "Error: missing arguments or invalid file path.\n";
		return -1;
	}
	

 	std::vector<cv::KeyPoint> keypoints1, keypoints2;
 	std::vector<cv::DMatch> matches, best_matches;
 	cv::Mat descriptors1, descriptors2;

 	for ( int detector_i = 0; detector_i < number_of_detectors; detector_i++)
 	{
 		for ( int extractor_i = 0; extractor_i<number_of_extractors; extractor_i++ )
 		{
 			for ( int matcher_i = 0; matcher_i<number_of_matchers; matcher_i++)
 			{
 				char window_name[100];

 				// check if argv[3] is empty
 				char* id_string;
 				if (argc > 3)
 				{
 					id_string = argv[3];
 				} else {
 					id_string = "matches";
 				}

				snprintf(window_name, sizeof(window_name)*sizeof(char*), "%s_detector_%s_extractor_%s_matcher.png", detector_names[detector_i], extractor_names[extractor_i], matcher_names[matcher_i]);
				std::cout << window_name <<std::endl;

				snprintf(window_name, sizeof(window_name)*sizeof(char*), "%s_%s_detector_%s_extractor_%s_matcher.png", id_string, detector_names[detector_i], extractor_names[extractor_i], matcher_names[matcher_i]);

				detectors[detector_i]->detect( image1, keypoints1 );
		 		detectors[detector_i]->detect( image2, keypoints2 );

		 		std::cout << "Number of keypoints: " << keypoints1.size() << ", " << keypoints2.size() << std::endl;

		 		//cv::drawKeypoints( image1, keypoints1, image_result );

		 		extractors[extractor_i]->compute( image1, keypoints1, descriptors1);
		 		extractors[extractor_i]->compute( image2, keypoints2, descriptors2);

		 		std::cout << "Descriptor size: " << descriptors1.cols << std::endl;

 				if (extractor_i==2 && matcher_i==1){
 					//BRIEF descriptor and FLANN matcher
 					// FLANN default L2
 					FLANNmatcher2.match(descriptors1, descriptors2, matches);
 				} else {
 					matchers[matcher_i]->match(descriptors1, descriptors2, matches);
 				}

	 			cv::drawMatches( image1, keypoints1, image2, keypoints2,
                 matches, image_result, cv::Scalar::all(-1), cv::Scalar::all(-1),
std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	 			cv::imwrite( window_name,image_result );
//		 		cv::imshow( window_name, image_result );
//		 		cv::waitKey(0);
	
 			}
 		}
 	}
	
	return 0;
}

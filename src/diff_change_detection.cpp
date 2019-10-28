/**
 * \file diff_change_detection.cpp
 *
 * \brief Demonstrates difference image change detection.
 *		  Call from terminal.
 *
 * \param 1 source image
 * \param 2 target image
 * \param 3 threshold for mask
 * \param 4 output file name (difference image)
 * \param 5 output file name (image - mask)
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: April 12 2018
 */

#include <opencv2/opencv.hpp>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp> 

#include "Utils.h"


int main(int argc, char **argv) 
{

	// check if arguments ok
	if ( ! (argc>5 )  )
	{
		std::cout << "Error: missing arguments.\n";
		return -1;
	}

	if ( ! ( file_exists(argv[1]) && file_exists(argv[2]) )  )
	{
		std::cout << "Error: invalid file path.\n";
		return -1;
	}

	cv::FeatureDetector* detector;
	detector = new cv::FastFeatureDetector( 13, true );

	cv::DescriptorExtractor* SIFTextractor;
	SIFTextractor = new cv::SiftDescriptorExtractor();

	cv::DescriptorMatcher *BFmatcher;
	BFmatcher = new cv::BFMatcher();

	cv::Mat img_src, img_dst, img_tf;

	img_src = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
	img_dst = cv::imread(argv[2],CV_LOAD_IMAGE_GRAYSCALE);

	cv::resize(img_src, img_src, cv::Size(), .5, .5, CV_INTER_AREA );
	cv::resize(img_dst, img_dst, cv::Size(), .5, .5, CV_INTER_AREA );

	cv::normalize(img_dst, img_dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::normalize(img_src, img_src, 0, 255, cv::NORM_MINMAX, CV_8UC1);


	std::vector<cv::KeyPoint> keypoints1, keypoints2;

	detector->detect( img_src, keypoints1 );
	detector->detect( img_dst, keypoints2 );

	if ( keypoints1.size() < 5 || keypoints2.size() < 5 ) {
		std::cout << "Couldn't find enough keypoints.\n";
		return -1;
	}

	cv::Mat descriptors1, descriptors2;

	SIFTextractor->compute( img_src, keypoints1, descriptors1 );
	SIFTextractor->compute( img_dst, keypoints2, descriptors2 );

	std::vector<cv::DMatch> matches;

	BFmatcher->match( descriptors1, descriptors2, matches );

	std::sort(matches.begin(), matches.end());

	std::vector<cv::Point2f> pts_src, pts_dst;

	for( int i = 0; i<matches.size(); i++)
	{
		pts_src.push_back(keypoints1[matches[i].queryIdx].pt);
		pts_dst.push_back(keypoints2[matches[i].trainIdx].pt);
	}

	cv::Mat mask;
	cv::Mat h = cv::findHomography(pts_src,pts_dst, CV_RANSAC, 3, mask);

	cv::GaussianBlur(img_dst, img_dst, cv::Size(5,5), 0 );

	cv::warpPerspective(img_src, img_tf, h, img_src.size());

	cv::Mat img_mask = cv::Mat::ones(img_src.size(), CV_8UC1);
	cv::warpPerspective(img_mask, img_mask, h, img_mask.size());

	cv::multiply(img_dst, img_mask, img_dst);

	cv::Mat img_diff, img_diff_mask;
	cv::normalize(img_tf, img_tf, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	
	cv::absdiff(img_dst, img_tf, img_diff);
	cv::normalize(img_diff, img_diff, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	cv::GaussianBlur(img_diff, img_diff, cv::Size(15,15), 0 ); // must be odd
	cv::imwrite(argv[4],img_diff);

	int threshold = std::atoi(argv[3]); //90
	cv::threshold(img_diff, img_diff_mask, threshold, 255, cv::THRESH_BINARY); //125

	cv::imwrite(argv[5],img_diff_mask);
	
	return 0;
}
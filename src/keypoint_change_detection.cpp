/**
 * \file keypoint_change_detection.cpp
 *
 * \brief Demonstrates keypoint change detection.
 * 		  Call from terminal.
 *
 * \param 1 source image
 * \param 2 target image
 * \param 3 output file name (image)
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: April 29 2018
 */

#include "Utils.h"

int main(int argc, char **argv) 
{

	// check if arguments ok
	if ( ! (argc>3 )  )
	{
		std::cout << "Error: missing arguments.\n";
		return -1;
	}

	if ( !(file_exists(argv[1]) && file_exists(argv[2]) )  )
	{
		std::cout << "Error: invalid file path.\n";
		return -1;
	}

	cv::Mat img_src = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat img_dst = cv::imread(argv[2],CV_LOAD_IMAGE_GRAYSCALE);

	//-----------------------------------------------------
	
	cv::resize(img_src, img_src, cv::Size(), .5, .5, CV_INTER_AREA );
	cv::resize(img_dst, img_dst, cv::Size(), .5, .5, CV_INTER_AREA );

	cv::normalize(img_src, img_src, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::normalize(img_dst, img_dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	cv::GaussianBlur(img_dst, img_dst, cv::Size(5,5), 0 );
	cv::GaussianBlur(img_src, img_src, cv::Size(5,5), 0 );

	cv::FeatureDetector *detector;
	
	detector = new cv::FastFeatureDetector( 13, true );
	
	std::vector<cv::KeyPoint> keypoints_src, keypoints_dst;
	detector->detect( img_src, keypoints_src );
	detector->detect( img_dst, keypoints_dst );

	if ( keypoints_src.size() < 5 || keypoints_dst.size() < 5 ) {
		std::cout << "Couldn't find enough keypoints.\n";
		return -1;
	}

	cv::DescriptorExtractor* extractor;
	extractor = new cv::SiftDescriptorExtractor();

	cv::Mat descriptors_src, descriptors_dst;
	extractor->compute(img_src, keypoints_src, descriptors_src);
	extractor->compute(img_dst, keypoints_dst, descriptors_dst);
	
	cv::DescriptorMatcher *matcher;
	matcher = new cv::BFMatcher();

	std::vector<cv::DMatch> matches;
	matcher->match(descriptors_src, descriptors_dst, matches);

	std::sort(matches.begin(), matches.end());

	std::vector<cv::Point2f> pts_src, pts_dst, pts_tf;
	for( int i = 0; i<matches.size(); i++)
	{
		pts_src.push_back(keypoints_src[matches[i].queryIdx].pt);
		pts_dst.push_back(keypoints_dst[matches[i].trainIdx].pt);
	}

	cv::Mat mask;
	cv::Mat h = cv::findHomography(pts_src,pts_dst, CV_RANSAC, 3, mask);

	//--------------------------------------------------

	cv::Mat img_tf_s2d, img_tf_tmp, img_tf_d2s;
	cv::warpPerspective(img_src, img_tf_s2d, h, img_src.size());
	cv::warpPerspective(img_dst, img_tf_d2s, h.inv(), img_src.size());

	cv::Mat descriptors_tf_s2d, descriptors_tf_d2s;
	extractor->compute(img_tf_s2d, keypoints_dst, descriptors_tf_s2d);
	extractor->compute(img_tf_d2s, keypoints_src, descriptors_tf_d2s);

	pts_dst.clear();
	pts_src.clear();
	cv::KeyPoint::convert(keypoints_dst, pts_dst);
	cv::KeyPoint::convert(keypoints_src, pts_src);

	img_tf_s2d.copyTo(img_tf_tmp);
	//compare descr.
	std::vector<cv::Point2f> points_tf_s2d, points_tf_d2s, points_dst, points_src;
	cv::perspectiveTransform(pts_dst, points_tf_s2d, h.inv());
	cv::perspectiveTransform(pts_src, points_tf_d2s, h);

	for(int i = 0; i < descriptors_tf_s2d.rows; i++ )
	{

		if( points_tf_s2d.at(i).x > 0 && points_tf_s2d.at(i).y > 0 && points_tf_s2d.at(i).x < img_src.cols && points_tf_s2d.at(i).y < img_src.rows )
		{
			cv::circle(img_tf_tmp, pts_dst.at(i), (int)(cv::norm(descriptors_tf_s2d.row(i),descriptors_dst.row(i),cv::NORM_L2)/50), cv::Scalar(255,0,0),2,8,0);
		}
	}

	for(int i = 0; i < descriptors_tf_d2s.rows; i++ )
	{
		if( pts_src.at(i).x > 0 && pts_src.at(i).y > 0 && pts_src.at(i).x < img_dst.cols && pts_src.at(i).y < img_dst.rows )
		{
			cv::circle(img_tf_tmp, points_tf_d2s.at(i), (int)(cv::norm(descriptors_tf_d2s.row(i),descriptors_src.row(i),cv::NORM_L2)/50), cv::Scalar(255,0,0),2,8,0);
		}

	}

	cv::imwrite(argv[3], img_tf_tmp);
	cv::waitKey();

	return 0;
}
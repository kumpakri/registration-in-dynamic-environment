/**
 * \file Manual_keypoints.h
 *
 * \brief 	Declares methods for manual keypoint
 *			selection.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: May 22 2018
 */
#ifndef MANUAL_KEYPOINTS_H
#define MANUAL_KEYPOINTS_H

#include <opencv2/opencv.hpp>

/**
* \brief Allows to preciesly select keypoints in the images in
* two steps 
*	- rough selection of the keypoint and 
*	- precise selection on zoomed in area of the image.
*/
int get_manual_keypoints(cv::Mat img_src, cv::Mat img_dst, std::vector<cv::Point2f>* keypoints_src, std::vector<cv::Point2f>* keypoints_dst);

int get_manual_keypoints(cv::Mat img_src, cv::Mat img_dst, std::vector<cv::Point2f>* keypoints_src, std::vector<cv::Point2f>* keypoints_dst, cv::Mat* kp_img_src, cv::Mat* kp_img_dst );

/**
* \brief Saves <cv::Point2f> vector into the file.
*
* \param path 		path to a file
* \param keypoints 	points to save
*/
int save_kp(char* path, std::vector<cv::Point2f> keypoints);

#endif //MANUAL_KEYPOINTS_H
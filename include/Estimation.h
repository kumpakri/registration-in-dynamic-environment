/**
 * \file Estimation.h
 *
 * \brief 	Declares methods for position estimation and 
 * 			supportive functions.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: May 22 2018
 */
#ifndef ESTIMATION_H
#define ESTIMATION_H

#include <opencv2/nonfree/nonfree.hpp> 
#include <opencv2/opencv.hpp>


#define KEYPOINT_TF_WEIGHTS 0 
#define GLOBAL_REF 1
#define PAST_DISTANCE 2
#define VECTOR_NORMALIZATION 3
#define IMG_TF_WEIGHTS 4

/**
* \brief Writes true and the estimated position into a result_file. Based on matched descriptors. Without weighting.
*/
int get_position_estimate(
	cv::Mat db_descriptors_all, 
	cv::Mat query_descriptors_all, 
	cv::Mat db_positions, 
	cv::Mat query_positions, 
	std::ofstream* result_file);

/**
* \brief Writes true and the estimated position into a result_file. Based on transformed keypoints. Without weighting.
*/
int get_position_estimate_tf_kp(
	char* path_to_positions_file, 
	char* path_to_image_folder,
	std::vector< cv::Mat > descriptors_db,
	std::vector< cv::Mat > positions_db,
	std::vector< std::vector< std::vector<cv::KeyPoint> > > keypoints_db,
	cv::FeatureDetector *detector,
	std::ofstream* result_file);

/**
* \brief Gets the estimated position by the weighted method and writes them into a result_file with the true position values.
* 
* \param method_type: 
* 	- KEYPOINT_TF_WEIGHTS	- keypoint transformation method
* 	- GLOBAL_REF 			- global reference value method
* 	- PAST_DISTANCE 		- past distance subtraction method
* 	- VECTOR_NORMALIZATION - distance vector normalization method
* 	- IMG_TF_WEIGHTS 		- image transformation method
*/
int get_position_estimate_weighted(
	int method_type,                          
	char* path_to_positions_file, 
	char* path_to_image_folder,
	std::vector< cv::Mat > descriptors_db,
	std::vector< cv::Mat > positions_db,
	std::vector< std::vector< std::vector<cv::KeyPoint> > > keypoints_db,
	std::vector< std::vector< std::vector<float> > >* weights,
	std::vector< std::vector< std::vector<float> > > *distances_past,
	float gamma,
	float ref_val,
	float alpha,
	cv::FeatureDetector *detector,
	std::ofstream* result_file
	);

#endif //ESTIMATION_H
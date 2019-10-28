/**
 * \file Data_processing.h
 *
 * \brief 	Declares functions for data processing
 *			and data file creation.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 09 2018
 */
#ifndef DATA_PROCESSING_H
#define DATA_PROCESSING_H

/**
* \brief Returns a trained BoW dictionary.
*
* \param descriptor_files array of file path to descriptors
* \param dictionary BoW dictionary to be trained
*/
int train_BoW(char* descriptor_files, cv::Mat* dictionary);

/**
* \brief Makes BoW descriptor file with SIFT features.
*/
int make_BoW_descriptor_file(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file, cv::BOWImgDescriptorExtractor* bowDE, int nFeatures, int nOctaveLayers, float contrastThreshold, float edgeThreshold, float sigma);

/**
* Takes a path to a folder containing file `positions.txt` and folder `images`
* position records in format:`sec pos_x pos_y rot_z rot_w\n`
* image labels in format: `image_sec.png`
* Create file with descriptors of images and respective positions of the robot
*/
int make_naive_descriptor_file(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file);

/**
* \brief 	Makes descriptor file and saves the descriptors 
* 			into a matrix.
*
* \param all_descriptors the descriptor matrix to be created
*/
int make_descriptor_file(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file, char* path_to_used_pos_file, cv::Mat *all_descriptors, cv::FeatureDetector* detector);

/**
* \brief Devides the data into train and test datasets.
*/
int make_train_and_test_datasets(char* path);

/**
* \brief 	Creates a file with weights set to '1' for a set 
*			of database images.
*/
int make_weights_file(cv::Mat descriptors, std::ofstream* weights_file, std::vector< std::vector<float> >* weights);

/**
* \brief 	Processes positions and images taken at those 
* 			positions into descriptor and keypoint file.
*/
int make_descriptor_file_with_keypoints(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file, char* path_to_used_pos_file, char* path_to_kp_file, cv::Mat *all_descriptors, cv::FeatureDetector* detector);

#endif //DATA_PROCESSING_H
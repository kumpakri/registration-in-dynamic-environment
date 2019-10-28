/**
 * \file Utils.h
 *
 * \brief   Declares supportive functions.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 09 2018
 */

#ifndef UTILS_H
#define UTILS_H

#define SQUARE 0
#define ABSDEV 1

#include <limits> //infinity
#include <opencv2/opencv.hpp> //cv::norm
#include <opencv2/nonfree/nonfree.hpp>  //cv::DescriptorExtractor etc.

int print_out_vector(std::vector<double> vec);

int print_out_vector(std::vector<float> vec);

int print_out_vector(std::vector<int> vec);

/**
* \return true if file exists.
*/
bool file_exists(const char *filename);

/**
* \brief Returns substring of source string starting at
* index `start_index` (0 for beginning of the string)
* and `num_of_chars` long.
*
* Free the substring after use!
*
* \return substring
*/
char* get_substring(char* source, int start_index, int end_index);

/**
* \brief Puts a double element from `pull_index` to the position 
* 		 of `push_index` and shifts remaining values in the vector.
*
* \param vec vector to execute the operation on
* \param pull_index position to take the value from
* \param push_index position to insert the value to
*/
int reorder_vector(std::vector<double>* vec, int pull_index, int push_index);

/**
* \brief Puts an int element from `pull_index` to the position 
* 		 of `push_index` and shifts remaining values in the vector.
*
* \param vec vector to execute the operation on
* \param pull_index position to take the value from
* \param push_index position to insert the value to
*/
int reorder_vector(std::vector<int>* vec, int pull_index, int push_index);

/**
* /brief Computes the vector of sorted indices in ascending order.
*/
int sort_vector(std::vector<double> data, std::vector<int>* indices);

/**
* \brief Computes image distace according to chosen method. 
*
* \param method :
* 	- ABSDEV 	sum(abs(I1-I2))/N
* 	- SQUARE 	sum((I1-I2)^2)/N
* \return distance between images
*/
double image_distance_naive(std::vector<unsigned int> source, std::vector<unsigned int> target, int metric);

/**
* \brief Computes the best matched position.
*
* \param path path to descriptor file
* \param result stringstream where to store results
*/
int find_high_scoring_position(std::vector<unsigned int> query_descriptor, char* path, std::stringstream* result);

/**
* \brief Computes the best matched position .
*
* \param path path to descriptor file
* \param result stringstream where to store results
*/
int find_high_scoring_position(std::vector<float> query_descriptor, char* path, std::stringstream* result);

/**
* \brief Prints out statistics for feature matching.
*
* \param statistics {#keypoints1,#keypoints2,#absIn,#relIn,error,error_man}
*/
int get_statistics_feature_matching(cv::Mat img_src, cv::Mat img_dst, cv::FeatureDetector* detector, cv::DescriptorExtractor* extractor, cv::DescriptorMatcher* matcher, std::vector<cv::Point2f> keypoints_src, std::vector<cv::Point2f> keypoints_dst, double statistics[6], cv::Mat* transformation);

/**
* \brief Computes error of image transformed from 
* 		 img_src to viewpoint of img_dst using 
* 		 all pixels of the image.
*/
double get_transform_error_from_all_pixels(cv::Mat img_src, cv::Mat img_dst, cv::Mat h);

/**
* \brief Loads `<cv::Point2f>` vector from the file
*/
int load_point2f_vector(char* path, std::vector<cv::Point2f>* keypoints);



/**
* \brief Computes the error transformation from 
* 	     `img_src` to viewpoint of `img_dst` using 
* 		 manually selected keypoints.
*
* \return transform error
*/
double get_keypoint_transform_error(std::vector<cv::Point2f> keypoints_src, std::vector<cv::Point2f>keypoints_dst, cv::Mat h, int width, int height);

/**
* \brief Draws circles into the image `img` at points in `kps`.
*/
int drawPoints(cv::Mat img, std::vector<cv::Point2f> kps);

/**
* \brief Gets a matrix of positions loaded from txt file
* matrix positions must be of the appropriate size (descriptors.rows,3)
*/
int load_position_matrix(char* path, cv::Mat *positions);

bool kp_in_bounds(cv::KeyPoint kp, int width, int height);

/**
* \brief Returns keypoints and descriptors and keypoints of the image.
*/
int extract_image_data(cv::Mat image, cv::FeatureDetector* detector, std::vector<cv::KeyPoint> *keypoints, cv::Mat* descriptors);

/**
* \brief Gets submatrix with first column of value `id`.
*/
int get_descriptor_matrix(cv::Mat descriptors_all, cv::Mat* descriptors, float id);

/**
* \return concated descriptor matrix
*/
int vconcat_descriptor_matrix(cv::Mat descriptors_mat1, cv::Mat descriptors_mat2, cv::Mat *descriptors_all);

/**
* \brief Creates a vector of weights set to '1' for a set of database images.
*/
int make_weights_vector(cv::Mat descriptors, std::vector< std::vector<float> > *weights);

/**
* \brief Loads weights from file.
*/
int load_weights(char* path, std::vector< std::vector<float> > *weights);

/**
* \brief Loads keypoints from file.
*/
int load_keypoints(char* path, std::vector< std::vector<cv::KeyPoint> > *keypoints);

/**
* \brief Loads descriptors, positions and keypoints and 
* 	  	 creates weights and past distances vectors.
*/
int load_database_with_weights(
   std::vector<cv::Mat>* descriptors_db, 
   std::vector<cv::Mat>* positions_db, 
   std::vector< std::vector< std::vector<cv::KeyPoint> > >* keypoints_db, 
   std::vector< std::vector< std::vector<float> > > *weights, 
   std::vector< std::vector< std::vector<float> > > *distances_past
   );

#endif //UTILS_H
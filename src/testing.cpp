/**
 * \file testing.cpp
 *
 * \brief Runs position estimation process.
 *
 * Allows to define setting and method of estimation,
 * 
 * \param n_of_runs Estimation is possible to run multiple times
 * on the same data to see whether the results 
 * better with time 
 *
 * \param method_type values:
 * 	- KEYPOINT_TF_WEIGHTS 	for keypoint tranformation method
 * 	- GLOBAL_REF			for global reference value method 
 * 	- PAST_DISTANCE 		for past distance subtraction method
 * 	- VECTOR_NORMALIZATION 	for distance vector normalization method
 * 	- IMG_TF_WEIGHTS 		for image transformation method
 *
 * \param gamma weighting parameter
 * \param ref_val global reference value
 * \param alpha past distances subtraction method parameter
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: May 22 2018
 */

#include <fstream>
#include "Utils.h"
#include "Data_processing.h"
#include "Estimation.h"


int do_the_test(
    char* output_path,
    int type,
    char* label, 
    std::vector< cv::Mat > descriptors_db, 
    std::vector< cv::Mat > positions_db, 
    std::vector< std::vector< std::vector<cv::KeyPoint> > > keypoints_db, 
    std::vector< std::vector< std::vector<float> > >* weights,
    std::vector< std::vector< std::vector<float> > >* distances_past,
    float ref_val,
    float gamma,
    float alpha,
    char* path_to_positions_file, 
    char* path_to_image_folder,
    int run_i )
{

	char file_path[100];

	// check if path ends with '/'
	if( output_path[strlen(output_path)-1] != '/' )
	{
		char output_path_tmp[100];
		strncpy(output_path_tmp, output_path, sizeof(output_path_tmp));
		output_path_tmp[strlen(output_path)+1] = '\0';
		output_path_tmp[strlen(output_path)] = '/';
		output_path=output_path_tmp;
	} 

	// print out the path for conformation
	std::cout << "\nResults will be written into: " << output_path << "\n";

	//prepare file name
	switch(type) {
		case KEYPOINT_TF_WEIGHTS:
			snprintf(file_path, sizeof(file_path)*sizeof(char*), "%skp_tf_tst_%s_gamma%d_ref_%d_run%d.txt",output_path, label,(int)(gamma*100),(int)ref_val,run_i);
			break;

		case GLOBAL_REF:
			snprintf(file_path, sizeof(file_path)*sizeof(char*), "%sglobal_ref_tst_%s_gamma%d_ref_%d_run%d.txt", output_path, label,(int)(gamma*100),(int)ref_val,run_i);
			break;

		case PAST_DISTANCE:
			snprintf(file_path, sizeof(file_path)*sizeof(char*), "%spast_dist_tst_%s_gamma%d_alpha_%d_run%d.txt", output_path, label,(int)(gamma*100),(int)alpha,run_i);
			break;

		case VECTOR_NORMALIZATION:
			snprintf(file_path, sizeof(file_path)*sizeof(char*), "%svec_norm_tst_%s_gamma%d__run%d.txt", output_path, label,(int)(gamma*100),run_i);
			break;

		case IMG_TF_WEIGHTS:
			snprintf(file_path, sizeof(file_path)*sizeof(char*), "%simg_tf_tst_%s_gamma%d_ref_%d_run%d.txt",output_path, label,(int)(gamma*100),(int)ref_val,run_i);
			break;

	}
	 
 
	std::ofstream result_file (file_path);
	if( ! result_file )	
	{
		std::cout << "Error opening file for writing "<< label << "\n" << std::endl ;
		return -1 ;
	}

	cv::FeatureDetector *detector;
	detector = new cv::FastFeatureDetector(20,true);

	get_position_estimate_weighted(type, path_to_positions_file, path_to_image_folder, descriptors_db, positions_db, keypoints_db, weights, distances_past, gamma, ref_val, alpha, detector, &result_file);

	return 0;
}

/**
* \brief Runs position estamation with specified setting.
*
* \param method_type values:
* 	- KEYPOINT_TF_WEIGHTS 	for keypoint tranformation method
* 	- GLOBAL_REF			for global reference value method 
* 	- PAST_DISTANCE 		for past distance subtraction method
* 	- VECTOR_NORMALIZATION 	for distance vector normalization method
* 	- IMG_TF_WEIGHTS 		for image transformation method
*/

int main(int argc, char **argv) 
{

	//=== SETTING ========
	/* parameters that are not used by the chosen method must be kept of size 1 */
	char* output_path = "src/data_acquisition/data/testing/";
	int method_type = GLOBAL_REF;
	/*specify label for each query record (s- set, m- measurement)*/
	char* labels[3] = {"s3m1","s3m2","s3m3"};
	float gamma[1] = {0.95};
	float ref_val[1] = {500};
	float alpha[1]={300};
	int n_of_runs = 10;

	//====================

	// plays to ensure user that volume is up
	system("play -n synth 0.1 sine 500 vol 0.5");



	// location of query data
	char* path_to_positions_file[3] = {"src/data_acquisition/data/processed_data/arena_record_changed_left/positions.txt","src/data_acquisition/data/processed_data/arena_record_changed_right/positions.txt","src/data_acquisition/data/processed_data/arena_record_changed_random/positions.txt"};
	char* path_to_image_folder[3] = {"src/data_acquisition/data/processed_data/arena_record_changed_left/images","src/data_acquisition/data/processed_data/arena_record_changed_right/images","src/data_acquisition/data/processed_data/arena_record_changed_random/images"};


	// LOAD database

	std::vector< cv::Mat > descriptors_db;
	std::vector< cv::Mat > positions_db;
	std::vector< std::vector< std::vector<cv::KeyPoint> > > keypoints_db;
	std::vector< std::vector< std::vector<float> > > weights;
	std::vector< std::vector< std::vector<float> > > distances_past;

	load_database_with_weights(&descriptors_db, &positions_db, &keypoints_db, &weights, &distances_past);

	

	for( int run_i=0; run_i < n_of_runs; run_i++)
	{  
	for( int gamma_i=0; gamma_i < sizeof(gamma)/sizeof(float); gamma_i++) 
	{
	for (int ref_i = 0; ref_i < sizeof(ref_val)/sizeof(float); ref_i++)
	{
	for (int alpha_i = 0; alpha_i < sizeof(alpha)/sizeof(float); alpha_i++)
	{
	for( int i = 0; i<sizeof(labels)/sizeof(char*); i++ ) {

		do_the_test(output_path, method_type, labels[i], descriptors_db, positions_db, keypoints_db, &weights, &distances_past, ref_val[ref_i], gamma[gamma_i], alpha[alpha_i], path_to_positions_file[i], path_to_image_folder[i],run_i);
					
	}
	}
	}
	}
	}

	// plays to notify user of finished computation
	system("play -n synth 0.3 sine 1100 vol 0.5");
	system("play -n synth 0.2 sine 800 vol 0.5");

	return 0;
}

/**
 * \file feature_detector_evaluation.cpp
 *
 * \brief Makes feature detectors testing and result
 * 		  writes into a file in a format readable to
 *		  spreadsheet applications.
 *
 * \param 1 path to folder where to save transformation images
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: April 03 2018
 */

#include <fstream>
#include "Utils.h"
#include "Data_processing.h"

int main(int argc, char **argv)
{

	// check if arguments ok
	if ( ! (argc>1 )  )
	{
		std::cout << "Error: missing arguments.\n";
		return -1;
	}

	if ( !(file_exists(argv[1]) )  )
	{
		std::cout << "Error: invalid file path.\n";
		return -1;
	}

	char* output_path = argv[1];

	// specification of image pairs over which to compute the statistics
	const int num_of_pairs = 3;

	char* source_images[num_of_pairs] = {"src/data_acquisition/data/image_pairs/source_arena.png","src/data_acquisition/data/image_pairs/source_change.png","src/data_acquisition/data/image_pairs/source_door.png"};

	char* target_images[num_of_pairs] = {"src/data_acquisition/data/image_pairs/target_arena.png","src/data_acquisition/data/image_pairs/target_change.png","src/data_acquisition/data/image_pairs/target_door.png"};

	char* keypoints_path_src[num_of_pairs] = {"src/data_acquisition/data/example_detector_evaluation/source_arena_kp.txt","src/data_acquisition/data/example_detector_evaluation/source_change_kp.txt","src/data_acquisition/data/example_detector_evaluation/source_door_kp.txt"};

	char* keypoints_path_dst[num_of_pairs] = {"src/data_acquisition/data/example_detector_evaluation/target_arena_kp.txt","src/data_acquisition/data/example_detector_evaluation/target_change_kp.txt","src/data_acquisition/data/example_detector_evaluation/target_door_kp.txt"};

	char* labels[num_of_pairs] = {"ARENA","CHANGE","DOOR"};

	std::stringstream absolute_table_big, absolute_table_small, relative_table_big,  relative_table_small, keypoints_table_big, keypoints_table_small, time_table_big, time_table_small, error_table_big, error_table_small;

	absolute_table_big << "absolute;arena;change;door;sum;average\n";
	relative_table_big << "\nrelative;arena;change;door;sum;average\n";
	keypoints_table_big << "\nkeypoits;arena;change;door;sum;average\n";
	time_table_big  << "\ntime;arena;change;door;sum;average\n";
	error_table_big  << "\nerror;arena;change;door;sum;average\n";

	absolute_table_small << "absolute;arena;change;door;sum;average\n";
	relative_table_small << "\nrelative;arena;change;door;sum;average\n";
	keypoints_table_small << "\nkeypoits;arena;change;door;sum;average\n";
	time_table_small << "\ntime;arena;change;door;sum;average\n";
	error_table_small  << "\nerror;arena;change;door;sum;average\n";

	char name[100];
 	double statistics_big[6], statistics_small[6];

	
// ==== DETECTORS =======================

 	//SIFT detector setting
	float nFeatures = 0;
	float nOctaveLayers[2] = {3,4};
	float contrastThreshold[2] = {.02,.03};
	float edgeThreshold[2] = {3,10};
	float sigma[3] = {1.2,1.4,1.6};


	//FAST detector setting
	int  	threshold[3] = {13,16,20};
	bool  	nonmaxSuppression[2] = {false,true};

	cv::FeatureDetector *detector;

	char* detector_names[2] = {"SIFT", "FAST"};

// ==== EXTRACTORS ======================

	cv::DescriptorExtractor* SIFTextractor, *extractor;

	SIFTextractor = new cv::SiftDescriptorExtractor();

	char* extractor_names[1] = {"SIFT"};

// ==== MATCHERS ========================

	cv::DescriptorMatcher *BFmatcher;

	BFmatcher = new cv::BFMatcher();


// ======================================

// ==== IMAGES ==========================

	cv::Mat img_tmp;
	std::vector<cv::Mat> imgs_src_big, imgs_dst_big, imgs_src_small, imgs_dst_small;

	std::vector<std::vector<cv::Point2f> > keypoints_src_big, keypoints_dst_big, keypoints_src_small, keypoints_dst_small;

	std::vector<cv::Point2f> keypoints_src_tmp, keypoints_dst_tmp;


	for(int image_i=0; image_i<num_of_pairs; image_i++)
	{
		img_tmp = cv::imread( source_images[image_i], CV_LOAD_IMAGE_GRAYSCALE );
		cv::normalize(img_tmp, img_tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		imgs_src_big.push_back(img_tmp);
		img_tmp.release();

		img_tmp = cv::imread( target_images[image_i], CV_LOAD_IMAGE_GRAYSCALE );
		cv::normalize(img_tmp, img_tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	 	imgs_dst_big.push_back(img_tmp);
	 	img_tmp.release();


	 	/*
	    * INTER_NEAREST - a nearest-neighbor interpolation
	    * INTER_LINEAR - a bilinear interpolation (used by default), 
	    *                * recommended for enlargement
	    * INTER_AREA - resampling using pixel area relation. It may 
	    * be a preferred method for image decimation, as it gives 
	    * moire’-free results. But when the image is zoomed, it is 
	    * similar to the INTER_NEAREST method. * recommended for schrinking
	    * INTER_CUBIC - a bicubic interpolation over 4x4 pixel neighborhood, 
	    *               * recommended for enlargement
	    * INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood
		*/

		cv::resize(imgs_src_big[image_i], img_tmp, cv::Size(), .5, .5, CV_INTER_AREA );
		imgs_src_small.push_back(img_tmp);
		img_tmp.release();
		cv::resize(imgs_dst_big[image_i], img_tmp, cv::Size(), .5, .5, CV_INTER_AREA );
		imgs_dst_small.push_back(img_tmp);
		img_tmp.release();

		//get keypoints
		load_point2f_vector(keypoints_path_src[image_i], &keypoints_src_tmp );
		load_point2f_vector(keypoints_path_dst[image_i], &keypoints_dst_tmp );

		keypoints_src_big.push_back(keypoints_src_tmp);
		keypoints_dst_big.push_back(keypoints_dst_tmp);

		keypoints_src_tmp.clear();
		keypoints_dst_tmp.clear();

		for( int i = 0; i<keypoints_src_big[image_i].size(); i++)
		{								
			keypoints_src_tmp.push_back(cv::Point2f((int)(keypoints_src_big[image_i].at(i).x/2),(int)(keypoints_src_big[image_i].at(i).y/2)));

			keypoints_dst_tmp.push_back(cv::Point2f((int)(keypoints_dst_big[image_i].at(i).x/2),(int)(keypoints_dst_big[image_i].at(i).y/2)));

		}

		keypoints_src_small.push_back(keypoints_src_tmp);
		keypoints_dst_small.push_back(keypoints_dst_tmp);

		keypoints_src_tmp.clear();
		keypoints_dst_tmp.clear();
		
	}
// ======================================

	cv::Mat img_tf_all_big;

	cv::Mat img_tf_results_big(cv::Size(imgs_src_big[0].cols*2,imgs_src_big[0].rows),imgs_src_big[0].type(), cv::Scalar::all(0));

	cv::Mat img_tf_all_small;

	cv::Mat img_tf_results_small(cv::Size(imgs_src_small[0].cols*2,imgs_src_small[0].rows),imgs_src_small[0].type(), cv::Scalar::all(0));


 	for ( int detector_i = 0; detector_i < sizeof(detector_names)/sizeof(detector_names[0]); detector_i++)
 	{
 		try{

 		if( detector_i == 0 ){ //SIFT
			extractor = SIFTextractor;

			for( int nOL_i = 0; nOL_i<sizeof(nOctaveLayers)/sizeof(nOctaveLayers[0]);nOL_i++){
			for( int cT_i = 0; cT_i<sizeof(contrastThreshold)/sizeof(contrastThreshold[0]); cT_i++ ){
			for( int eT_i = 0; eT_i<sizeof(edgeThreshold)/sizeof(edgeThreshold[0]); eT_i++ ){
			for( int s_i = 0; s_i<sizeof(sigma)/sizeof(sigma[0]); s_i++ ){
				
				detector = new cv::SiftFeatureDetector( 0, nOctaveLayers[nOL_i], contrastThreshold[cT_i], edgeThreshold[eT_i], sigma[s_i]);

				snprintf(name, sizeof(name)*sizeof(char*), "%s(0,%.2f,%.2f,%.2f,%.2f);", detector_names[detector_i],nOctaveLayers[nOL_i],contrastThreshold[cT_i],edgeThreshold[eT_i],sigma[s_i]);

				absolute_table_big << name;
				relative_table_big << name;
				keypoints_table_big << name;
				time_table_big << name;
				error_table_big << name;

				absolute_table_small << name;
				relative_table_small << name;
				keypoints_table_small << name;
				time_table_small << name;
				error_table_small << name;

				for(int image_i=0; image_i<num_of_pairs; image_i++)
				{

					get_statistics_feature_matching(imgs_src_big[image_i], imgs_dst_big[image_i], detector, extractor, BFmatcher, keypoints_src_big[image_i], keypoints_dst_big[image_i], statistics_big, &img_tf_results_big);

					if(image_i==0){
						img_tf_results_big.copyTo(img_tf_all_big);
					} else {
						cv::vconcat(img_tf_all_big, img_tf_results_big, img_tf_all_big);
					}

					get_statistics_feature_matching(imgs_src_small[image_i], imgs_dst_small[image_i], detector, extractor, BFmatcher, keypoints_src_small[image_i], keypoints_dst_small[image_i], statistics_small, &img_tf_results_small);

					if(image_i==0){
						img_tf_results_small.copyTo(img_tf_all_small);
					} else {
					cv::vconcat(img_tf_all_small, img_tf_results_small, img_tf_all_small);}


					// absolute # of inliers
					absolute_table_big << statistics_big[2] << ";";
					// relative # of inliers
					relative_table_big << statistics_big[3] << ";";
					// # of keypoints (average from both pictures) 
					keypoints_table_big <<  (statistics_big[0] + statistics_big[1])/2 << ";";
					// processing time
					time_table_big << statistics_big[4] << ";";
					// error
					error_table_big << statistics_big[5] <<";";

					// absolute # of inliers
					absolute_table_small << statistics_small[2] << ";";
					// relative # of inliers
					relative_table_small << statistics_small[3] << ";"; 
					// # of keypoints (average from both pictures)
					keypoints_table_small << (statistics_small[0] + statistics_small[1])/2 << ";";
					// processing time
					time_table_small << statistics_small[4] << ";";
					// error
					error_table_small << statistics_small[5] << ";";
					
				}

				snprintf(name, sizeof(name)*sizeof(char*), "%s/%s(0,%.2f,%.2f,%.2f,%.2f)B.png", output_path, detector_names[detector_i],nOctaveLayers[nOL_i],contrastThreshold[cT_i],edgeThreshold[eT_i],sigma[s_i]);

				cv::imwrite(name,img_tf_all_big);

				snprintf(name, sizeof(name)*sizeof(char*), "%s/%s(0,%.2f,%.2f,%.2f,%.2f)s.png", output_path, detector_names[detector_i],nOctaveLayers[nOL_i],contrastThreshold[cT_i],edgeThreshold[eT_i],sigma[s_i]);

				cv::imwrite(name,img_tf_all_small);

				absolute_table_big <<  "\n";
				relative_table_big <<  "\n"; 
				keypoints_table_big << "\n";
				time_table_big << "\n";
				error_table_big << "\n";


				absolute_table_small <<  "\n";
				relative_table_small <<  "\n"; 
				keypoints_table_small << "\n";
				time_table_small << "\n";
				error_table_small << "\n";

			}
			}
			}
			}

 		}

		if( detector_i == 1 ){ //FAST
			extractor = SIFTextractor;

			for( int th_i = 0; th_i<sizeof(threshold)/sizeof(threshold[0]);th_i++){
			for( int nonS_i = 0; nonS_i<sizeof(nonmaxSuppression)/sizeof(nonmaxSuppression[0]); nonS_i++ ){

				detector = new cv::FastFeatureDetector( threshold[th_i], nonmaxSuppression[nonS_i] );

				snprintf(name, sizeof(name)*sizeof(char*), "%s(%d,%s);", detector_names[detector_i], threshold[th_i], nonmaxSuppression[nonS_i] ? "true":"false");

				absolute_table_big << name;
				relative_table_big << name;
				keypoints_table_big << name;
				time_table_big << name;
				error_table_big << name;

				absolute_table_small << name;
				relative_table_small << name;
				keypoints_table_small << name;
				time_table_small << name;
				error_table_small << name;

				for(int image_i=0; image_i<num_of_pairs; image_i++)
				{
					get_statistics_feature_matching(imgs_src_big[image_i], imgs_dst_big[image_i], detector, extractor, BFmatcher, keypoints_src_big[image_i], keypoints_dst_big[image_i], statistics_big, &img_tf_results_big);

					if(image_i==0){
						img_tf_results_big.copyTo(img_tf_all_big);
					} else {
					cv::vconcat(img_tf_all_big, img_tf_results_big, img_tf_all_big);
					}

					get_statistics_feature_matching(imgs_src_small[image_i], imgs_dst_small[image_i], detector, extractor, BFmatcher, keypoints_src_small[image_i], keypoints_dst_small[image_i], statistics_small, &img_tf_results_small);

					if(image_i==0){
						img_tf_results_small.copyTo(img_tf_all_small );
					} else {
					cv::vconcat(img_tf_all_small, img_tf_results_small, img_tf_all_small);}

					// absolute # of inliers
					absolute_table_big << statistics_big[2] << ";";
					// relative # of inliers
					relative_table_big << statistics_big[3] << ";";
					// # of keypoints (average from both pictures) 
					keypoints_table_big <<  (statistics_big[0] + statistics_big[1])/2 << ";";
					// processing time
					time_table_big << statistics_big[4] << ";";
					// error
					error_table_big << statistics_big[5] <<";";

					// absolute # of inliers
					absolute_table_small << statistics_small[2] << ";";
					// relative # of inliers
					relative_table_small << statistics_small[3] << ";"; 
					// # of keypoints (average from both pictures)
					keypoints_table_small << (statistics_small[0] + statistics_small[1])/2 << ";";
					// processing time
					time_table_small << statistics_small[4] << ";";
					// error
					error_table_small << statistics_small[5] << ";";
					
				}

				snprintf(name, sizeof(name)*sizeof(char*), "%s/%s(%d,%s)B.png", output_path, detector_names[detector_i], threshold[th_i], nonmaxSuppression[nonS_i] ? "true":"false");

				cv::imwrite(name,img_tf_all_big);

				snprintf(name, sizeof(name)*sizeof(char*), "%s/%s(%d,%s)s.png",output_path, detector_names[detector_i], threshold[th_i], nonmaxSuppression[nonS_i] ? "true":"false");

				cv::imwrite(name,img_tf_all_small);

				absolute_table_big <<  "\n";
				relative_table_big <<  "\n"; 
				keypoints_table_big << "\n";
				time_table_big << "\n";
				error_table_big << "\n";

				absolute_table_small <<  "\n";
				relative_table_small <<  "\n"; 
				keypoints_table_small << "\n";
				time_table_small << "\n";
				error_table_small << "\n";
			}
			}

		}

	}catch(...) {
		std::cout << "Big images\n";
	 	std::cout << absolute_table_big.str();
		std::cout << relative_table_big.str();
		std::cout << keypoints_table_big.str();
		std::cout << time_table_big.str();
		std::cout << error_table_big.str()<< std::endl;

		std::cout << "Small images\n";
	 	std::cout << absolute_table_small.str();
		std::cout << relative_table_small.str();
		std::cout << keypoints_table_small.str();
		std::cout << time_table_small.str();
		std::cout << error_table_small.str();

		return -1;
	}	
 		
 	}

 	std::cout << "Big images\n";
 	std::cout << absolute_table_big.str();
	std::cout << relative_table_big.str();
	std::cout << keypoints_table_big.str();
	std::cout << time_table_big.str();
	std::cout << error_table_big.str()<< std::endl;

	std::cout << "Small images\n";
 	std::cout << absolute_table_small.str();
	std::cout << relative_table_small.str();
	std::cout << keypoints_table_small.str();
	std::cout << time_table_small.str();
	std::cout << error_table_small.str();

	return 0;
}

/**
 * \file Data_processing.cpp
 *
 * \brief 	Contains functions for data processing
 *			and data file creation.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 09 2018
 */

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp> 
#include <fstream>
#include <iterator>
#include <stdlib.h> // strtoul

#include "Data_processing.h"
#include "Utils.h"

/**
* Returns a trained BoW dictionary. descriptor_file contains matrix of SIFT 
* descriptors without id column, stored with cv::FileStorage as "matDescriptors"
*/
int train_BoW(char* descriptor_file, cv::Mat* dictionary)
{

	if (! file_exists(descriptor_file))
		{ 
			std::cout << "\nError: train SIFT descriptor file does not exist.\n";
			return -1;
		}

	int dictionarySize = 1000; // < number of descriptors
	cv::TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
	int retries=1;
	int flags=cv::KMEANS_PP_CENTERS;
	cv::BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);

	cv::Mat temp;
	cv::FileStorage descriptor_file_h(descriptor_file,cv::FileStorage::READ);
	descriptor_file_h["matDescriptors"] >> temp;
	bowTrainer.add(temp);
	descriptor_file_h.release();

	*dictionary=bowTrainer.cluster();

	return 0;
}

/**
* Returns a trained BoW dictionary. Accepts list of pilepaths to the descriptor files.
*/
int train_BoW_multi(char** descriptor_files, cv::Mat* dictionary)
{
	bool enable = true;
	int i = 0;

	int dictionarySize = 1000; // < number of descriptors
	cv::TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
	int retries=1;
	int flags=cv::KMEANS_PP_CENTERS;
	cv::BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);

	while(enable==true)
	{
		try{
			cv::Mat temp;
			cv::FileStorage descriptor_file(descriptor_files[i],cv::FileStorage::READ);
			descriptor_file["matDescriptors"] >> temp;
			bowTrainer.add(temp);
			descriptor_file.release();
			i++;
		}catch(...){enable=false;}
	}

	*dictionary=bowTrainer.cluster();

	return 0;
}

/**
* Makes BoW descriptor file with SIFT features.
*/
int make_BoW_descriptor_file(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file, cv::BOWImgDescriptorExtractor* bowDE, int nFeatures, int nOctaveLayers, float contrastThreshold, float edgeThreshold, float sigma)
{

	//load position file
	std::ifstream position_file (path_to_positions_file);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	}

	std::ofstream descriptor_file(path_to_descriptor_file);
	if( ! descriptor_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}

	int sec;
	float pos_x, pos_y, yaw;
	
	while (position_file >> sec >> pos_x >> pos_y >> yaw)
	{
		//search for image taken at the same time
		char im_path[100];
		snprintf(im_path, sizeof(im_path)*sizeof(char*), "%s/image_%d.png", path_to_image_folder,sec);

		if (file_exists(im_path))
		{
			cv::Mat image, descriptors;
			image = cv::imread(im_path, CV_LOAD_IMAGE_GRAYSCALE);

			std::vector<cv::KeyPoint> keypoints;

			cv::FeatureDetector* detector;
			detector = new cv::SiftFeatureDetector( nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
			detector->detect( image, keypoints );

			if( keypoints.size() >= 5 )
			{

				cv::Mat bowDescriptor;//(1,1000,CV_32FC1);   
				std::vector<float> bow_vec;     
			    //extract BoW (or BoF) descriptor from given image
			    bowDE->compute(image,keypoints,bowDescriptor);

			    bow_vec = bowDescriptor.row(0);

				//write in the descriptor file
				descriptor_file << pos_x << " " << pos_y << " " << yaw << " ";

				std::ostream_iterator<float> output_iterator(descriptor_file, " ");
		    	std::copy(bow_vec.begin(), bow_vec.end(), output_iterator);

				descriptor_file << "\n";
			}

		}
	}
	descriptor_file.close();
	position_file.close();

	return 0;
}

/*
* Makes SIFT descriptor file.
*/
int make_descriptor_file(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file, char* path_to_used_pos_file, cv::Mat *all_descriptors, cv::FeatureDetector* detector)
{
	//load position file
	std::string line;

	std::ifstream position_file (path_to_positions_file);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	}

	std::ofstream used_pos_file (path_to_used_pos_file);
	if( ! used_pos_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}

	int sec;
	float pos_x, pos_y, yaw;
	
	int id = 0;
	while (position_file >> sec >> pos_x >> pos_y >> yaw)
	{
		//search for image taken at the same time
		char im_path[100];
		snprintf(im_path, sizeof(im_path)*sizeof(char*), "%s/image_%d.png", path_to_image_folder,sec);

		if (file_exists(im_path))
		{
			cv::Mat image, descriptors;
			image = cv::imread(im_path, CV_LOAD_IMAGE_GRAYSCALE);

			std::vector<cv::KeyPoint> keypoints;

			detector->detect( image, keypoints );
			if( keypoints.size() > 4 ) 
			{

				//make descriptor of the feature (128-D vector)
				cv::DescriptorExtractor* extractor;
				extractor = new cv::SiftDescriptorExtractor();

				extractor->compute(image, keypoints, descriptors);

				//size of descriptors
				//create column of id
				cv::Mat id_column= cv::Mat::ones(descriptors.rows, 1, descriptors.type())*id;

				//hconcat
				cv::hconcat(id_column,descriptors,descriptors); //error

				(*all_descriptors).push_back(descriptors);
				id ++;
				used_pos_file << pos_x << " " << pos_y << " " << yaw << std::endl;
			} 

		}
	}
	cv::FileStorage descriptor_file(path_to_descriptor_file, cv::FileStorage::WRITE);
	descriptor_file << "matDescriptors" << (*all_descriptors);
	descriptor_file.release();
	position_file.close();

	return 0;
}

/*
* Takes a path to a folder containing file `positions.txt` and folder `images`
* position records in format:`sec pos_x pos_y rot_z rot_w\n`
* image labels in format: `image_sec.png`
* Create file with naive descriptors of images and respective positions of the robot.
*/
int make_naive_descriptor_file(char* path_to_positions_file, char* path_to_image_folder, char* path_to_descriptor_file)
{

	//load position file
	std::string line;

	std::ifstream position_file (path_to_positions_file);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	}

	int sec;
	float pos_x, pos_y, yaw;

	std::ofstream descriptor_file(path_to_descriptor_file);
	if( ! descriptor_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}
	
	while (position_file >> sec >> pos_x >> pos_y >> yaw)
	{
		//search for image taken at the same time
		char im_path[100];
		snprintf(im_path, sizeof(im_path)*sizeof(char*), "%s/image_%d.png", path_to_image_folder,sec);

		if (file_exists(im_path))
		{
			cv::Mat image;
			image = cv::imread(im_path, CV_LOAD_IMAGE_GRAYSCALE);

			//resize image
			cv::Size size(100,100);
			cv::Mat small_image;
			cv::resize(image,small_image,size);

			//compute descriptor
			std::vector<unsigned int>descriptor(small_image.begin<unsigned int>(), small_image.end<unsigned int>());
			descriptor.assign(small_image.datastart,small_image.dataend);

			//write in the descriptor file
			
			descriptor_file << pos_x << " " << pos_y << " " << yaw << " ";
			std::ostream_iterator<unsigned int> output_iterator(descriptor_file, " ");
	    	std::copy(descriptor.begin(), descriptor.end(), output_iterator);
			descriptor_file << "\n";
		}
	}
	descriptor_file.close();
	position_file.close();
	return 0;
}

/*
* Devides the data into train and test datasets in ratio 2:1.
*/
int make_train_and_test_datasets(char* path)
{
	// read the descriptor file
	std::ifstream descriptor_file (path);
	if( ! descriptor_file )	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	} 

	char test_file_path[100];
	char* str = get_substring(path, 0, strlen(path)-4);

	snprintf(test_file_path, sizeof(test_file_path)*sizeof(char*), "%s_test.txt", str);

	std::ofstream test_dataset_file(test_file_path);
	if( ! test_dataset_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}

	char train_file_path[100];
	snprintf(train_file_path, sizeof(train_file_path)*sizeof(char*), "%s_train.txt", str);

	std::ofstream train_dataset_file(train_file_path);
	if( ! train_dataset_file )	
	{
		std::cout << "Error opening file for writing\n" << std::endl ;
		return -1 ;
	}

	free(str);
	
	std::vector<std::string> vec;
	std::string row;
	std::string element;
	std::vector<unsigned int> descriptor;
	
	int count = 1;
	while( std::getline(descriptor_file,row) )
	{
		vec.clear();
		descriptor.clear();
		std::stringstream ss ( row );
		while ( getline( ss, element, ' ') ) 
		{
			vec.push_back(element);
		}
		
		for (int i = 3; i<vec.size();i++)
		{
			descriptor.push_back(std::strtoul(vec[i].c_str(),0,10));
		}

		if (vec.size() > 3) {
			
			if( (count%3 == 0) )
			{ //add to test dataset
				test_dataset_file << vec[0] << " " << vec[1] << " " << vec[2] << " ";
				std::ostream_iterator<unsigned int> output_iterator(test_dataset_file, " ");
		    	std::copy(descriptor.begin(), descriptor.end(), output_iterator);
				test_dataset_file << "\n";
			}else{ //add to train dataset
				train_dataset_file << vec[0] << " " << vec[1] << " " << vec[2] << " ";
				std::ostream_iterator<unsigned int> output_iterator(train_dataset_file, " ");
		    	std::copy(descriptor.begin(), descriptor.end(), output_iterator);
				train_dataset_file << "\n";
			}
			count ++;
		}
	}
	train_dataset_file.close();
	test_dataset_file.close();
	descriptor_file.close();
	
	return 0;
}


/**
* Creates a file with weights set to '1' for a set of database images.
*/
int make_weights_file(cv::Mat descriptors, std::ofstream* weights_file, std::vector< std::vector<float> > *weights)
{

	std::vector<float> v;
	float id = 0;
	for( int i = 0; i<descriptors.rows; i++)
	{
		if ( id == descriptors.at<float>(i,0) )
		{
			*weights_file << "1 ";
			v.push_back(1.0);
		} else {
			weights->push_back(v);
			v.clear();
			v.push_back(1.0);
			*weights_file << "\n1 ";
			id = descriptors.at<float>(i,0);
		}
	}

	weights_file->close();

	return 0;
}

/**
* Processes positions and images taken at those positions into descriptor and keypoint file.
*/
int make_descriptor_file_with_keypoints(
	char* path_to_positions_file, 
	char* path_to_image_folder, 
	char* path_to_descriptor_file, 
	char* path_to_used_pos_file, 
	char* path_to_kp_file, 
	cv::Mat *all_descriptors, 
	cv::FeatureDetector* detector)
{
	//load position file
	std::string line;

	std::ifstream position_file (path_to_positions_file);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading pos\n" << std::endl ;
		return -1 ;
	}

	std::ofstream used_pos_file (path_to_used_pos_file);
	if( ! used_pos_file )	
	{
		std::cout << "Error opening file for writing pos\n" << std::endl ;
		return -1 ;
	}

	std::ofstream kp_file (path_to_kp_file);
	if( ! kp_file )	
	{
		std::cout << "Error opening file for writing kp\n" << std::endl ;
		return -1 ;
	}

	int sec;
	float pos_x, pos_y, yaw;
	
	int id = 0;
	while (position_file >> sec >> pos_x >> pos_y >> yaw)
	{
		//search for image taken at the same time
		char im_path[100];
		snprintf(im_path, sizeof(im_path)*sizeof(char*), "%s/image_%d.png", path_to_image_folder,sec);

		if (file_exists(im_path))
		{
			
			cv::Mat image, descriptors;
			image = cv::imread(im_path, CV_LOAD_IMAGE_GRAYSCALE);

			std::vector<cv::KeyPoint> keypoints;

			detector->detect( image, keypoints );
			if( keypoints.size() > 4 ) 
			{

				//make descriptor of the feature (128-D vector)
				cv::DescriptorExtractor* extractor;
				extractor = new cv::SiftDescriptorExtractor();

				extractor->compute(image, keypoints, descriptors);

				//size of descriptors
				//create column of id
				cv::Mat id_column= cv::Mat::ones(descriptors.rows, 1, descriptors.type())*id;

				//hconcat
				cv::hconcat(id_column,descriptors,descriptors);

				(*all_descriptors).push_back(descriptors);
				id ++;

				used_pos_file << pos_x << " " << pos_y << " " << yaw << std::endl;

				//writing into keypoints file
				for (int i = 0; i < keypoints.size(); i++)
				{
					kp_file << keypoints[i].pt.x << " " << keypoints[i].pt.y << " "<< keypoints[i].size <<";";
				}
				kp_file << "\n";
			} 

		}
	}
	cv::FileStorage descriptor_file(path_to_descriptor_file, cv::FileStorage::WRITE);
	descriptor_file << "matDescriptors" << (*all_descriptors);
	descriptor_file.release();
	position_file.close();
	kp_file.close();

	return 0;
}
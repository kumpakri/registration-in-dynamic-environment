/**
 * \file Estimation.cpp
 *
 * \brief 	Contains methods for position estimation and 
 * 			supportive functions.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: May 22 2018
 */

#include "Estimation.h"
#include "Utils.h"
#include <fstream>

/**
* \brief Updates vector weights with new values.
*/
int update_weights_normalization(std::vector<float> *weights, std::vector<float> distances, float gamma)
{
	std::vector<float>  distances_normalized, ones, new_weights;

	cv::normalize(distances, distances_normalized, 0, 1,cv::NORM_MINMAX, CV_32F );

	for (int i = 0; i<distances_normalized.size(); i++)
	{
		ones.push_back(1);
	}

	cv::subtract(ones, distances_normalized, new_weights);

	for (int i = 0; i < weights->size(); i++)
	{
		weights->at(i) = gamma*weights->at(i) + (1-gamma)*new_weights[i];
	}

	return 0;
}

/**
* \brief Updates the vector weights derived for the global 
*   	 reference value method.
*/
int update_weights_ref_val(std::vector<float> *weights, std::vector<float> distances, float gamma, float ref_val)
{
	// TIP ref_val 500
	std::vector<float>  new_weights, new_weights_inverted, ones;
	//std::cout << *std::max_element(distances.begin(), distances.end()) << std::endl;

	for (int i = 0; i<distances.size(); i++)
	{
		ones.push_back(1);
		if( distances[i]/ref_val > 1)
		{
			new_weights_inverted.push_back(1);
		} else {new_weights_inverted.push_back(distances[i]/ref_val);}
		
	}

	cv::subtract(ones, new_weights_inverted, new_weights);

	for (int i = 0; i < weights->size(); i++)
	{
		weights->at(i) = gamma*weights->at(i) + (1-gamma)*new_weights[i];
	}

	return 0;
}

/**
* \brief Updates vector weights for the past distance subtracktion method.
*/
int update_weights_past(std::vector<float> *weights, std::vector<float> distances, std::vector<float> distances_past, float gamma, float d)
{
	std::vector<float>  distances_diff;

	cv::subtract(distances_past, distances, distances_diff);
	float feedback;

	//std::cout << distances_diff[(int)(std::distance(distances_diff.begin(), std::max_element(distances_diff.begin(),distances_diff.end())))] <<"\n";

	for (int i = 0; i < weights->size(); i++)
	{
		feedback = distances_diff[i]/d;
		weights->at(i) = gamma*weights->at(i) + (1-gamma)*(feedback);
		if(weights->at(i) < 0) {weights->at(i) = 0;}
		if(weights->at(i) > 1) {weights->at(i) = 1;}
	}

	return 0;
}

/**
* \brief Takes two descriptor matrix and computes
* 		 their euklidean distance.
*
* \return average distance between images computed from pairs of descriptors.
*/
double get_image_distance(cv::Mat descriptors_db, cv::Mat descriptors_query)
{
	cv::DescriptorMatcher* matcher = new cv::BFMatcher();
	std::vector<cv::DMatch> matches;
	matcher->match(descriptors_query, descriptors_db, matches);

	//compute the distance
	double sum = 0;
	for (int i = 0; i<matches.size(); i++)
	{
		sum = sum + cv::norm(descriptors_query.row(matches[i].queryIdx), descriptors_db.row(matches[i].trainIdx),cv::NORM_L2);
	}

	return sum/matches.size(); 
}

/**
* \brief Computes distance based on matched keipoints 
* 		 methods and updates the weights.
*
* \return distance between images computed from pairs of 
* 		  descriptors using weighting.
*/
double get_image_distance_kp_matching_weighted(
	int method_type, 
	cv::Mat descriptors_db, 
	cv::Mat descriptors_query, 
	std::vector<float>* weights, 
	std::vector<float>* distances_past,
	float gamma, 
	float ref_val,
	float alpha)
{
	cv::DescriptorMatcher *matcher;
	matcher = new cv::BFMatcher();

	std::vector<cv::DMatch> matches;
	matcher->match(descriptors_db, descriptors_query, matches);

	//compute the distance
	std::vector<float> distances;
	double sum = 0;

	for (int i = 0; i<matches.size(); i++)
	{
		distances.push_back(cv::norm(descriptors_db.row(matches.at(i).queryIdx), descriptors_query.row(matches.at(i).trainIdx),cv::NORM_L2));

		sum = sum + weights->at(matches.at(i).queryIdx)*cv::norm(descriptors_db.row(matches.at(i).queryIdx), descriptors_query.row(matches.at(i).trainIdx),cv::NORM_L2);
	}

	switch(method_type){
							

		case GLOBAL_REF:
			update_weights_ref_val(weights, distances, gamma, ref_val);
			break; 

		case PAST_DISTANCE:
			update_weights_past(weights, distances, *distances_past, gamma, alpha);
			std::copy ( distances.begin(), distances.end(), distances_past->begin() );
			break;

		case VECTOR_NORMALIZATION:
			update_weights_normalization(weights, distances, gamma);
			break;

	}
	

	return sum/matches.size(); 
}

/**
* \brief Computes distance based on keypoint transformation
*		 method without weighting.
*
* \return image average descriptor distance 
*/
double get_keypoint_transform_image_distance_without_weighting(
   std::vector<cv::KeyPoint> keypoints_src, 
   cv::Mat descriptors_src, 
   cv::Mat h, 
   cv::Mat img_query
) {
	std::vector<cv::KeyPoint>  keypoints_tf;
	std::vector<cv::Point2f>  points_src, points_tf;

	cv::KeyPoint::convert(keypoints_src, points_src);
	cv::perspectiveTransform( points_src, points_tf, h);
	cv::KeyPoint::convert(points_tf, keypoints_tf);
	
	cv::Mat descriptors_tf;
	cv::DescriptorExtractor* extractor;
	extractor = new cv::SiftDescriptorExtractor();
	extractor->compute(img_query, keypoints_tf, descriptors_tf);

	
	double sum = 0;
	int n = 0;

	for (int i = 0; i < keypoints_tf.size(); i++ ) {
		if( kp_in_bounds(keypoints_tf[i], img_query.rows, img_query.cols) ) 
		{
			sum += cv::norm(descriptors_src.row(i), descriptors_tf.row(i),cv::NORM_L2);
			n++;
		}
	}

	return sum/n;

}

/**
* \brief Computes distance based on the weighted keypoint
*		 transformation method and updates the weights.
*
* \return distance between images
*/
double get_keypoint_transform_image_distance_with_weights(
   std::vector<cv::KeyPoint> keypoints_src, 
   cv::Mat descriptors_src, 
   std::vector<float>* weights,
   cv::Mat h, 
   float gamma,
   cv::Mat img_query,
   float ref_val
) {
	std::vector<cv::KeyPoint>  keypoints_tf;
	std::vector<cv::Point2f>  points_src, points_tf;

	cv::KeyPoint::convert(keypoints_src, points_src);
	cv::perspectiveTransform( points_src, points_tf, h);
	cv::KeyPoint::convert(points_tf, keypoints_tf);
	
	cv::Mat descriptors_tf;
	cv::DescriptorExtractor* extractor;
	extractor = new cv::SiftDescriptorExtractor();
	extractor->compute(img_query, keypoints_tf, descriptors_tf);

	std::vector<float> distances;
	
	double sum = 0;
	int n = 0;

	for (int i = 0; i < keypoints_tf.size(); i++ ) {
		if( kp_in_bounds(keypoints_tf[i], img_query.rows, img_query.cols) ) 
		{
			distances.push_back(cv::norm(descriptors_src.row(i), descriptors_tf.row(i),cv::NORM_L2));
			sum += (*weights)[i]*cv::norm(descriptors_src.row(i), descriptors_tf.row(i),cv::NORM_L2);
			n++;
		}
	}

	update_weights_ref_val(weights, distances, gamma, ref_val);

	return sum/n;

}

/**
* \brief Computes distance based on the weighted image
*		 transformation method and updates the weights.
*
* \return distance between images
*/
double get_keypoint_transform_image_distance_with_weights_imtf(
   std::vector<cv::KeyPoint> keypoints_dst, 
   cv::Mat descriptors_dst, 
   std::vector<float>* weights,
   cv::Mat h, 
   cv::Mat img_query,
   float gamma,
   float ref_val
) {
	cv::Mat img_tf;

	cv::warpPerspective( img_query, img_tf, h.inv(), img_query.size() );
	
	cv::Mat descriptors_tf;
	cv::DescriptorExtractor* extractor;
	extractor = new cv::SiftDescriptorExtractor();
	extractor->compute(img_tf, keypoints_dst, descriptors_tf);

	std::vector<float> distances;
	
	double sum = 0;
	int n = 0;

	for (int i = 0; i < keypoints_dst.size(); i++ ) {
		distances.push_back(cv::norm(descriptors_dst.row(i), descriptors_tf.row(i),cv::NORM_L2));
		sum += (*weights)[i]*cv::norm(descriptors_dst.row(i), descriptors_tf.row(i),cv::NORM_L2);
		n++;
		
	}

	update_weights_ref_val(weights, distances, gamma, ref_val);

	return sum/n;

}



/**
* \brief Gets distance vector for the weighted keypoints
*		 transformation methods.
*/
int get_image_distances_kp_transformation_weighted(
	int method_type,
	cv::Mat descriptors_db_i, 
	cv::Mat descriptors_query,
	std::vector<cv::KeyPoint> keypoints_db_i,
	std::vector<cv::KeyPoint> keypoints_query,
	cv::Mat image,
	std::vector<double> *distance_vec,
	float ref_val,
	float gamma,
	std::vector<float> *weights_i
)
{
	//get homography
	cv::DescriptorMatcher *matcher;
	matcher = new cv::BFMatcher();

	std::vector<cv::DMatch> matches;
	matcher->match(descriptors_db_i, descriptors_query, matches);

	cv::Mat mask;
		std::vector<cv::Point2f> points2f_db, points2f_query;
	for( int x = 0; x<matches.size(); x++)
	{
		points2f_db.push_back(keypoints_db_i[matches[x].queryIdx].pt);
		points2f_query.push_back(keypoints_query[matches[x].trainIdx].pt);
	}

	cv::Mat h = cv::findHomography(points2f_db, points2f_query, CV_RANSAC,3, mask);


	// if relative # of inliers less than, then assign infty
	// if h is singular (unsuccessfull transform), assign infty
	if( (cv::sum( mask )[0]/points2f_db.size() < .20) || (cv::determinant(h) < 0.2) )
	{
		distance_vec->push_back(std::numeric_limits<double>::infinity());
	} else {
		switch(method_type){
							

		case KEYPOINT_TF_WEIGHTS:
			distance_vec->push_back(get_keypoint_transform_image_distance_with_weights(keypoints_db_i, descriptors_db_i, weights_i, h, gamma, image, ref_val));
			break;

		case IMG_TF_WEIGHTS:
			distance_vec->push_back(get_keypoint_transform_image_distance_with_weights_imtf(keypoints_db_i, descriptors_db_i, weights_i, h, image, gamma, ref_val));
			break;

	}
		
	}
	matches.clear();

	return 0;
}

/**
* \brief Writes true and the estimated position into a result_file. Based on matched descriptors. Without weighting.
*/
int get_position_estimate(
      cv::Mat db_descriptors_all, 
      cv::Mat query_descriptors_all, 
      cv::Mat db_positions, 
      cv::Mat query_positions, 
      std::ofstream* result_file
      )
{
	cv::Mat descriptors_db, descriptors_query;

	for (int j=0; j<=query_descriptors_all.at<float>(query_descriptors_all.rows-1,0); j++)
	{

		get_descriptor_matrix(query_descriptors_all, &descriptors_query, j);

		//find the closest descriptors
		std::vector<double> distance_vec;

		for( int i = 0; i<=db_descriptors_all.at<float>(db_descriptors_all.rows-1,0); i++)
		{

			get_descriptor_matrix(db_descriptors_all, &descriptors_db, i);
			distance_vec.push_back(get_image_distance(descriptors_db, descriptors_query)); 

		}

		std::vector<int> indices;
		indices.reserve( distance_vec.size() );
		
		for ( int i = 0; i<distance_vec.size(); i++)
		{
			indices.push_back(i);
		}

		sort_vector(distance_vec, &indices);

		*result_file << query_positions.at<float>(j,0) << " " << query_positions.at<float>(j,1) << " " << query_positions.at<float>(j,2) << " " << db_positions.at<float>(indices[0],0) << " " << db_positions.at<float>(indices[0],1) << " " << db_positions.at<float>(indices[0],2)<< std::endl;

		distance_vec.clear();
	}

	result_file->close();

	return 0;
}

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
	std::ofstream* result_file
	)
{
	std::ifstream position_file (path_to_positions_file);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading pos\n" << std::endl ;
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

			cv::Mat image, descriptors_query;

			image = cv::imread(im_path, CV_LOAD_IMAGE_GRAYSCALE);

			std::vector<cv::KeyPoint> keypoints_query;
			std::vector<cv::KeyPoint> keypoints_db_i;

			int result = extract_image_data(image, detector, &keypoints_query, &descriptors_query);

			if( result > -1 ){

				std::vector<double> distance_vec;
				cv::Mat descriptors_db_i;

				for( int db_set_i = 0; db_set_i < 3; db_set_i++)
				{
					for( int i = 0; i<keypoints_db[db_set_i].size(); i++ )
					{

						get_descriptor_matrix(descriptors_db[db_set_i], &descriptors_db_i, i);
						keypoints_db_i = keypoints_db[db_set_i][i];

						//get homography
						cv::DescriptorMatcher *matcher;
						matcher = new cv::BFMatcher();

						std::vector<cv::DMatch> matches;
						matcher->match(descriptors_db_i, descriptors_query, matches);

						std::vector<cv::Point2f> points2f_db, points2f_query;
						for( int x = 0; x<matches.size(); x++)
						{
							points2f_db.push_back(keypoints_db_i[matches[x].queryIdx].pt);
							points2f_query.push_back(keypoints_query[matches[x].trainIdx].pt);
						}

						cv::Mat mask;
						cv::Mat h = cv::findHomography(points2f_db, points2f_query, CV_RANSAC,3, mask);

						// if relative # of inliers less than, then assign infty
						// if h is singular (unsuccessfull transform), assign infty
						if( (cv::sum( mask )[0]/points2f_db.size() < .20) || (cv::determinant(h) < 0.2) )
						{
							distance_vec.push_back(std::numeric_limits<double>::infinity());
						} else {
							distance_vec.push_back(get_keypoint_transform_image_distance_without_weighting(keypoints_db_i, descriptors_db_i, h, image));
						}
						matches.clear();

						}
					}

					// fill indices vector
					std::vector<int> indices;
					indices.reserve( distance_vec.size() );
					for ( int i = 0; i<distance_vec.size(); i++)
					{
					indices.push_back(i);
					}

					// sort distances to find the shortest
					sort_vector(distance_vec, &indices);

					int db_set_i, db_im_i;
					if( indices[0] < positions_db[0].rows )
					{
					db_set_i = 0;
					db_im_i = indices[0];
					} else {
					if ( indices[0] < (positions_db[0].rows + positions_db[1].rows)) 
					{
					db_set_i = 1;
					db_im_i = indices[0] - positions_db[0].rows;
					} else {
					db_set_i = 2;
					db_im_i = (indices[0] - positions_db[0].rows) - positions_db[1].rows;
					}
				}

				*result_file << pos_x << " " << pos_y << " " << yaw << " " << positions_db[db_set_i].at<float>(db_im_i,0) << " " << positions_db[db_set_i].at<float>(db_im_i,1) << " " << positions_db[db_set_i].at<float>(db_im_i,2)<< std::endl;

				distance_vec.clear();
			}
		}
	
	}

	result_file->close();

	return 0;
}


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
	)
{
	std::ifstream position_file (path_to_positions_file);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading pos\n" << std::endl ;
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

			cv::Mat image, descriptors_query;

			// load query image
			image = cv::imread(im_path, CV_LOAD_IMAGE_GRAYSCALE);

			std::vector<cv::KeyPoint> keypoints_query;
			std::vector<cv::KeyPoint> keypoints_db_i;
			std::vector<float>* weights_i;

			int result = extract_image_data(image, detector, &keypoints_query, &descriptors_query);

			if( result > -1 ){

				std::vector<double> distance_vec;
				cv::Mat descriptors_db_i;
				std::vector< std::vector<float> > distances_past_i;

				//database set
				for( int db_set_i = 0; db_set_i < weights->size(); db_set_i++)
				{
					//database image
					for( int i = 0; i<keypoints_db[db_set_i].size(); i++ )
					{

						get_descriptor_matrix(descriptors_db[db_set_i], &descriptors_db_i, i);
						keypoints_db_i = keypoints_db[db_set_i][i];
						weights_i = &((*weights)[db_set_i][i]);
						distances_past_i = distances_past->at(db_set_i);

						if( method_type==KEYPOINT_TF_WEIGHTS || method_type==IMG_TF_WEIGHTS ){
							get_image_distances_kp_transformation_weighted(method_type,descriptors_db_i, descriptors_query, keypoints_db_i, keypoints_query,image, &distance_vec, ref_val, gamma, weights_i);
						} else {
							distance_vec.push_back(get_image_distance_kp_matching_weighted(method_type, descriptors_db_i, descriptors_query, weights_i,  &distances_past_i.at(i), gamma, ref_val, alpha));
						}
						}
					}

					// fill indices vector
					std::vector<int> indices;
					indices.reserve( distance_vec.size() );
					for ( int i = 0; i<distance_vec.size(); i++)
					{
					indices.push_back(i);
					}

					// sort distances to find the shortest
					sort_vector(distance_vec, &indices);

					// compute the right set and image index
					int db_set_i, db_im_i;
					if( indices[0] < positions_db[0].rows )
					{
						db_set_i = 0;
						db_im_i = indices[0];
					} else {
						if ( indices[0] < (positions_db[0].rows + positions_db[1].rows)) 
						{
							db_set_i = 1;
							db_im_i = indices[0] - positions_db[0].rows;
						} else {
							db_set_i = 2;
							db_im_i = (indices[0] - positions_db[0].rows) - positions_db[1].rows;
						}
				}

				std::cout << "... writing to file ...\n";
				*result_file << pos_x << " " << pos_y << " " << yaw << " " << positions_db[db_set_i].at<float>(db_im_i,0) << " " << positions_db[db_set_i].at<float>(db_im_i,1) << " " << positions_db[db_set_i].at<float>(db_im_i,2)<< std::endl;

				distance_vec.clear();
			}
		}
	
	}

	result_file->close();

	return 0;
}
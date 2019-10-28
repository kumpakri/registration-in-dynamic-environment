/**
 * \file Utils.cpp
 *
 * \brief   Contains supportive functions.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 09 2018
 */

#include <fstream>
#include "Utils.h"
#include <sys/time.h> //timeval, gettimeofday

int print_out_vector(std::vector<double> vec)
{
	std::cout << "vector = " << std::endl;
	for(int i = 0; i < vec.size(); ++i)
	{
		std::cout  << vec[i] << ", ";
	}
	std::cout << std::endl;

	return 0;
}

int print_out_vector(std::vector<float> vec)
{
	std::cout << "vector = " << std::endl;
	for(int i = 0; i < vec.size(); ++i)
	{
		std::cout  << (float)vec[i] << ", ";
	}
	std::cout << std::endl;

	return 0;
}

int print_out_vector(std::vector<int> vec)
{
	std::cout << "vector = " << std::endl;
	for(int i = 0; i < vec.size(); ++i)
	{
		std::cout  << vec[i] << ", ";
	}
	std::cout << std::endl;

	return 0;
}

/**
* \return true if file exists.
*/
bool file_exists(const char *filename)
{
  std::ifstream f(filename);
  return f;
}

/**
* \brief Returns substring of source string starting at
* index `start_index` (0 for beginning of the string)
* and `num_of_chars` long.
*
* Free the substring after use!
*
* \return substring
*/
char* get_substring(char* source, int start_index, int num_of_chars)
{
	try
	{
		if(start_index+num_of_chars > strlen(source))
		{
			throw 43;
		}
		char* substring = (char*) malloc(strlen(source)+1);
		strncpy( substring, source+start_index, num_of_chars );
		substring[num_of_chars]='\0';

		return substring;
	} catch (int error){
		if (error == 43) 
		{
			std::cout << "Error " << error << ": substring start_index + num_of_chars is bigger than the source string." <<std::endl;
		}
	}
	
}

/**
* \brief Puts a double element from `pull_index` to the position 
* 		 of `push_index` and shifts remaining values in the vector.
*
* \param vec vector to execute the operation on
* \param pull_index position to take the value from
* \param push_index position to insert the value to
*/
int reorder_vector(std::vector<double>* vec, int pull_index, int push_index)
{
	if( pull_index > (*vec).size() )	
	{
		std::cout << "Error reorder_vector fcn: index is out of bounds" << std::endl ;
		return -1 ;
	}

	if( push_index > pull_index )	
	{
		std::cout << "Error reorder_vector fcn: push bigger than pull" << std::endl ;
		return -1 ;
	}

	double val_at_i = (*vec)[pull_index];
	for (int i=pull_index; i>push_index; i--)
	{
		(*vec)[i]=(*vec)[i-1];
	}
	(*vec)[push_index] = val_at_i;
	return 0;
}

/**
* \brief Puts an int element from `pull_index` to the position 
* 		 of `push_index` and shifts remaining values in the vector.
*
* \param vec vector to execute the operation on
* \param pull_index position to take the value from
* \param push_index position to insert the value to
*/
int reorder_vector(std::vector<int>* vec, int pull_index, int push_index)
{
	if( pull_index > (*vec).size() )	
	{
		std::cout << "Error reorder_vector fcn: index is out of bounds" << std::endl ;
		return -1 ;
	}

	if( push_index > pull_index )	
	{
		std::cout << "Error reorder_vector fcn: push bigger than pull" << std::endl ;
		return -1 ;
	}

	int val_at_i = (*vec)[pull_index];
	for (int i=pull_index; i>push_index; i--)
	{
		(*vec)[i]=(*vec)[i-1];
	}
	(*vec)[push_index] = val_at_i;
	return 0;
}

/**
* /brief Computes the vector of sorted indices in ascending order.
*/
int sort_vector(std::vector<double> data, std::vector<int>* indices)
{
	if( data.size() != (*indices).size() )	
	{
		std::cout << "Error sort_vector fcn: size mismatch" << std::endl ;
		return -1 ;
	}

	for (int open_position = 0; open_position<data.size()-1; open_position++)
	{
		for(double testing_val = open_position+1; testing_val<data.size(); testing_val++)
		{
			if( data[open_position] > data[testing_val] )
			{
				reorder_vector(indices, testing_val, open_position);
				reorder_vector(&data, testing_val, open_position);
			}
		}
	}

	return 0;
}

/**
* \brief Computes image distace according to chosen method. 
*
* \param method :
* 	- ABSDEV 	sum(abs(I1-I2))/N
* 	- SQUARE 	sum((I1-I2)^2)/N
* \return distance between images
*/
double image_distance_naive(std::vector<unsigned int> source, std::vector<unsigned int> target, int method)
{
	try
	{
		if( method == 0 ) // SQUARE
		{
			double long_distance = 0;
			for(int i=0; i<=source.size();i++)
			{
				long_distance = long_distance + pow((double(source[i])/255 - double(target[i])/255),2);
			}

			double distance = long_distance/source.size();
			return distance;
		} else { 
			if( method == 1 )// ABSDEV 
			{
				unsigned long int long_distance = 0;
				for(int i=0; i<=source.size();i++)
				{
					long_distance = long_distance + std::abs(source[i] - target[i]);
				}

				double distance = double(long_distance)/source.size();
				return distance;
			} else {
				throw 42;
			}
		}
	} catch(int error){
		if ( error == 42 ) {
			std::cout << "Error " << error << ": Loss function method have not been defined. Use ABSDEV or SQUARE as the parameter." <<std::endl;
		}
	}
}

/**
* \brief Computes the best matched position.
*
* \param path path to descriptor file
* \param result stringstream where to store results
*/
int find_high_scoring_position(std::vector<unsigned int> query_descriptor, char* path, std::stringstream* result)
{
	// read the descriptor file
	std::ifstream descriptor_file (path);
	if( ! descriptor_file )	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	} 

	std::vector<std::string> vec;
	std::string row;
	std::string element;
	
	std::vector<float> pos_x_vec;
	std::vector<float> pos_y_vec;
	std::vector<float> yaw_vec;
	std::vector<unsigned int> database_descriptor;
	std::vector<double> distance_vec;
	
	while( std::getline(descriptor_file,row) )
	{
		vec.clear();
		database_descriptor.clear();

		std::stringstream ss ( row );
		while ( getline( ss, element, ' ') ) 
		{
			vec.push_back(element);
		}
		pos_x_vec.push_back(std::strtof(vec[0].c_str(),0));
		pos_y_vec.push_back(std::strtof(vec[1].c_str(),0));
		yaw_vec.push_back(std::strtof(vec[2].c_str(),0));
		
		for (int i = 3; i<vec.size();i++)
		{
			database_descriptor.push_back(std::strtoul(vec[i].c_str(),0,10));
		}
		
		// compute distances & save in the vector
		distance_vec.push_back(image_distance_naive(query_descriptor, database_descriptor,SQUARE));		
	}

	// find the smallest distances
	std::vector<int> indices;
	indices.reserve( distance_vec.size() );
	for ( int i = 0; i<distance_vec.size(); i++)
	{
		indices.push_back(i);
	}

	sort_vector(distance_vec, &indices);

	// return few/the best
	for ( int i = 0; i < 1; i++ )
	{
		//score: distance_vec[indices[i]]
		*result << pos_x_vec[indices[i]] << " " << pos_y_vec[indices[i]] << " " << yaw_vec[indices[i]];
	}

	return 0;
}

/**
* \brief Computes the best matched position .
*
* \param path path to descriptor file
* \param result stringstream where to store results
*/
int find_high_scoring_position(std::vector<float> query_descriptor, char* path, std::stringstream* result)
{
	// read the descriptor file
	std::ifstream descriptor_file (path);
	if( ! descriptor_file )	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	} 

	std::vector<std::string> vec;
	std::string row;
	std::string element;
	
	std::vector<float> pos_x_vec;
	std::vector<float> pos_y_vec;
	std::vector<float> yaw_vec;
	std::vector<float> database_descriptor;
	std::vector<double> distance_vec;
	
	while( std::getline(descriptor_file,row) )
	{
		vec.clear();
		database_descriptor.clear();

		std::stringstream ss ( row );
		while ( getline( ss, element, ' ') ) 
		{
			vec.push_back(element);
		}
		pos_x_vec.push_back(std::strtof(vec[0].c_str(),0));
		pos_y_vec.push_back(std::strtof(vec[1].c_str(),0));
		yaw_vec.push_back(std::strtof(vec[2].c_str(),0));
		
		for (int i = 3; i<vec.size();i++)
		{
			database_descriptor.push_back(std::strtof(vec[i].c_str(),0));
		}
		
		// compute distances & save in the vector
		distance_vec.push_back(cv::norm(query_descriptor, database_descriptor,cv::NORM_L2));		
	}

	descriptor_file.close();

	// find the smallest distances
	std::vector<int> indices;
	indices.reserve( distance_vec.size() );
	for ( int i = 0; i<distance_vec.size(); i++)
	{
		indices.push_back(i);
	}

	sort_vector(distance_vec, &indices);

	// return few/the best
	for ( int i = 0; i < 1; i++ )
	{
		//score: distance_vec[indices[i]]
		*result << pos_x_vec[indices[i]] << " " << pos_y_vec[indices[i]] << " " << yaw_vec[indices[i]];
	}



	return 0;
}

/**
* \brief Prints out statistics for feature matching.
*
* \param statistics {#keypoints1,#keypoints2,#absIn,#relIn,error,error_man}
*/
int get_statistics_feature_matching(cv::Mat img_src, cv::Mat img_dst, cv::FeatureDetector* detector, cv::DescriptorExtractor* extractor, cv::DescriptorMatcher* matcher, std::vector<cv::Point2f> keypoints_src, std::vector<cv::Point2f>keypoints_dst, double statistics[6], cv::Mat* transformation)
{
	struct timeval tstart, tend;
	
	cv::normalize(img_src, img_src, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::normalize(img_dst, img_dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	gettimeofday(&tstart, NULL);
	std::vector<cv::KeyPoint> keypoints1, keypoints2;

	detector->detect( img_src, keypoints1 );
	detector->detect( img_dst, keypoints2 );

	statistics[0] = keypoints1.size();
	statistics[1] = keypoints2.size();

	if ( keypoints1.size() < 5 || keypoints2.size() < 5 ) {
		std::cout << "Couldn't find enough keypoints.\n";
		statistics[2] = 0;
		statistics[3] = 0;
		statistics[4] = std::numeric_limits<double>::infinity();
		statistics[5] = std::numeric_limits<double>::infinity();
		cv::hconcat( img_dst, cv::Mat::zeros(img_dst.rows, img_dst.cols, img_dst.type()), *transformation);
		return -1;
	}

	cv::Mat descriptors1, descriptors2;

	extractor->compute(img_src, keypoints1, descriptors1);
	extractor->compute(img_dst, keypoints2, descriptors2);

	std::vector<cv::DMatch> matches, matches_subset;
	
	matcher->match(descriptors1, descriptors2, matches);

	std::sort(matches.begin(), matches.end());

	std::vector<cv::Point2f> pts_src, pts_dst, pts_tf;

	for( int i = 0; i<matches.size(); i++)
	{
		pts_src.push_back(keypoints1[matches[i].queryIdx].pt);
		pts_dst.push_back(keypoints2[matches[i].trainIdx].pt);
	}

	cv::Mat mask, img_tf;

	cv::Mat h = cv::findHomography(pts_src, pts_dst, CV_RANSAC, 3, mask);

	gettimeofday(&tend,NULL);

	//save transformed images
	cv::warpPerspective(img_src, img_tf, h, img_dst.size());
	cv::hconcat( img_dst, img_tf, *transformation);

	statistics[2] = (double)cv::sum( mask )[0];
	statistics[3] = (double)(cv::sum( mask )[0])/mask.rows;
	statistics[4] = (double) (tend.tv_usec - tstart.tv_usec) / 1000000 +
     (double) (tend.tv_sec - tstart.tv_sec);
    statistics[5] = (double)get_keypoint_transform_error( keypoints_src,keypoints_dst, h, img_dst.cols, img_dst.rows);

	return 0;
}

/**
* \brief Computes error of image transformed from 
* 		 img_src to viewpoint of img_dst using 
* 		 all pixels of the image.
*/
double get_transform_error_from_all_pixels(cv::Mat img_src, cv::Mat img_dst, cv::Mat h)
{

	std::vector<cv::Point2f> all_pts_src, all_pts_tf;
	cv::Mat mask;

	cv::Mat img_out = cv::Mat::zeros(img_src.size(), CV_8UC1);


	for( int x=0; x<img_src.cols; x++ )
	{
		for( int y=0; y<img_src.rows; y++)
		{
			all_pts_src.push_back(cv::Point2f(x,y));
		}
	}

	cv::perspectiveTransform( all_pts_src, all_pts_tf, h);

	// make transformed image
	for( int i=0; i<all_pts_src.size(); i++ )
	{
		if((int)all_pts_tf[i].y>=0 && (int)all_pts_tf[i].x>=0 && (int)all_pts_tf[i].y<img_src.rows && (int)all_pts_tf[i].x<img_src.cols)   {

			img_out.at<uchar>((int)all_pts_tf[i].y,(int)all_pts_tf[i].x)=img_src.at<uchar>((int)all_pts_src[i].y,(int)all_pts_src[i].x);
		}
	}

	// compute error
	long sum = 0;
	for( int i=0; i<all_pts_tf.size(); i++ )
	{
		if((int)all_pts_tf[i].y>=0 && (int)all_pts_tf[i].x>=0 && (int)all_pts_tf[i].y<img_src.rows && (int)all_pts_tf[i].x<img_src.cols)   {

			sum = sum + pow((double)(img_out.at<uchar>((int)all_pts_tf[i].y,(int)all_pts_tf[i].x))-(double)(img_dst.at<uchar>((int)all_pts_tf[i].y,(int)all_pts_tf[i].x)),2);

		}
	}

	return sum/all_pts_tf.size();
}

/**
* \brief Loads `<cv::Point2f>` vector from the file
*/
int load_point2f_vector(char* path, std::vector<cv::Point2f>* keypoints)
{
	std::ifstream kp_file (path);
	if( ! kp_file )	
	{
		std::cout << "Error opening file for reading\n";
		return -1 ;
	}

	std::string row;
	std:: string element;
	std:: string coordinate;
	cv::Point2f kp;

	while( std::getline(kp_file, row) )
	{
		std::stringstream ss_vec (row);
		while( getline( ss_vec, element, ' ') )
		{
			std::stringstream ss_kp (element);
			getline( ss_kp, coordinate, ',');
			//casting to integer
			std::stringstream ss_x (coordinate);
			int x, y;
			ss_x >> x;
			kp.x = x;

			getline( ss_kp, coordinate, ',');
			std::stringstream ss_y (coordinate);
			ss_y >> y;
			kp.y = y;

			keypoints->push_back(kp);
		}
		
	}

	kp_file.close();

	return 0;
}

/**
* \brief Computes the error transformation from 
* 	     `img_src` to viewpoint of `img_dst` using 
* 		 manually selected keypoints.
*
* \return transform error
*/
double get_keypoint_transform_error(std::vector<cv::Point2f> keypoints_src, std::vector<cv::Point2f>keypoints_dst, cv::Mat h, int width, int height)
{
	std::vector<cv::Point2f>  keypoints_tf;

	cv::perspectiveTransform( keypoints_src, keypoints_tf, h);

	return cv::norm(keypoints_dst,keypoints_tf,cv::NORM_L2);

}

/**
* \brief Draws circles into the image `img` at points in `kps`.
*/
int drawPoints(cv::Mat img, std::vector<cv::Point2f> kps)
{
	for( int i = 0; i < kps.size(); i++)
	{
		if( kps[i].x>0 && kps[i].y >0 && kps[i].x<img.cols && kps[i].y<img.rows )
		{cv::circle(img, kps.at(i), 5, cv::Scalar(255,0,0),2,8,0);}
	}
	return 0;
}



/**
* \brief Gets a matrix of positions loaded from txt file
* matrix positions must be of the appropriate size (descriptors.rows,3)
*/
int load_position_matrix(char* path, cv::Mat *positions)
{
	std::ifstream position_file (path);
	if( ! position_file )	
	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	}

	float pos_x, pos_y, yaw;
	int i = 0;
	
	while (position_file >>  pos_x >> pos_y >> yaw)
	{
		
		positions->at<float>(i,0)= pos_x;
		positions->at<float>(i,1)= pos_y;
		positions->at<float>(i,2)= yaw;
		i++;

	}

}

bool kp_in_bounds(cv::KeyPoint kp, int width, int height)
{
	return  (kp.pt.x > 0) &&  (kp.pt.x < width) && (kp.pt.y > 0) && (kp.pt.y < height);
}

/**
* \brief Returns keypoints and descriptors and keypoints of the image.
*/
int extract_image_data(cv::Mat image, cv::FeatureDetector* detector, std::vector<cv::KeyPoint> *keypoints, cv::Mat* descriptors)
{

	detector->detect( image, *keypoints );
	if( keypoints->size() > 4 ) 
	{

		//make descriptor of the feature (128-D vector)
		cv::DescriptorExtractor* extractor;
		extractor = new cv::SiftDescriptorExtractor();

		extractor->compute(image, *keypoints, *descriptors);

	} else {
		std::cout << "Warning: Not enough keypoints.\n"; 
		return -1;
	}
	return 0;
}

/**
* \brief Gets submatrix with first column of value `id`.
*/
int get_descriptor_matrix(cv::Mat descriptors_all, cv::Mat* descriptors, float id)
{
	int index_begin, index_end, index;
	bool index_found;

	if ( id<= descriptors_all.at<float>(descriptors_all.rows-1,0))
	{
		index = 0;
		index_found = false;
		while(!index_found)
		{
			if( descriptors_all.at<float>(index,0) == id )
			{
				// index of the first row starting with id
				index_begin = index;
				index_found = true;
			}
			index++;
		}

		index_found = false;
		while(!index_found)
		{
			if( descriptors_all.at<float>(index,0) != id )
			{
				//index of the last row starting with id
				index_end = index-1;
				index_found = true;
			}
			index++;
		}
	} else { std::cout << "Error: the id is bigger than number of records.\n"; return -1;}

	*descriptors = descriptors_all(cv::Range(index_begin,index_end+1), cv::Range(1,129));

	return 0;
}

/**
* \return concated descriptor matrix
*/
int vconcat_descriptor_matrix(cv::Mat descriptors_mat1, cv::Mat descriptors_mat2, cv::Mat *descriptors_all)
{
	cv::Mat temp;
	descriptors_mat2.copyTo(temp);
	float last_id = descriptors_mat1.at<float>(descriptors_mat1.rows-1,0);
	float current_id = temp.at<float>(0,0);


	for (int i = 0; i < temp.rows; i++ )
	{

		if( current_id < temp.at<float>(i,0) ) 
		{
			current_id = temp.at<float>(i,0);
			last_id ++;
		}
		
		temp.at<float>(i,0) = last_id+1;

	}

	cv::vconcat(descriptors_mat1, temp, *descriptors_all);
}

/**
* \brief Creates a vector of weights set to '1' for a set of database images.
*/
int make_weights_vector(cv::Mat descriptors, std::vector< std::vector<float> > *weights)
{

	std::vector<float> v;
	float id = 0;
	for( int i = 0; i<descriptors.rows; i++)
	{
		if ( id == descriptors.at<float>(i,0) )
		{
			v.push_back(1.0);
		} else {
			weights->push_back(v);
			v.clear();
			v.push_back(1.0);
			id = descriptors.at<float>(i,0);
		}
	}

	weights->push_back(v);

	return 0;
}

/**
* \brief Loads weights from file.
*/
int load_weights(char* path, std::vector< std::vector<float> > *weights)
{
	std::ifstream weights_file (path);
	if( ! weights_file )	
	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	}

	std::string row;
	std::string element;
	std::vector<float> vec;
	while( std::getline(weights_file,row) )
	{
		vec.clear();
		std::stringstream ss ( row );
		while ( getline( ss, element, ' ') ) 
		{
			vec.push_back(std::strtod(element.c_str(),0));
		}
		
		weights->push_back(vec);

	}
	weights_file.close();
	return 0;
}

/**
* \brief Loads keypoints from file.
*/
int load_keypoints(char* path, std::vector< std::vector<cv::KeyPoint> > *keypoints)
{
	std::ifstream keypoint_file (path);
	if( ! keypoint_file )	
	{
		std::cout << "Error opening file for reading\n" << std::endl ;
		return -1 ;
	}

	std::string line;
	std::string point;
	std::string coordinate;
	float x,y, size;
	std::vector<cv::KeyPoint> vec;
	while( std::getline(keypoint_file, line) )
	{
		vec.clear();
		std::stringstream ss_line ( line );
		while ( getline( ss_line, point, ';') ) 
		{
			std::stringstream ss_point ( point );
			getline( ss_point, coordinate, ' '); //x
			x = std::strtod(coordinate.c_str(),0);
			getline( ss_point, coordinate, ' '); //y
			y = std::strtod(coordinate.c_str(),0);
			getline( ss_point, coordinate, ' '); //size
			size = std::strtod(coordinate.c_str(),0);

			vec.push_back(cv::KeyPoint(x,y,size));
		}
		
		keypoints->push_back(vec);

	}
	keypoint_file.close();
	return 0;
}

int get_descriptors(char* path, cv::Mat *descriptors)
{
	cv::FileStorage db_descriptor1_file(path,cv::FileStorage::READ);
	db_descriptor1_file["matDescriptors"] >> *descriptors;
	db_descriptor1_file.release();

	return 0;
}

int add_into_database(
	std::vector<cv::Mat>* descriptors_db, 
	std::vector<cv::Mat>* positions_db, 
	std::vector< std::vector< std::vector<cv::KeyPoint> > >* keypoints_db,
	std::vector< std::vector< std::vector<float> > > *weights, 
	std::vector< std::vector< std::vector<float> > > *distances_past, 
	char* path_descr, 
	char* path_pos,
	char* kp_path)
{
	cv::Mat descriptors;
	std::vector< std::vector<float> > weight;

	get_descriptors(path_descr, &descriptors);
	make_weights_vector(descriptors, &weight);
	descriptors_db->push_back(descriptors);
	weights->push_back(weight);
	distances_past->push_back(weight);

	cv::Mat positions = cv::Mat::zeros(descriptors.at<float>(descriptors.rows-1,0)+1,3,CV_32F);
	load_position_matrix(path_pos, &positions);
	positions_db->push_back(positions);

	std::vector< std::vector<cv::KeyPoint> > keypoints;
	load_keypoints(kp_path, &keypoints);
	keypoints_db->push_back(keypoints);
}

/**
* \brief Loads descriptors, positions and keypoints and 
* 	  	 creates weights and past distances vectors.
*/
int load_database_with_weights(
	std::vector<cv::Mat>* descriptors_db, 
	std::vector<cv::Mat>* positions_db, 
	std::vector< std::vector< std::vector<cv::KeyPoint> > >* keypoints_db, 
	std::vector< std::vector< std::vector<float> > > *weights, 
	std::vector< std::vector< std::vector<float> > > *distances_past)
{

	char* desc_paths[3] = {"src/data_acquisition/data/arena_data_kp_tf/set2_meas1_descriptors.txt","src/data_acquisition/data/arena_data_kp_tf/set2_meas2_descriptors.txt","src/data_acquisition/data/arena_data_kp_tf/set2_meas3_descriptors.txt"};

	char* pos_paths[3] = {"src/data_acquisition/data/arena_data_kp_tf/set2_meas1_positions.txt", "src/data_acquisition/data/arena_data_kp_tf/set2_meas2_positions.txt", "src/data_acquisition/data/arena_data_kp_tf/set2_meas3_positions.txt"};

	char* kp_paths[3] = {"src/data_acquisition/data/arena_data_kp_tf/set2_meas1_keypoints.txt", "src/data_acquisition/data/arena_data_kp_tf/set2_meas2_keypoints.txt", "src/data_acquisition/data/arena_data_kp_tf/set2_meas3_keypoints.txt"};

	for (int i = 0; i < sizeof(desc_paths)/sizeof(char*); i++ )
	{
		add_into_database(descriptors_db, positions_db, keypoints_db, weights, distances_past, desc_paths[i], pos_paths[i], kp_paths[i]);
	}

    return 0;
}



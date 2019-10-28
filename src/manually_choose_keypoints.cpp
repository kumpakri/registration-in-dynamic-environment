/**
 * \file manually_choose_keypoints.cpp
 *
 * \brief 	Lets user define keypoints 
 *		    in the image and saves them
 *	        in file.
 *
 * \param 1 source image file path
 * \param 2 target image file path
 * \param 3 output path to source image keypoints
 * \param 4 output path to target image keypoints
 * Optional:
 * \param 5 output path to source image with keypoints plotted
 * \param 6 output path to target image with keypoints plotted
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: July 17 2018
 */

#include "Utils.h"
#include "Manual_keypoints.h"

int main(int argc, char **argv) 
{

	// check if arguments ok
	if ( ! (argc>4 )  )
	{
		std::cout << "Error: missing arguments.\n";
		return -1;
	}

	if ( ! (file_exists(argv[1]) && file_exists(argv[2]) )  )
	{
		std::cout << "Error: invalid file path.\n";
		return -1;
	}

	cv::Mat img_src, img_dst;

	img_src = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
	img_dst = cv::imread(argv[2],CV_LOAD_IMAGE_GRAYSCALE);

	std::vector<cv::Point2f> keypoints_src, keypoints_dst;
	cv::Mat kp_img_src, kp_img_dst;

	get_manual_keypoints( img_src, img_dst, &keypoints_src, &keypoints_dst, &kp_img_src, &kp_img_dst);
	save_kp(argv[3], keypoints_src);
	save_kp(argv[4], keypoints_dst);



	if(argc > 6) {
		cv::imwrite(argv[5], kp_img_src);
		cv::imwrite(argv[6], kp_img_dst);
	}

	return 0;
}
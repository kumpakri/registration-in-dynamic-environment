/**
 * \file Manual_keypoints.cpp
 *
 * \brief 	Contains functions that allow  for manual 
 * 			keypoint selection.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: April 14 2018
 */

#include <fstream>
#include "Manual_keypoints.h"


struct MouseParams
{
	cv::Mat img;
	std::vector<cv::Point2f>* keypoints;
	const char* window_name;
};


/**
* \brief Saves <cv::Point2f> vector into the file.
*
* \param path 		path to a file
* \param keypoints 	points to save
*/
int save_kp(char* path, std::vector<cv::Point2f> keypoints)
{
	std::ofstream kp_file (path);
	if( !kp_file ){
		std::cout << "Error opening file for writing\n";
		return -1;
	}

	for(int i = 0; i < keypoints.size(); ++i)
	{
		kp_file  << keypoints[i].x <<"," <<keypoints[i].y << " ";
	}

	return 0;
}

/**
* \brief Call back function detecting left mouse button click 
*  		 saving cursor coordinates (inside the image) into the vector.
*/
static void mouse_callback_rough(int event, int x, int y, int flags, void* param)
{
	if( event != cv::EVENT_LBUTTONDOWN )
	return;
	
	MouseParams* mp = (MouseParams*)param;
	cv::Mat image = (cv::Mat)mp->img;
	std::vector<cv::Point2f>* keypoints = (std::vector<cv::Point2f>*)mp->keypoints;
	const char* window_name = (const char*)mp->window_name;

	if( x<40 || y<40 || x>image.cols-40 || y>image.rows-40 )
	return;

	keypoints->push_back(cv::Point2f(x,y));

	cv::circle(image, cv::Point(x,y), 5, cv::Scalar(255,0,0),2,8,0);
	cv::imshow(window_name, image);
}

/**
* \brief Call back function detecting left mouse button click saving 
*        the last cursor coordinates (inside the image).
*/
static void mouse_callback_detail(int event, int x, int y, int flags, void* param)
{
	if( event != cv::EVENT_LBUTTONDOWN )
	return;
	
	MouseParams* mp = (MouseParams*)param;
	cv::Mat image = (cv::Mat)mp->img;
	std::vector<cv::Point2f>* keypoints = (std::vector<cv::Point2f>*)mp->keypoints;
	const char* window_name = (const char*)mp->window_name;

	keypoints->at(0)=cv::Point2f(x,y);

	cv::Mat tmp_img = image.clone();
	cv::circle(tmp_img, cv::Point(x,y), 5, cv::Scalar(255,0,0),2,8,0);
	cv::imshow(window_name, tmp_img);
}

/**
* \brief Allows to preciesly select keypoints in the images in
* two steps 
*	- rough selection of the keypoint and 
*	- precise selection on zoomed in area of the image.
*/
int get_manual_keypoints(cv::Mat img_src, cv::Mat img_dst, std::vector<cv::Point2f>* keypoints_src, std::vector<cv::Point2f>* keypoints_dst )
{
	const char* window_name_src="source image";
	const char* window_name_dst="target image";

	MouseParams mp_src, mp_dst;
	mp_src.img = img_src.clone();
	mp_src.keypoints = keypoints_src;
	mp_src.window_name = window_name_src;
	mp_dst.img = img_dst.clone();
	mp_dst.keypoints = keypoints_dst;
	mp_dst.window_name = window_name_dst;

	cv::namedWindow( window_name_src, cv::WINDOW_AUTOSIZE );
	cv::namedWindow( window_name_dst, cv::WINDOW_AUTOSIZE );

	cv::setMouseCallback(window_name_src, mouse_callback_rough,  (void*)&mp_src);
	cv::setMouseCallback(window_name_dst, mouse_callback_rough, (void*)&mp_dst);

	cv::imshow(window_name_src, img_src);
	cv::imshow(window_name_dst, img_dst);

	cv::waitKey();
	cv::destroyAllWindows();

	cv::Rect roi_src, roi_dst;
	cv::Mat img_crop_src, img_crop_dst, img_tmp_src, img_tmp_dst;
	std::vector<cv::Point2f> kp_temp_src, kp_temp_dst;
	cv::Point2f kp_cur_src, kp_cur_dst;

	for(int i=0; i<keypoints_src->size() && i<keypoints_dst->size(); i++)
	{
		// set cropping borders
		roi_src.x = keypoints_src->at(i).x-40;
		roi_src.y = keypoints_src->at(i).y-40;

		roi_dst.x = keypoints_dst->at(i).x-40;
		roi_dst.y = keypoints_dst->at(i).y-40;

		roi_src.width = 80;
		roi_src.height = 80;

		roi_dst.width = 80;
		roi_dst.height = 80; 

		// enlarge selection
		cv::resize(img_src(roi_src), img_crop_src, cv::Size(), 10, 10, CV_INTER_CUBIC );
		cv::resize(img_dst(roi_dst), img_crop_dst, cv::Size(), 10, 10, CV_INTER_CUBIC );

		cv::namedWindow( window_name_src, cv::WINDOW_AUTOSIZE );
		cv::namedWindow( window_name_dst, cv::WINDOW_AUTOSIZE );

		mp_src.img = img_crop_src;
		mp_dst.img = img_crop_dst;

		// take relevant keypoint
		kp_temp_src.push_back(keypoints_src->at(i));
		kp_temp_dst.push_back(keypoints_dst->at(i));

		kp_cur_src = keypoints_src->at(i);
		kp_cur_dst = keypoints_dst->at(i);

		mp_src.keypoints = &kp_temp_src;
		mp_dst.keypoints = &kp_temp_dst;

		cv::setMouseCallback(window_name_src, mouse_callback_detail,  (void*)&mp_src);
		cv::setMouseCallback(window_name_dst, mouse_callback_detail, (void*)&mp_dst);

		//draw current keypoint position
		img_tmp_src = img_crop_src.clone();
		img_tmp_dst = img_crop_dst.clone();
		cv::circle(img_tmp_src, cv::Point(400,400), 5, cv::Scalar(255,0,0),2,8,0);
		cv::circle(img_tmp_dst, cv::Point(400,400), 5, cv::Scalar(255,0,0),2,8,0);

		cv::imshow(window_name_src, img_tmp_src );
		cv::moveWindow(window_name_src, 0, 0);
		cv::imshow(window_name_dst, img_tmp_dst );
		cv::moveWindow(window_name_dst, 810, 0);

		cv::waitKey();

		//save preciese position
		keypoints_src->at(i) = cv::Point2f((int)(mp_src.keypoints->at(0).x/10)+kp_cur_src.x-40,(int)(mp_src.keypoints->at(0).y/10)+kp_cur_src.y-40);

		keypoints_src->at(i) = cv::Point2f((int)(mp_src.keypoints->at(0).x/10)+kp_cur_src.x-40,(int)(mp_src.keypoints->at(0).y/10)+kp_cur_src.y-40);

		cv::destroyAllWindows();
		kp_temp_src.clear();
		kp_temp_dst.clear();

	}

	return 0;

}

/**
* \brief Allows to preciesly select keypoints in the images in
* two steps 
*	- rough selection of the keypoint and 
*	- precise selection on zoomed in area of the image.
* returns keypoints and images with keypoints plotted
*/
int get_manual_keypoints(cv::Mat img_src, cv::Mat img_dst, std::vector<cv::Point2f>* keypoints_src, std::vector<cv::Point2f>* keypoints_dst, cv::Mat* kp_img_src, cv::Mat* kp_img_dst )
{
	const char* window_name_src="source image";
	const char* window_name_dst="target image";

	MouseParams mp_src, mp_dst;
	mp_src.img = img_src.clone();
	mp_src.keypoints = keypoints_src;
	mp_src.window_name = window_name_src;
	mp_dst.img = img_dst.clone();
	mp_dst.keypoints = keypoints_dst;
	mp_dst.window_name = window_name_dst;

	cv::namedWindow( window_name_src, cv::WINDOW_AUTOSIZE );
	cv::namedWindow( window_name_dst, cv::WINDOW_AUTOSIZE );

	cv::setMouseCallback(window_name_src, mouse_callback_rough,  (void*)&mp_src);
	cv::setMouseCallback(window_name_dst, mouse_callback_rough, (void*)&mp_dst);

	cv::imshow(window_name_src, img_src);
	cv::imshow(window_name_dst, img_dst);

	cv::waitKey();
	cv::destroyAllWindows();

	cv::Rect roi_src, roi_dst;
	cv::Mat img_crop_src, img_crop_dst, img_tmp_src, img_tmp_dst;
	std::vector<cv::Point2f> kp_temp_src, kp_temp_dst;
	cv::Point2f kp_cur_src, kp_cur_dst;

	for(int i=0; i<keypoints_src->size() && i<keypoints_dst->size(); i++)
	{
		// set cropping borders
		roi_src.x = keypoints_src->at(i).x-40;
		roi_src.y = keypoints_src->at(i).y-40;

		roi_dst.x = keypoints_dst->at(i).x-40;
		roi_dst.y = keypoints_dst->at(i).y-40;

		roi_src.width = 80;
		roi_src.height = 80;

		roi_dst.width = 80;
		roi_dst.height = 80; 

		// enlarge selection
		cv::resize(img_src(roi_src), img_crop_src, cv::Size(), 10, 10, CV_INTER_CUBIC );
		cv::resize(img_dst(roi_dst), img_crop_dst, cv::Size(), 10, 10, CV_INTER_CUBIC );

		cv::namedWindow( window_name_src, cv::WINDOW_AUTOSIZE );
		cv::namedWindow( window_name_dst, cv::WINDOW_AUTOSIZE );

		mp_src.img = img_crop_src;
		mp_dst.img = img_crop_dst;

		// take relevant keypoint
		kp_temp_src.push_back(keypoints_src->at(i));
		kp_temp_dst.push_back(keypoints_dst->at(i));

		kp_cur_src = keypoints_src->at(i);
		kp_cur_dst = keypoints_dst->at(i);

		mp_src.keypoints = &kp_temp_src;
		mp_dst.keypoints = &kp_temp_dst;

		cv::setMouseCallback(window_name_src, mouse_callback_detail,  (void*)&mp_src);
		cv::setMouseCallback(window_name_dst, mouse_callback_detail, (void*)&mp_dst);

		//draw current keypoint position
		img_tmp_src = img_crop_src.clone();
		img_tmp_dst = img_crop_dst.clone();
		cv::circle(img_tmp_src, cv::Point(400,400), 5, cv::Scalar(255,0,0),2,8,0);
		cv::circle(img_tmp_dst, cv::Point(400,400), 5, cv::Scalar(255,0,0),2,8,0);

		cv::imshow(window_name_src, img_tmp_src );
		cv::moveWindow(window_name_src, 0, 0);
		cv::imshow(window_name_dst, img_tmp_dst );
		cv::moveWindow(window_name_dst, 810, 0);

		cv::waitKey();

		//save preciese position
		keypoints_src->at(i) = cv::Point2f((int)(mp_src.keypoints->at(0).x/10)+kp_cur_src.x-40,(int)(mp_src.keypoints->at(0).y/10)+kp_cur_src.y-40);

		keypoints_src->at(i) = cv::Point2f((int)(mp_src.keypoints->at(0).x/10)+kp_cur_src.x-40,(int)(mp_src.keypoints->at(0).y/10)+kp_cur_src.y-40);

		cv::destroyAllWindows();
		kp_temp_src.clear();
		kp_temp_dst.clear();

	}


	img_src.copyTo(*kp_img_src);
	img_dst.copyTo(*kp_img_dst);

	//show keypoints
	for(int i=0; i<keypoints_src->size() && i<keypoints_dst->size(); i++)
	{
		cv::circle(*kp_img_src, keypoints_src->at(i), 5, cv::Scalar(255,0,0),2,8,0);
		cv::circle(*kp_img_dst, keypoints_dst->at(i), 5, cv::Scalar(255,0,0),2,8,0);
	}


	return 0;

}
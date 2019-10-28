# Life-long Visual Localization of a Mobile Robot in Changing Environments


This project focuses on the development of methods suitable for the localization task in changing environments using the life-long learning approach. The methods implemented in this thesis are based on keypoint descriptor weighted distances. Three different methods of weighting were implemented: distance vector normalization, past distance subtraction and global reference value. Two approaches to localization were tested: matched keypoints and transformed keypoints. The methods were evaluated on experimental data that have been recorded in an arena constructed for this purpose. The project is implemented in C++ in the ROS environment.

## Datasets

## Runnable tasks

There are several files containing main method that can be run using ROS.

### Naive localization
Naive localization method is implemented in file naive_localization.cpp. It uses a naive descriptor files. Each row of the file represents one image. The row contain image position and its descriptor in format:

    pos_x   pos_y   angle   naive_descriptor 

where `pos_x` is the x coordinate, `pos_y` in the y coordinate and `angle` is the angle coordinate of the position and `naive descriptor` is the descriptor vector. The file can be obtained by running command:

    $ rosrun data_acquisition data_acquisition_make_naive_descriptors_node <pos_path> <img_path> <descr_path>

| Argument       | Description                            | 
| :---           |  :---                                  |
| `<pos_path>`   | path to position file                  | 
| `<img_path>`   | path to image folder                   |
| `<descr_path>` | output path for naive descriptor file  |

The estimations of naive localization method can be obtained by running command:

    $ rosrun data_acquisition data_acquisition_naive_localization_node <test_path> <train_path> <estimation_path>
    
| Argument            | Description                                         |
| :---                | :---                                                |
| `<test_path>`       | path to naive descriptor file of a test dataset     |
| `<train_path>`      | path to naive descriptor file of a training dataset |
| `<estimation_path>` |   output path for position estimation file          |

The position estimation file contains rows of positions in the format:

    true_x   true_y   true_angle   estimated_x   estimated_y   estimated_angle
The example naive descriptor files are located in the  `/data/example_naive_localization` folder.

### Bag of Words with SIFT descriptors localization
The first step in BoW method is to obtain descriptors file of a training dataset. In our case we use SIFT descriptors. SIFT descriptors file can be obtained by running following command in terminal:

    $ rosrun data_acquisition data_acquisition_make_sift_descriptors_node <nFeatures> <nOctaveLayers> <contrastThreshold> <edgeThreshold> <sigma>  <pos_path> <img_path> <used_pos_path> <descr_path>

| Argument              | Description                         |
| :---                  | :---                                |
| `<nFeatures>`         | SIFT parameter                      |
| `<nOctaveLayers>`     | SIFT parameter                      |
| `<contrastThreshold>` | SIFT parameter                      |
| `<edgeThreshold>`     | SIFT parameter                      |
| `<sigma>`             | SIFT parameter                      |
| `<pos_path>`          | path to positions file              |
| `<img_path>`          | path to image folder                |
| `<used_pos_path>`     | output path for used positions file |
| `<descr_path>`        | output path for descriptors file    |


The file consists of matrices of descriptors for each image in the dataset. The BoW localization method uses BoW descriptor files in format:

    pos_x   pos_y   angle   bow_descriptor 

where `pos_x` is the x coordinate, `pos_y` in the y coordinate and `angle` is the angle coordinate of the position and `bow descriptor` is the descriptor vector. The file can be obtained by running command:

    $ rosrun data_acquisition data_acquisition_make_bow_descriptors_node <nFeatures> <nOctaveLayers> <contrastThreshold> <edgeThreshold> <sigma> <train_path> <pos_path> <img_path> <bow_descr_path>

| Argument              | Description                                |
| :---                  | :---                                       |
| `<nFeatures>`         | SIFT parameter                             |
| `<nOctaveLayers>`     | SIFT parameter                             |
| `<contrastThreshold>` | SIFT parameter                             |
| `<edgeThreshold>`     | SIFT parameter                             |
| `<sigma>`             | SIFT parameter                             |
| `<train_path>`        | path to train dataset SIFT descriptor file |
| `<pos_path>`          | path to positions file                     |
| `<img_path>`          | path to image folder                       |
| `<descr_path>`        | output path for descriptors file           |

The BoW localization implemented in file bow_localization.cpp can be run from terminal using command

    $ rosrun data_acquisition data_acquisition_BoW_localization_node <test_path> <train_path> <out_path>
    
| Argument              | Description                                |
| :---                  | :---                                       |
| `<test_path>`         | path to test dataset BoW descriptor file   |
| `<train_path>`        | path to train dataset BoW descriptor file  |
| `<out_path>`          | output path for position estimation file   |

The position estimation file contains rows of positions in the format:

    true_x   true_y   true_angle   estimated_x   estimated_y   estimated_angle
The example BoW localization files are located in the  `/data/example_BoW_localization` folder.

### Keypoint matching for all descriptor setting
Procedure implemented in file show_all_descriptor_settings.cpp can be run from terminal using command

    $ rosrun data_acquisition data_acquisition_show_matches_node <path_to_img1> <path_to_img2> <image_path>

It takes the images from `<path_to_img1>` and `<path_to_img2>` argument and find matches between them using SIFT, SURF, STAR, MSER and FAST feature detector. Draws matches into image and saves it at `<image_path>`. Prints out statistics. Setting of the detectors is specified in the source file. Image pairs (`source_*.png` & `target_*.png`) that can be used to test this procedure are located in `/data/image_pairs`folder.

The example keypoint matching files are located in the  `/data/example_show_matches` folder.

### Manual keypoint selection
To get file of keypoints manually selected in a picture run command:

    $  rosrun data_acquisition data_acquisition_man_keypoints_node <src_path> <dst_path> <src_kp_path> <dst_kp_path> <src_kp_img_path> <dst_kp_img_path>

| Argument              | Description                                        |
| :---                  | :---                                               |
| `<src_path>`          | path to source image                               |
| `<dst_path>`          | path to target image                               |
| `<src_kp_path>`       | output path to keypoint file of the source image   |
| `<dst_kp_path>`       | output path to keypoint file of the target image   |
| `<src_kp_img_path>`   | (optional) output path to plotted keypoints image of the source image |
| `<dst_kp_img_path>`   | (optional) output path to plotted keypoints image of the target image |

After running the command two windows will appear. User has to chose carefully the same keypoints in the same order in both images. After desired number of keypoints are selected the user hits Enter. One by one each selected keypoint will appear in zoomed-in picture for both source and target image. The user can adjust arbitrarily many times the currently showing keypoint. After the user is satisfied with the keypoint position in both source and target images, the user hits Enter.

The keypoint file has following format:

    kp[1].x,kp[1].y  kp[2].x,kp[2].y kp[3].x,kp[3].y (...)

The example manual keypoint selection files are located in the  `/data/example_manual_kp` folder.

### Feature detector evaluation
File feature_detector_evaluation.cpp contains implementation of feature detector evaluation procedure. It can be run from the terminal using command

    $ rosrun data_acquisition data_acquisition_eval_node <output_path>

This procedure loads pairs of images defined in the source file and loads manually selected keypoints of those images also defined in the source file. Both big and small (half size of the big) sizes of images are used. For several settings of FAST and SIFT feature detector computes absolute number of inliers, relative number of inliers, number of keypoints, processing time and transformation error. Prints the statistics information into terminal and saves transformations images into `<output_path>` folder. 

Each image filename contains a name ofthe used feature detector and its setting. The letter `B` at the and of the name denotes the image of original size and the letter `s` denotes the image of half the size. 

The statistics information can be saved into `filename.txt` file using command

    $ rosrun data_acquisition data_acquisition_eval_node <output_path> >> filename.txt

The example of feature detectors evaluation is located in the `data/example_detector_evaluation` folder.
    
### Difference change detection
Difference change detection method implemented in file diff_change_detection.cpp can be run in terminal using command
    
     $ rosrun data_acquisition data_acquisition_diff_change_node <img_src_path> <img_dst_path> <threshold> <diff_path> <mask_path>
    
| Argument              | Description                                             |
| :---                  | :---                                                    |
| `<img_src_path>`      | file path to source image                               |
| `<img_dst_path>`      | file path to target image                               |
| `<threshold>`         | threshold value                                         |
| `<diff_path>`         | file path to difference image created by this procedure |
| `<mask_path>`         | file path to mask created by this procedure             |

Creates difference image and mask using the threshold from argument `<threshold>`. Stores the difference image at location `<diff_path>` and stores mask at location `<mask_path>`.

The example change detection with difference image is located in `/data/example_change_detection` folder.

### Keypoint change detection
Keypoints change detection method implemented in file keypoint_change_detection.cpp can be run in terminal using command
    
     $ rosrun data_acquisition data_acquisition_kp_change_node <img_src_path> <img_dst_path> <out_img_path> 
    
| Argument              | Description                       |
| :---                  | :---                              |
| `<img_src_path>`      | file path to source image         |
| `<img_dst_path>`      | file path to target image         |
| `<out_img_path>`      | file path to output image         |

Computes descriptor distances between images at `<img_src_path>` and `<img_dst_path>`. Draws the distances into image as circles (the bigger distance the bigger circle) and stores the output image at location `<out_img_path>`.

The example keypoint change detection is located in `/data/example_change_detection` folder.

### Position estimation using weighting
Procedure implemented in file testing.cpp performs position estimation using any of the weighted methods. The procedure can be run from terminal using command

    $ rosrun data_acquisition data_acquisition_test_node
    
The setting is located inside the testing.cpp file. Example of setting use:

    char* output_path = "src/data_acquisition/data/testing/";
	int method_type = GLOBAL_REF;
	/*specify label for each query record (s- set, m- measurement)*/
	char* labels[3] = {"s3m1","s3m2","s3m3"};
	float gamma[2] = {0.9, 0.95};
	float ref_val[3] = {400, 500, 600};
	float alpha[1]={300};
	int n_of_runs = 10;
	
`output_path` points to the folder where to store results. 

`method_type` defines what method to use. Possible values:

| Value of `method_type` | Description                              | Using parameters   |
| :---                   | :---                                     | :---               |
| KEYPOINT_TF_WEIGHTS    | for keypoint tranformation method        | `gamma`, `ref_val` |
| GLOBAL_REF             | for global reference value method        | `gamma`, `ref_val` |
| PAST_DISTANCE          | for past distance subtraction method     | `gamma`, `alpha`   |
| VECTOR_NORMALIZATION   | for distance vector normalization method | `gamma`            |
| IMG_TF_WEIGHTS         | for image transformation method          | `gamma`, `ref_val` |

`labels` array of labels for each used record

`gamma` array of gamma values to be tested

`ref_val` array of reference values to be tested 

`alpha` array of alpha values to be tested

`n_of_runs` number of runs (how many time to repeat all tests)

> Arrays of parameters that are not used by the chosen method must be kept of size `1`.

## Structure of the package


├── data<br/>
│   &emsp;├── arena_data<br/>
│   &emsp;├── example_BoW_localization<br/>
│   &emsp;├── example_change_detection<br/>
│   &emsp;├── example_detector_evaluation<br/>
│   &emsp;├── example_manual_kp <br/>
│   &emsp;├── example_naive_localization<br/>
│   &emsp;├── example_show_matches<br/>
│   &emsp;├── image_pairs<br/>
│   &emsp;├── processed_data<br/>
│   &emsp;├── test<br/>
│   &emsp;└── positions.txt<br/>
├── git<br/>
├── include<br/>
│   &emsp;├── Data_processing.h<br/>
│   &emsp;├── Estimation.h<br/>
│   &emsp;├── Interpolation.h<br/>
│   &emsp;├── Manual_keypoints.h<br/>
│   &emsp;├── Utils.h<br/>
│   &emsp;└── spline.h<br/>
├── launch<br/>
│   &emsp;├── run_gmapping.launch<br/>
│   &emsp;└── turtlebot_gazebo_simulation.launch<br/>
├── src<br/>
│   &emsp;├── Data_processing.cpp<br/>
│   &emsp;├── Estimation.cpp<br/>
│   &emsp;├── Interpolation.cpp<br/>
│   &emsp;├── Manual_keypoints.cpp<br/>
│   &emsp;├── Utils.cpp<br/>
│   &emsp;├── bow_localization.cpp<br/>
│   &emsp;├── camera_listener.cpp<br/>
│   &emsp;├── data_recorder.cpp<br/>
│   &emsp;├── diff_change_detection.cpp<br/>
│   &emsp;├── feature_detector_evaluation.cpp<br/>
│   &emsp;├── keypoint_change_detection.cpp<br/>
│   &emsp;├── make_bow_descriptor_file.cpp<br/>
│   &emsp;├── make_naive_descriptor_file.cpp<br/>
│   &emsp;├── make_sift_descriptor_file.cpp<br/>
│   &emsp;├── manually_choose_keypoints.cpp<br/>
│   &emsp;├── map_listener.cpp<br/>
│   &emsp;├── naive_localization.cpp<br/>
│   &emsp;├── odom_listener.cpp<br/>
│   &emsp;├── show_all_descriptor_settings.cpp<br/>
│   &emsp;└── testing.cpp<br/>
├── CMakeList.txt<br/>
├── Documentation.html<br/>
├── Doxyfile<br/>
├── README.md<br/>
└── package.xml<br/>


/**
 * \file Interpolation.h
 *
 * \brief 	Declares methods for position 
 * 			and angle interpolation.
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 *
 * Created on: March 09 2018
 */
#ifndef INTERPOLATION_H
#define INTERPOLATION_H

// DEFINE interpolation method
//#define COMPUTE_DIRECTION
//#define QUATERNION_INTERPOLATION
#define CONTINUOUS_REPRESENTATION
//#define SINUS_REPRESENTATION

/**
* \brief Writes into a file interpolated positions.
*/
int get_interpolated_positions(char* path);


#endif //INTERPOLATION_H
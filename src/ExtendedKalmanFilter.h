/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/**
 * @file ExtendedKalmanFilter.h
 * @brief Standard extended Kalman filter implementation
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <opencv2/core.hpp>

/**
 * @brief Standard EKF, see http://en.wikipedia.org/wiki/Extended_Kalman_filter
 */
class ExtendedKalmanFilter{

public:

    /**
     * @brief Default constructor
     */
    ExtendedKalmanFilter();

    /**
     * @brief Full constructor
     *
     * @param dynamParams Dimensionality of the state
     * @param measureParams Dimensionality of the observation
     * @param type CV_32F or CV_64F
     */
    ExtendedKalmanFilter(int dynamParams, int measureParams, int type = CV_32F);

    /**
     * @brief Re-initializes the filter, previous content is destroyed
     *
     *  @param dynamParams New dimensionality of the state
     * @param measureParams New dimensionality of the observation
     * @param type CV_32F or CV_64F
     */
    void init(int dynamParams, int measureParams, int type = CV_32F);

    /**
     * @brief Performs predict step
     *
     * @param process Process value calculated from previous a posteriori state estimate and control input i.e f(x'(k-1|k-1), u(k-1))
     *
     * @return Predicted (a priori) state estimate
     */
    cv::Mat const& predict(cv::Mat const& process);

    /**
     * @brief Performs update state
     *
     * @param observation Observation vector i.e z(k)
     * @param predictedObservation Observation calculated from a priori state estimate i.e h(x'(k|k-1))
     *
     * @return Updated (a posteriori) state estimate
     */
    cv::Mat const& correct(cv::Mat const& observation, cv::Mat const& predictedObservation);

    cv::Mat statePre;               ///< Predicted state                                x'(k|k-1) := f(x'(k-1|k-1), u(k-1))
    cv::Mat processNoiseCov;        ///< Process noise covariance matrix                Q(k-1)
    cv::Mat transitionMatrix;       ///< State transition matrix i.e process Jacobian   F(k-1) := (delf/delx)(x'(k-1|k-1), u(k-1))
    cv::Mat errorCovPre;            ///< A priori error estimate covariance matrix      P'(k|k-1) := F(k-1)*P(k-1|k-1)*F(k-1)t + Q(k-1)
    cv::Mat observationMatrix;      ///< Observation matrix i.e observation Jacobian    H(k):= (delh/delx)(x'(k|k-1))
    cv::Mat observationNoiseCov;    ///< Observation noise covariance matrix            R(k)
    cv::Mat gain;                   ///< Kalman gain                                    K(k) := P(k|k-1)*H(k)t*inv(H(k)*P(k|k-1)*H(k)t+R(k))
    cv::Mat statePost;              ///< Corrected state                                x'(k|k) := x'(k|k-1) + K(k)*(z(k) - h(x'(k|k-1)))
    cv::Mat errorCovPost;           ///< A posteriori error estimate covariance matrix  P(k|k) := (I - K(k)*H(k))*P(k|k-1)

private:

    cv::Mat temp1;
    cv::Mat temp2;
    cv::Mat temp3;
    cv::Mat temp4;
    cv::Mat temp5;
};

#endif /* EXTENDEDKALMANFILTER_H */


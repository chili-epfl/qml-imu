/*
 * Copyright (C) 2014 EPFL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 */

/**
 * @file ExtendedKalmanFilter.cpp
 * @brief Standard extended Kalman filter
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#include"ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

ExtendedKalmanFilter::ExtendedKalmanFilter(int dynamParams, int measureParams, int type)
{
    init(dynamParams, measureParams, type);
}

void ExtendedKalmanFilter::init(int DP, int MP, int type)
{
    CV_Assert(DP > 0 && MP > 0);
    CV_Assert(type == CV_32F || type == CV_64F);

    statePre = cv::Mat::zeros(DP, 1, type);
    statePost = cv::Mat::zeros(DP, 1, type);
    transitionMatrix = cv::Mat::eye(DP, DP, type);

    processNoiseCov = cv::Mat::eye(DP, DP, type);
    observationMatrix = cv::Mat::zeros(MP, DP, type);
    observationNoiseCov = cv::Mat::eye(MP, MP, type);

    errorCovPre = cv::Mat::zeros(DP, DP, type);
    errorCovPost = cv::Mat::zeros(DP, DP, type);
    gain = cv::Mat::zeros(DP, MP, type);

    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);
}

cv::Mat const& ExtendedKalmanFilter::predict(cv::Mat const& process)
{
    //Update the state: x'(k|k-1) = f(x'(k-1|k-1), u(k-1))
    process.copyTo(statePre);

    //Update error covariance matrices: temp1 = F(k-1)*P(k-1|k-1)
    temp1 = transitionMatrix*errorCovPost;

    //P(k|k-1) = temp1*F(k-1)t + Q(k-1)
    cv::gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, cv::GEMM_2_T);

    //Handle the case when there will be measurement before the next predict
    //statePre.copyTo(statePost); //We will do this outside after corrections
    errorCovPre.copyTo(errorCovPost);

    return statePre;
}

cv::Mat const& ExtendedKalmanFilter::correct(cv::Mat const& observation, cv::Mat const& predictedObservation)
{
    //temp2 = H(k)*P(k|k-1)
    temp2 = observationMatrix*errorCovPre;

    //temp3 = temp2*H(k)t + R(k)
    cv::gemm(temp2, observationMatrix, 1, observationNoiseCov, 1, temp3, cv::GEMM_2_T);

    //temp4 = inv(temp3)*temp2 = K(k)t
    cv::solve(temp3, temp2, temp4, cv::DECOMP_SVD);

    //K(k)
    gain = temp4.t();

    //temp5 = z(k) - h(x'(k|k-1))
    temp5 = observation - predictedObservation;

    //x'(k|k) = x'(k|k-1) + K(k)*temp5
    statePost = statePre + gain*temp5;

    //P(k|k) = P(k|k-1) - K(k)*temp2
    errorCovPost = errorCovPre - gain*temp2;

    return statePost;
}


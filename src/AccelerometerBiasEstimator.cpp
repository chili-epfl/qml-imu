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
 * @file AccelerometerBiasEstimator.cpp
 * @brief Implementation of the QML wrapper for accelerometer bias estimation
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#include"AccelerometerBiasEstimator.h"

#include<type_traits>
#include<cfloat>
#include<cmath>

#include<QtMath>

const int AccelerometerBiasEstimator::CV_TYPE = std::is_same<qreal, double>::value ? CV_64F : CV_32F;

AccelerometerBiasEstimator::AccelerometerBiasEstimator(QQuickItem* parent) :
    QQuickItem(parent),
    accId(""),
    acc(nullptr),
    lastAccTimestamp(0),
    filter(3, 3, CV_TYPE),
    R_g_k_0(1e+3f),  //This depends on the coefficient below
    R_g_k_g(1e+6f)   //This depends on accelerometer sensor limits, typically 2g
{

    //Open first encountered and valid accelerometer
    for(auto const& type : QSensor::sensorTypes())
        for(auto const& id : QSensor::sensorsForType(type))
            if(type == "QAccelerometer")
                if(openAcc(id))
                    break;

    //Just do assumptions for initial values
    filter.statePre =   (cv::Mat_<qreal>(3,1) << 0.0f, 0.0f, 0.0f);
    filter.statePost =  (cv::Mat_<qreal>(3,1) << 0.0f, 0.0f, 0.0f);

    observation =           (cv::Mat_<qreal>(3,1) << 0.0f, 0.0f, 0.0f);
    predictedObservation =  (cv::Mat_<qreal>(3,1) << 0.0f, 0.0f, 0.0f);

    filter.transitionMatrix = (cv::Mat_<qreal>(3,3) <<
            1.0f,   0.0f,   0.0f,
            0.0f,   1.0f,   0.0f,
            0.0f,   0.0f,   1.0f);

    filter.observationMatrix = (cv::Mat_<qreal>(3,3) <<
            1.0f,   0.0f,   0.0f,
            0.0f,   1.0f,   0.0f,
            0.0f,   0.0f,   1.0f);

    filter.processNoiseCov = (cv::Mat_<qreal>(3,3) <<
            1.0f,   0.0f,   0.0f,
            0.0f,   1.0f,   0.0f,
            0.0f,   0.0f,   1.0f);

    filter.errorCovPre = (cv::Mat_<qreal>(3,3) <<
            1.0f,   0.0f,   0.0f,
            0.0f,   1.0f,   0.0f,
            0.0f,   0.0f,   1.0f);

    filter.errorCovPost = (cv::Mat_<qreal>(3,3) <<
            1.0f,   0.0f,   0.0f,
            0.0f,   1.0f,   0.0f,
            0.0f,   0.0f,   1.0f);
}

AccelerometerBiasEstimator::~AccelerometerBiasEstimator()
{
    delete acc;
}

bool AccelerometerBiasEstimator::openAcc(QByteArray const& id)
{
    QAccelerometer* newAcc = new QAccelerometer(this);
    newAcc->setIdentifier(id);
    bool success = false;

    //Sensor is fine
    if(newAcc->connectToBackend()){
        accId = QByteArray(id);
        emit accIdChanged();
        delete acc;
        acc = newAcc;
        connect(acc, &QAccelerometer::readingChanged, this, &AccelerometerBiasEstimator::accReadingChanged);
        acc->setDataRate(1000); //Probably will not go this high and will reach maximum
        if(acc->start()){
            qDebug() << "Opened accelerometer with identifier " << id;
            success = true;
        }
    }

    //Sensor could not be opened for some reason
    if(!success){
        qDebug() << "Error: Could not open accelerometer with identifier " << id;
        delete newAcc;
    }
    return success;
}

QString AccelerometerBiasEstimator::getAccId()
{
    return accId;
}

void AccelerometerBiasEstimator::setAccId(QString const& newId)
{
    if(newId == accId)
        return;

    for(auto const& id : QSensor::sensorsForType("QAccelerometer"))
        if(id == newId){
            openAcc(id);
            return;
        }

    qDebug() << "Error: Accelerometer with identifier " << newId << " not found.";
}

void AccelerometerBiasEstimator::accReadingChanged()
{
    quint64 timestamp = acc->reading()->timestamp();

    if(lastAccTimestamp > 0)
        if(((qreal)(timestamp - lastAccTimestamp))/1000000.0f > 0){
            a.setX(acc->reading()->x()); //Linear acceleration along x axis in m/s^2
            a.setY(acc->reading()->y()); //Linear acceleration along y axis in m/s^2
            a.setZ(acc->reading()->z()); //Linear acceleration along z axis in m/s^2

            filter.predict(filter.statePost);
            calculateObservation();
            filter.correct(observation, predictedObservation);
            bias.setX(filter.statePost.at<qreal>(0));
            bias.setY(filter.statePost.at<qreal>(1));
            bias.setZ(filter.statePost.at<qreal>(2));

            covTrace = filter.errorCovPost.at<qreal>(0,0) + filter.errorCovPost.at<qreal>(1,1) + filter.errorCovPost.at<qreal>(2,2);
            qDebug() << "tr(cov): " << covTrace << " bias: " << bias;

            emit biasChanged();
        }

    lastAccTimestamp = timestamp;
}

void AccelerometerBiasEstimator::calculateObservation()
{
    //cv::Mat data pointers
    qreal* statePrePtr = (qreal*)filter.statePre.ptr();
    qreal* observationPtr = (qreal*)observation.ptr();
    qreal* predictedObservationPtr = (qreal*)predictedObservation.ptr();

    qreal ax = a.x();
    qreal ay = a.y();
    qreal az = a.z();
    const qreal g = 9.81f;

    //We assume here that the device is lying completely flat so that only one axis feels gravity
    if(ax > std::abs(ay) && ax > std::abs(az))
        ax -= g;
    else if(-ax > std::abs(ay) && -ax > std::abs(az))
        ax += g;
    else if(ay > std::abs(ax) && ay > std::abs(az))
        ay -= g;
    else if(-ay > std::abs(ax) && -ay > std::abs(az))
        ay += g;
    else if(az > std::abs(ax) && az > std::abs(ay))
        az -= g;
    else
        az += g;

    observationPtr[0] = ax;
    observationPtr[1] = ay;
    observationPtr[2] = az;
    predictedObservationPtr[0] = statePrePtr[0];
    predictedObservationPtr[1] = statePrePtr[1];
    predictedObservationPtr[2] = statePrePtr[2];

    qreal R = R_g_k_0 + R_g_k_g*std::fabs(g - a.length());
    filter.observationNoiseCov.at<qreal>(0,0) = R;
    filter.observationNoiseCov.at<qreal>(1,1) = R;
    filter.observationNoiseCov.at<qreal>(2,2) = R;
}

QVector3D AccelerometerBiasEstimator::getBias()
{
    return bias;
}

qreal AccelerometerBiasEstimator::getCovTrace()
{
    return covTrace;
}

void AccelerometerBiasEstimator::changeParent(QQuickItem* parent)
{
    //FIXME: we probably need to disconnect the previous parent
    //TODO: probably a good idea to stop the camera (and restart it if we are auto-starting in this context)
}


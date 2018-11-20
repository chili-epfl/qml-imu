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
 * @file IMU.cpp
 * @brief Implementation of the QML wrapper for IMU processing
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#include"IMU.h"

#include<type_traits>
#include<cfloat>
#include<cmath>

#include<QtMath>

const int IMU::CV_TYPE = std::is_same<qreal, double>::value ? CV_64F : CV_32F;
const qreal IMU::EPSILON = std::is_same<qreal, double>::value ? DBL_EPSILON : FLT_EPSILON;

IMU::IMU(QQuickItem* parent) :
    QQuickItem(parent),
    gyroId(""),
    accId(""),
    magId(""),
    gyro(nullptr),
    acc(nullptr),
    mag(nullptr),
    lastGyroTimestamp(0),
    lastAccTimestamp(0),
    lastMagTimestamp(0),
    gyroSilentCycles(0),
    accSilentCycles(0),
    magSilentCycles(0),
    filter(7, 6, CV_TYPE),
    startupTime(1.0f),
    R_g_startup(1e-1f),
    R_y_startup(1e-3f),
    R_g_k_0(1.0f),  //This depends on the two coefficients below
    R_g_k_w(7.5f),  //This depends on gyroscope sensor limits, typically 250 deg/s = 7.6 rad/s
    R_g_k_g(10.0f), //This depends on accelerometer sensor limits, typically 2g
    R_y_k_0(10.0f),  //This depends on the four coefficients below
    R_y_k_w(7.5f),   //This depends on gyroscope sensor limits, typically 250 deg/s = 7.6 rad/s
    R_y_k_g(10.0f),  //This depends on the accelerometer sensor limits, typically 2g
    R_y_k_n(20.0f),  //This depends on magnetic vector magnitude in milliTeslas
    R_y_k_d(15.0f),  //This depents on magnetic vector dip against floor vector, in radians
    magDataReady(false),
    m_norm_mean(-1),
    m_dip_angle_mean(-1),
    m_mean_alpha(0.99f),
    a_bias(0, 0, 0),
    velocityWDecay(15.0f),
    velocityADecay(8.0f)
{

    //Open first encountered and valid gyroscope and accelerometer
    for(auto const& type : QSensor::sensorTypes())
        for(auto const& id : QSensor::sensorsForType(type))
            if(type == "QGyroscope"){
                if(openGyro(id))
                    break;
            }
            else if(type == "QAccelerometer"){
                if(openAcc(id))
                    break;
            }
            else if(type == "QMagnetometer"){
                if(openMag(id))
                    break;
            }

    //Just do assumptions for initial values
    process =           (cv::Mat_<qreal>(7,1) << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    filter.statePre =   (cv::Mat_<qreal>(7,1) << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    statePreHistory =   (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);
    filter.statePost =  (cv::Mat_<qreal>(7,1) << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f ,0.0f);
    statePostHistory =  (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);

    const qreal g = 9.81f;
    observation =           (cv::Mat_<qreal>(6,1) << 0.0f, 0.0f, g, 0.0f, 1.0f, 0.0f);
    predictedObservation =  (cv::Mat_<qreal>(6,1) << 0.0f, 0.0f, g, 0.0f, 1.0f, 0.0f);

    filter.transitionMatrix = (cv::Mat_<qreal>(7,7) <<
            1.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   1.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   1.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   1.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f);

    //Process noise covariance matrix is deltaT*Q at each step
    Q = (cv::Mat_<qreal>(7,7) <<
            1e-4f,  0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   1e-4f,  0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   1e-4f,  0.0f,   0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   1e-4f,  0.0f,   0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   0.0f,   1e-2f,  0.0f,   0.0f,
            0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   1e-2f,  0.0f,
            0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   1e-2f);
    Q.copyTo(filter.errorCovPre);
}

IMU::~IMU()
{
    delete gyro;
    delete acc;
    delete mag;
}

bool IMU::openGyro(QByteArray const& id)
{
    QGyroscope* newGyro = new QGyroscope(this);
    newGyro->setIdentifier(id);
    bool success = false;

    //Sensor is fine
    if(newGyro->connectToBackend()){
        gyroId = QString(id);
        emit gyroIdChanged();
        delete gyro;
        gyro = newGyro;
        connect(gyro, &QGyroscope::readingChanged, this, &IMU::gyroReadingChanged);
        gyro->setDataRate(1000); //Probably will not go this high and will reach maximum
        if(gyro->start()){
            qDebug() << "Opened gyroscope with identifier " << id;
            success = true;
        }
    }

    //Sensor could not be opened for some reason
    if(!success){
        qDebug() << "Error: Could not open gyroscope with identifier " << id;
        delete newGyro;
    }
    return success;
}

bool IMU::openAcc(QByteArray const& id)
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
        connect(acc, &QAccelerometer::readingChanged, this, &IMU::accReadingChanged);
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

bool IMU::openMag(QByteArray const& id)
{
    QMagnetometer* newMag = new QMagnetometer(this);
    newMag->setIdentifier(id);
    bool success = false;

    //Sensor is fine
    if(newMag->connectToBackend()){
        magId = QByteArray(id);
        emit magIdChanged();
        delete mag;
        mag = newMag;
        connect(mag, &QMagnetometer::readingChanged, this, &IMU::magReadingChanged);
        mag->setDataRate(1000); //Probably will not go this high and will reach maximum
        mag->setReturnGeoValues(true); //Try to cancel out magnetic interference
        if(mag->start()){
            qDebug() << "Opened magnetometer with identifier " << id;
            success = true;
        }
    }

    //Sensor could not be opened for some reason
    if(!success){
        qDebug() << "Error: Could not open magnetometer with identifier " << id;
        delete newMag;
    }
    return success;
}

void IMU::setGyroId(QString const& newId)
{
    if(newId == gyroId)
        return;

    for(auto const& id : QSensor::sensorsForType("QGyroscope"))
        if(id == newId){
            openGyro(id);
            return;
        }

    qDebug() << "Error: Gyroscope with identifier " << newId << " not found.";
}

void IMU::setAccId(QString const& newId)
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

void IMU::setMagId(QString const& newId)
{
    if(newId == magId)
        return;

    for(auto const& id : QSensor::sensorsForType("QMagnetometer"))
        if(id == newId){
            openMag(id);
            return;
        }

    qDebug() << "Error: Magnetometer with identifier " << newId << " not found.";
}

void IMU::gyroReadingChanged()
{
    quint64 timestamp = gyro->reading()->timestamp();

    if(lastGyroTimestamp > 0){
        wDeltaT = ((qreal)(timestamp - lastGyroTimestamp))/1000000.0f;
        if(wDeltaT > 0){
            gyroSilentCycles = 0;

            //Take care of startup time
            if(startupTime > 0){
                startupTime -= wDeltaT;
                if(startupTime < 0){
                    startupTime = 0;
                    resetDisplacement();
                    velocity = QVector3D(0,0,0);
                    qDebug() << "Startup is over";
                    emit startupCompleteChanged();
                }
            }

            w.setX(qDegreesToRadians(gyro->reading()->x())); //Angular velocity around x axis in rad/s
            w.setY(qDegreesToRadians(gyro->reading()->y())); //Angular velocity around y axis in rad/s
            w.setZ(qDegreesToRadians(gyro->reading()->z())); //Angular velocity around z axis in rad/s
            w_norm = w.length();

            //Calculate process value, transition matrix and process noise covariance matrix
            calculateProcess();

            //Do prediction step
            filter.predict(process);

            //Ensure output quaternion is unit norm
            normalizeQuat(filter.statePre);

            //Ensure output quaternion doesn't unwind
            shortestPathQuat(statePreHistory, filter.statePre);

            //Ensure a posteriori state reflects prediction in case measurement doesn't occur
            filter.statePre.copyTo(filter.statePost);

            //Export rotation and linear acceleration
            calculateOutput();
        }
    }
    lastGyroTimestamp = timestamp;
}

void IMU::accReadingChanged()
{
    quint64 timestamp = acc->reading()->timestamp();

    if(lastAccTimestamp > 0){
        aDeltaT = ((qreal)(timestamp - lastAccTimestamp))/1000000.0f;
        if(aDeltaT > 0){
            accSilentCycles = 0;
            a.setX(acc->reading()->x() - a_bias.x()); //Linear acceleration along x axis in m/s^2
            a.setY(acc->reading()->y() - a_bias.y()); //Linear acceleration along y axis in m/s^2
            a.setZ(acc->reading()->z() - a_bias.z()); //Linear acceleration along z axis in m/s^2
            a_norm = a.length();

            //Calculate observation value, predicted observation value and observation matrix
            //We assume here that the magnetometer reading is less frequent compared to accelerometer
            calculateObservation();

            //Do correction step
            filter.correct(observation, predictedObservation);

            //Ensure ouput quaternion is unit norm
            normalizeQuat(filter.statePost);

            //Ensure output quaternion doesn't unwind
            shortestPathQuat(statePostHistory, filter.statePost);

            //Export rotation
            calculateOutput();

            //Update displacement values
            updateDisplacement();
        }
    }
    lastAccTimestamp = timestamp;
}

void IMU::magReadingChanged()
{
    quint64 timestamp = mag->reading()->timestamp();

    if(lastMagTimestamp > 0)
        if(((qreal)(timestamp - lastMagTimestamp))/1000000.0f > 0){
            magSilentCycles = 0;
            m.setX(mag->reading()->x()*1000000.0f); //Magnetic flux along x axis in milliTeslas
            m.setY(mag->reading()->y()*1000000.0f); //Magnetic flux along y axis in milliTeslas
            m.setZ(mag->reading()->z()*1000000.0f); //Magnetic flux along z axis in milliTeslas
            m_norm = m.length();
            magDataReady = true;
        }
    lastMagTimestamp = timestamp;
}

inline void IMU::normalizeQuat(cv::Mat& quat)
{
    qreal* q = (qreal*)quat.ptr();
    qreal norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm > EPSILON){
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
    else{
        q[0] = 1.0f;
        q[1] = 1.0f;
        q[2] = 1.0f;
        q[3] = 1.0f;
    }
}

inline void IMU::shortestPathQuat(cv::Mat& prevQuat, cv::Mat& quat)
{
    qreal* p = (qreal*)prevQuat.ptr();
    qreal* q = (qreal*)quat.ptr();

    //If -q would be closer to q_prev than +q, replace new q with -q
    //The following comes from the derivation of |q - q_prev|^2 - |-q - q_prev|^2
    if(q[0]*p[0] + q[1]*p[1] + q[2]*p[2] + q[3]*p[3] < 0){
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
    p[0] = q[0];
    p[1] = q[1];
    p[2] = q[2];
    p[3] = q[3];
}

void IMU::calculateProcess()
{
    //cv::Mat data pointers
    qreal* processPtr = (qreal*)process.ptr();
    qreal* F0 = (qreal*)filter.transitionMatrix.ptr(0);
    qreal* F1 = (qreal*)filter.transitionMatrix.ptr(1);
    qreal* F2 = (qreal*)filter.transitionMatrix.ptr(2);
    qreal* F3 = (qreal*)filter.transitionMatrix.ptr(3);
    qreal* F4 = (qreal*)filter.transitionMatrix.ptr(4);
    qreal* F5 = (qreal*)filter.transitionMatrix.ptr(5);
    qreal* F6 = (qreal*)filter.transitionMatrix.ptr(6);

    //Calculate process value
    const qreal q0 = filter.statePost.at<qreal>(0);
    const qreal q1 = filter.statePost.at<qreal>(1);
    const qreal q2 = filter.statePost.at<qreal>(2);
    const qreal q3 = filter.statePost.at<qreal>(3);
    const qreal wx = w.x();
    const qreal wy = w.y();
    const qreal wz = w.z();
    const qreal ax = a.x();
    const qreal ay = a.y();
    const qreal az = a.z();
    const qreal g = 9.81f;

    //Absolute rotation
    processPtr[0] = q0 + 0.5f*wDeltaT*(-q1*wx - q2*wy - q3*wz);
    processPtr[1] = q1 + 0.5f*wDeltaT*(+q0*wx - q3*wy + q2*wz);
    processPtr[2] = q2 + 0.5f*wDeltaT*(+q3*wx + q0*wy - q1*wz);
    processPtr[3] = q3 + 0.5f*wDeltaT*(-q2*wx + q1*wy + q0*wz);

    //Absolute linear acceleration
    processPtr[4] = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*ax + 2*(q1*q2 - q0*q3)*ay + 2*(q1*q3 + q0*q2)*az;
    processPtr[5] = 2*(q1*q2 + q0*q3)*ax + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*ay + 2*(q2*q3 - q0*q1)*az;
    processPtr[6] = 2*(q1*q3 - q0*q2)*ax + 2*(q2*q3 + q0*q1)*ay + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*az - g;

    normalizeQuat(process);

    //Calculate transition matrix
    /* 1.0f */                  F0[1] = -0.5f*wDeltaT*wx;   F0[2] = -0.5f*wDeltaT*wy;   F0[3] = -0.5f*wDeltaT*wz;
    F1[0] = +0.5f*wDeltaT*wx;   /* 1.0f */                  F1[2] = +0.5f*wDeltaT*wz;   F1[3] = -0.5f*wDeltaT*wy;
    F2[0] = +0.5f*wDeltaT*wy;   F2[1] = -0.5f*wDeltaT*wz;   /* 1.0f */                  F2[3] = +0.5f*wDeltaT*wx;
    F3[0] = +0.5f*wDeltaT*wz;   F3[1] = +0.5f*wDeltaT*wy;   F3[2] = -0.5f*wDeltaT*wx;  /* 1.0f */

    F4[0] = 2*(+q0*ax - q3*ay + q2*az); F4[1] = 2*(+q1*ax + q2*ay + q3*az); F4[2] = 2*(-q2*ax + q1*ay + q0*az); F4[3] = 2*(-q3*ax - q0*ay + q1*az);
    F5[0] = 2*(+q3*ax + q0*ay - q1*az); F5[1] = 2*(+q2*ax - q1*ay - q0*az); F5[2] = 2*(+q1*ax + q2*ay + q3*az); F5[3] = 2*(+q0*ax - q3*ay + q2*az);
    F6[0] = 2*(-q2*ax + q1*ay + q0*az); F6[1] = 2*(+q3*ax + q0*ay - q1*az); F6[2] = 2*(-q0*ax + q3*ay - q2*az); F6[3] = 2*(+q1*ax + q2*ay + q3*az);

    //Calculate process covariance matrix
    filter.processNoiseCov = Q*wDeltaT; //TODO: We should not multiply the acceleration part with deltaT
}

void IMU::calculateObservation()
{
    //cv::Mat data pointers
    qreal* statePrePtr = (qreal*)filter.statePre.ptr();
    qreal* observationPtr = (qreal*)observation.ptr();
    qreal* predictedObservationPtr = (qreal*)predictedObservation.ptr();
    qreal* H0 = (qreal*)filter.observationMatrix.ptr(0);
    qreal* H1 = (qreal*)filter.observationMatrix.ptr(1);
    qreal* H2 = (qreal*)filter.observationMatrix.ptr(2);
    qreal* H3 = (qreal*)filter.observationMatrix.ptr(3);
    qreal* H4 = (qreal*)filter.observationMatrix.ptr(4);
    qreal* H5 = (qreal*)filter.observationMatrix.ptr(5);

    //Variables dependent on current state
    const qreal q0 = statePrePtr[0];
    const qreal q1 = statePrePtr[1];
    const qreal q2 = statePrePtr[2];
    const qreal q3 = statePrePtr[3];
    const qreal g = 9.81f;
    const qreal R_DCM_z0 = 2*(q1*q3 - q0*q2);
    const qreal R_DCM_z1 = 2*(q2*q3 + q0*q1);
    const qreal R_DCM_z2 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //Accelerometer observation and noise
    observationPtr[0] = a.x();
    observationPtr[1] = a.y();
    observationPtr[2] = a.z();
    predictedObservationPtr[0] = R_DCM_z0*g;
    predictedObservationPtr[1] = R_DCM_z1*g;
    predictedObservationPtr[2] = R_DCM_z2*g;
    qreal R_g = R_g_k_0 + R_g_k_w*w_norm + R_g_k_g*std::fabs(g - a_norm);

    //Magnetometer reading variables and observation
    qreal R_y;
    if(magDataReady){
        qreal mx = m.x();
        qreal my = m.y();
        qreal mz = m.z();

        const qreal dot_m_z = mx*R_DCM_z0 + my*R_DCM_z1 + mz*R_DCM_z2;

        qreal m_dip_angle = acos(dot_m_z/m_norm);
        if(std::isnan(m_dip_angle))
            m_dip_angle = 0.0f;

        if(m_norm_mean < 0) //For fast startup
            m_norm_mean = m_norm;
        else
            m_norm_mean = m_mean_alpha*m_norm_mean + (1.0f - m_mean_alpha)*m_norm;

        if(m_dip_angle_mean < 0) //For fast startup
            m_dip_angle_mean = m_dip_angle;
        else
            m_dip_angle_mean = m_mean_alpha*m_dip_angle_mean + (1.0f - m_mean_alpha)*m_dip_angle;

        mx = mx - dot_m_z*R_DCM_z0; //Reject magnetic component on Z axis
        my = my - dot_m_z*R_DCM_z1; //Reject magnetic component on Z axis
        mz = mz - dot_m_z*R_DCM_z2; //Reject magnetic component on Z axis
        qreal uy_norm = sqrt(mx*mx + my*my + mz*mz);
        if(uy_norm > EPSILON){
            mx /= uy_norm;
            my /= uy_norm;
            mz /= uy_norm;
        }

        observationPtr[3] = mx;
        observationPtr[4] = my;
        observationPtr[5] = mz;
        predictedObservationPtr[3] = 2*(q1*q2 + q0*q3);
        predictedObservationPtr[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        predictedObservationPtr[5] = 2*(q2*q3 - q0*q1);

        R_y = R_y_k_0 + R_y_k_w*w_norm + R_y_k_g*std::fabs(g - a_norm) +
            R_y_k_n*std::fabs(m_norm - m_norm_mean) + R_y_k_d*std::fabs(m_dip_angle - m_dip_angle_mean);
    }
    else{
        observationPtr[3] = 0.0f;
        observationPtr[4] = 0.0f;
        observationPtr[5] = 0.0f;
        predictedObservationPtr[3] = 0.0f;
        predictedObservationPtr[4] = 0.0f;
        predictedObservationPtr[5] = 0.0f;

        R_y = 1.0f; //This doesn't matter, as long as it doesn't cause nans or infs in S^-1
    }

    //Calculate observation matrix
    H0[0] = -2*g*q2;  H0[1] = +2*g*q3;  H0[2] = -2*g*q0;  H0[3] = +2*g*q1;
    H1[0] = +2*g*q1;  H1[1] = +2*g*q0;  H1[2] = +2*g*q3;  H1[3] = +2*g*q2;
    H2[0] = +2*g*q0;  H2[1] = -2*g*q1;  H2[2] = -2*g*q2;  H2[3] = +2*g*q3;
    if(magDataReady){
        H3[0] = +2*q3;    H3[1] = +2*q2;    H3[2] = +2*q1;    H3[3] = +2*q0;
        H4[0] = +2*q0;    H4[1] = -2*q1;    H4[2] = +2*q2;    H4[3] = -2*q3;
        H5[0] = -2*q1;    H5[1] = -2*q0;    H5[2] = +2*q3;    H5[3] = +2*q2;
    }
    else{
        H3[0] = 0.0f; H3[1] = 0.0f; H3[2] = 0.0f; H3[3] = 0.0f;
        H4[0] = 0.0f; H4[1] = 0.0f; H4[2] = 0.0f; H4[3] = 0.0f;
        H5[0] = 0.0f; H5[1] = 0.0f; H5[2] = 0.0f; H5[3] = 0.0f;
    }

    //Calculate observation noise
    if(startupTime > 0){
        filter.observationNoiseCov.at<qreal>(0,0) = R_g_startup;
        filter.observationNoiseCov.at<qreal>(1,1) = R_g_startup;
        filter.observationNoiseCov.at<qreal>(2,2) = R_g_startup;
        filter.observationNoiseCov.at<qreal>(3,3) = R_y_startup;
        filter.observationNoiseCov.at<qreal>(4,4) = R_y_startup;
        filter.observationNoiseCov.at<qreal>(5,5) = R_y_startup;
    }
    else{
        filter.observationNoiseCov.at<qreal>(0,0) = R_g;
        filter.observationNoiseCov.at<qreal>(1,1) = R_g;
        filter.observationNoiseCov.at<qreal>(2,2) = R_g;
        filter.observationNoiseCov.at<qreal>(3,3) = R_y;
        filter.observationNoiseCov.at<qreal>(4,4) = R_y;
        filter.observationNoiseCov.at<qreal>(5,5) = R_y;
    }

    //Consumed latest magnetometer data
    magDataReady = false;
}

void IMU::calculateOutput()
{
    //Check existence and health of sensors
    if(gyroId == ""){
        qDebug() << "Error: Cannot operate without a gyroscope!";
        return;
    }
    else{
        gyroSilentCycles++;
        if(gyroSilentCycles > 1000)
            qDebug() << "Warning: Gyroscope is open but didn't receive data for " << gyroSilentCycles << " cycles!";
    }
    if(accId == "")
        qDebug() << "Warning: Operating without an accelerometer, results will drift!";
    else{
        accSilentCycles++;
        if(accSilentCycles > 1000)
            qDebug() << "Warning: Accelerometer is open but didn't receive data for " << accSilentCycles << " cycles!";
    }
    if(magId == "")
        qDebug() << "Warning: Operating without a magnetometer, results will drift!";
    else{
        magSilentCycles++;
        if(magSilentCycles > 1000)
            qDebug() << "Warning: Magnetometer is open but didn't receive data for " << magSilentCycles << " cycles!";
    }

    //Do not give output in the startup phase
    if(!isStartupComplete())
        return;

    //Calculate output rotation
    qreal* s = (qreal*)filter.statePost.ptr();

    rotQuat.setScalar(s[0]);
    rotQuat.setX(s[1]);
    rotQuat.setY(s[2]);
    rotQuat.setZ(s[3]);

    rotAngle = sqrt(s[1]*s[1] + s[2]*s[2] + s[3]*s[3]);
    rotAngle = 2*atan2(rotAngle, s[0]);
    if(rotAngle < EPSILON){
        rotAxis.setX(0.0f);
        rotAxis.setY(0.0f);
        rotAxis.setZ(0.0f);
        rotAngle = 0.0f;
    }
    else{
        qreal sTheta2 = sin(rotAngle/2);
        rotAxis.setX(s[1]*sTheta2);
        rotAxis.setY(s[2]*sTheta2);
        rotAxis.setZ(s[3]*sTheta2);
        rotAxis.normalize();
        rotAngle = qRadiansToDegrees(rotAngle);
    }

    //Calculate output linear acceleration
    linearAcceleration.setX(s[4]);
    linearAcceleration.setY(s[5]);
    linearAcceleration.setZ(s[6]);

    //Calculate floor vector in target frame
    targetFloorVector = rotQuat.conjugate().rotatedVector(QVector3D(0,0,1));
    targetFloorVector = targetRotation.conjugate().rotatedVector(targetFloorVector);

    emit stateChanged();
}

void IMU::updateDisplacement()
{
    if(!isStartupComplete())
        return;

    qreal* s = (qreal*)filter.statePost.ptr();

    QVector3D linearAcceleration(s[4], s[5], s[6]);
    dispTranslation += aDeltaT*velocity + 0.5f*aDeltaT*aDeltaT*linearAcceleration;
    velocity += aDeltaT*linearAcceleration;

    //Since velocity estimate random walks and is unbounded, we decay it when we assume the device is stationary
    qreal la_norm = linearAcceleration.length();
    qreal e_minus_w_norm = std::exp(-velocityWDecay*w_norm);
    qreal e_minus_la_norm = std::exp(-velocityADecay*la_norm);
    velocity = (1.0f - e_minus_w_norm)/(1.0f + e_minus_w_norm)*(1.0f - e_minus_la_norm)/(1.0f + e_minus_la_norm)*velocity;
}

void IMU::setStartupTime(qreal startupTime)
{
    if(this->startupTime <= 0 && startupTime > 0){
        this->startupTime = startupTime;
        emit startupCompleteChanged();
    }
}

qreal IMU::getStartupTime()
{
    return startupTime;
}

bool IMU::isStartupComplete()
{
    return startupTime <= 0;
}

void IMU::resetDisplacement()
{
    qreal* s = (qreal*)filter.statePost.ptr();
    prevRotation.setScalar(s[0]);
    prevRotation.setVector(s[1], s[2], s[3]);
    dispTranslation.setX(0.0f);
    dispTranslation.setY(0.0f);
    dispTranslation.setZ(0.0f);
}

QVector3D IMU::getLinearDisplacement()
{
    qreal* s = (qreal*)filter.statePost.ptr();
    QQuaternion currentRotation(s[0], s[1], s[2], s[3]);
    QVector3D outT =
        prevRotation.conjugate().rotatedVector(dispTranslation + currentRotation.rotatedVector(targetTranslation))
        - targetTranslation;
    return targetRotation.conjugate().rotatedVector(outT);
}

QQuaternion IMU::getAngularDisplacement()
{
    qreal* s = (qreal*)filter.statePost.ptr();
    QQuaternion currentRotation(s[0], s[1], s[2], s[3]);
    QQuaternion outR = targetRotation.conjugate()*prevRotation.conjugate()*currentRotation*targetRotation;
    outR.normalize();
    return outR;
}

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
    filter(4, 6, CV_TYPE),
    startupTime(1.0f),
    R_g_startup(1e-1f),
    R_y_startup(1e-2f),
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
    m_mean_alpha(0.99f)
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
    process =           (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);
    filter.statePre =   (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);
    statePreHistory =   (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);
    filter.statePost =  (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);
    statePostHistory =  (cv::Mat_<qreal>(4,1) << 1.0f, 0.0f, 0.0f, 0.0f);

    observation =           (cv::Mat_<qreal>(6,1) << 0.0f, 0.0f, 9.81f, 0.0f, 1.0f, 0.0f);
    predictedObservation =  (cv::Mat_<qreal>(6,1) << 0.0f, 0.0f, 9.81f, 0.0f, 1.0f, 0.0f);

    //Process noise covariance matrix is deltaT*Q at each step
    Q = (cv::Mat_<qreal>(4,4) <<
            1e-4f,  0.0f,   0.0f,   0.0f,
            0.0f,   1e-4f,  0.0f,   0.0f,
            0.0f,   0.0f,   1e-4f,  0.0f,
            0.0f,   0.0f,   0.0f,   1e-4f);
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

QString IMU::getGyroId()
{
    return gyroId;
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

QString IMU::getAccId()
{
    return accId;
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

QString IMU::getMagId()
{
    return magId;
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
                if(startupTime < 0)
                    qDebug() << "Startup is over";
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

            //Export rotation
            calculateOutputRotation();
        }
    }
    lastGyroTimestamp = timestamp;
}

void IMU::accReadingChanged()
{
    quint64 timestamp = acc->reading()->timestamp();

    if(lastAccTimestamp > 0)
        if(((qreal)(timestamp - lastAccTimestamp))/1000000.0f > 0){
            accSilentCycles = 0;
            a.setX(acc->reading()->x()); //Linear acceleration along x axis in m/s^2
            a.setY(acc->reading()->y()); //Linear acceleration along y axis in m/s^2
            a.setZ(acc->reading()->z()); //Linear acceleration along z axis in m/s^2
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
            calculateOutputRotation();
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
    qreal norm = sqrt(
            quat.at<qreal>(0)*quat.at<qreal>(0) +
            quat.at<qreal>(1)*quat.at<qreal>(1) +
            quat.at<qreal>(2)*quat.at<qreal>(2) +
            quat.at<qreal>(3)*quat.at<qreal>(3));

    if(norm > EPSILON){
        quat.at<qreal>(0) /= norm;
        quat.at<qreal>(1) /= norm;
        quat.at<qreal>(2) /= norm;
        quat.at<qreal>(3) /= norm;
    }
    else{
        quat.at<qreal>(0) = 1.0f;
        quat.at<qreal>(1) = 0.0f;
        quat.at<qreal>(2) = 0.0f;
        quat.at<qreal>(3) = 0.0f;
    }
}

inline void IMU::shortestPathQuat(cv::Mat& prevQuat, cv::Mat& quat)
{
    //If -q would be closer to q_prev than +q, replace new q with -q
    //The following comes from the derivation of |q - q_prev|^2 - |-q - q_prev|^2
    if(
            quat.at<qreal>(0)*prevQuat.at<qreal>(0) +
            quat.at<qreal>(1)*prevQuat.at<qreal>(1) +
            quat.at<qreal>(2)*prevQuat.at<qreal>(2) +
            quat.at<qreal>(3)*prevQuat.at<qreal>(3) < 0){
        quat.at<qreal>(0) = -quat.at<qreal>(0);
        quat.at<qreal>(1) = -quat.at<qreal>(1);
        quat.at<qreal>(2) = -quat.at<qreal>(2);
        quat.at<qreal>(3) = -quat.at<qreal>(3);
    }
    prevQuat.at<qreal>(0) = quat.at<qreal>(0);
    prevQuat.at<qreal>(1) = quat.at<qreal>(1);
    prevQuat.at<qreal>(2) = quat.at<qreal>(2);
    prevQuat.at<qreal>(3) = quat.at<qreal>(3);
}

void IMU::calculateProcess()
{
    //Calculate process value
    qreal q0 = filter.statePost.at<qreal>(0);
    qreal q1 = filter.statePost.at<qreal>(1);
    qreal q2 = filter.statePost.at<qreal>(2);
    qreal q3 = filter.statePost.at<qreal>(3);
    const qreal wx = w.x();
    const qreal wy = w.y();
    const qreal wz = w.z();

    process.at<qreal>(0) = q0 + 0.5f*wDeltaT*(-q1*wx - q2*wy - q3*wz);
    process.at<qreal>(1) = q1 + 0.5f*wDeltaT*(+q0*wx - q3*wy + q2*wz);
    process.at<qreal>(2) = q2 + 0.5f*wDeltaT*(+q3*wx + q0*wy - q1*wz);
    process.at<qreal>(3) = q3 + 0.5f*wDeltaT*(-q2*wx + q1*wy + q0*wz);

    normalizeQuat(process);

    //Calculate transition matrix
    filter.transitionMatrix = (cv::Mat_<qreal>(4,4) <<
            +1.0f,              -0.5f*wDeltaT*wx,   -0.5f*wDeltaT*wy,   -0.5f*wDeltaT*wz,
            +0.5f*wDeltaT*wx,   +1.0f,              +0.5f*wDeltaT*wz,   -0.5f*wDeltaT*wy,
            +0.5f*wDeltaT*wy,   -0.5f*wDeltaT*wz,   +1.0f,              +0.5f*wDeltaT*wx,
            +0.5f*wDeltaT*wz,   +0.5f*wDeltaT*wy,   -0.5f*wDeltaT*wx,   +1.0f);

    //Calculate process covariance matrix
    filter.processNoiseCov = Q*wDeltaT;
}

void IMU::calculateObservation()
{
    //Variables dependent on current state
    qreal q0 = filter.statePre.at<qreal>(0);
    qreal q1 = filter.statePre.at<qreal>(1);
    qreal q2 = filter.statePre.at<qreal>(2);
    qreal q3 = filter.statePre.at<qreal>(3);
    const qreal g = 9.81f;
    qreal R_DCM_z0 = 2*(q1*q3 - q0*q2);
    qreal R_DCM_z1 = 2*(q2*q3 + q0*q1);
    qreal R_DCM_z2 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //Accelerometer observation and noise
    observation.at<qreal>(0) = a.x();
    observation.at<qreal>(1) = a.y();
    observation.at<qreal>(2) = a.z();
    predictedObservation.at<qreal>(0) = R_DCM_z0*g;
    predictedObservation.at<qreal>(1) = R_DCM_z1*g;
    predictedObservation.at<qreal>(2) = R_DCM_z2*g;
    qreal R_g = R_g_k_0 + R_g_k_w*w_norm + R_g_k_g*std::fabs(g - a_norm);

    //Magnetometer reading variables and observation
    qreal R_y;
    if(magDataReady){
        qreal mx = m.x();
        qreal my = m.y();
        qreal mz = m.z();

        qreal dot_m_z = mx*R_DCM_z0 + my*R_DCM_z1 + mz*R_DCM_z2;

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

        observation.at<qreal>(3) = mx - dot_m_z*R_DCM_z0; //Reject magnetic component on Z axis
        observation.at<qreal>(4) = my - dot_m_z*R_DCM_z1; //Reject magnetic component on Z axis
        observation.at<qreal>(5) = mz - dot_m_z*R_DCM_z2; //Reject magnetic component on Z axis
        qreal uy_norm = sqrt(
                observation.at<qreal>(3)*observation.at<qreal>(3) +
                observation.at<qreal>(4)*observation.at<qreal>(4) +
                observation.at<qreal>(5)*observation.at<qreal>(5));
        if(uy_norm > EPSILON){
            observation.at<qreal>(3) /= uy_norm;
            observation.at<qreal>(4) /= uy_norm;
            observation.at<qreal>(5) /= uy_norm;
        }

        predictedObservation.at<qreal>(3) = 2*(q1*q2 + q0*q3);
        predictedObservation.at<qreal>(4) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        predictedObservation.at<qreal>(5) = 2*(q2*q3 - q0*q1);

        R_y = R_y_k_0 + R_y_k_w*w_norm + R_y_k_g*std::fabs(g - a_norm) +
            R_y_k_n*std::fabs(m_norm - m_norm_mean) + R_y_k_d*std::fabs(m_dip_angle - m_dip_angle_mean);
    }
    else{
        observation.at<qreal>(3) = 0.0f;
        observation.at<qreal>(4) = 0.0f;
        observation.at<qreal>(5) = 0.0f;
        predictedObservation.at<qreal>(3) = 0.0f;
        predictedObservation.at<qreal>(4) = 0.0f;
        predictedObservation.at<qreal>(5) = 0.0f;

        R_y = 1.0f;
    }

    //Calculate observation matrix
    if(magDataReady)
        filter.observationMatrix = (cv::Mat_<qreal>(6,4) <<
                -2*g*q2,    +2*g*q3,    -2*g*q0,    +2*g*q1,
                +2*g*q1,    +2*g*q0,    +2*g*q3,    +2*g*q2,
                +2*g*q0,    -2*g*q1,    -2*g*q2,    +2*g*q3,
                +2*q3,      +2*q2,      +2*q1,      +2*q0,
                +2*q0,      -2*q1,      +2*q2,      -2*q3,
                -2*q1,      -2*q0,      +2*q3,      +2*q2);
    else
        filter.observationMatrix = (cv::Mat_<qreal>(6,4) <<
                -2*g*q2,    +2*g*q3,    -2*g*q0,    +2*g*q1,
                +2*g*q1,    +2*g*q0,    +2*g*q3,    +2*g*q2,
                +2*g*q0,    -2*g*q1,    -2*g*q2,    +2*g*q3,
                0.0f,       0.0f,       0.0f,       0.0f,
                0.0f,       0.0f,       0.0f,       0.0f,
                0.0f,       0.0f,       0.0f,       0.0f);

    //Calculate observation noise
    if(startupTime > 0)
        filter.observationNoiseCov = (cv::Mat_<qreal>(6,6) <<
                R_g_startup,    0.0f,           0.0f,           0.0f,           0.0f,           0.0f,
                0.0f,           R_g_startup,    0.0f,           0.0f,           0.0f,           0.0f,
                0.0f,           0.0f,           R_g_startup,    0.0f,           0.0f,           0.0f,
                0.0f,           0.0f,           0.0f,           R_y_startup,    0.0f,           0.0f,
                0.0f,           0.0f,           0.0f,           0.0f,           R_y_startup,    0.0f,
                0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           R_y_startup);
    else
        filter.observationNoiseCov = (cv::Mat_<qreal>(6,6) <<
                R_g,    0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
                0.0f,   R_g,    0.0f,   0.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   R_g,    0.0f,   0.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   R_y,    0.0f,   0.0f,
                0.0f,   0.0f,   0.0f,   0.0f,   R_y,    0.0f,
                0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   R_y);

    //Consumed latest magnetometer data
    magDataReady = false;
}

void IMU::calculateOutputRotation()
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
    if(startupTime > 0)
        return;

    //Calculate output
    rotAngle = sqrt(
            filter.statePost.at<qreal>(1)*filter.statePost.at<qreal>(1) +
            filter.statePost.at<qreal>(2)*filter.statePost.at<qreal>(2) +
            filter.statePost.at<qreal>(3)*filter.statePost.at<qreal>(3));
    rotAngle = 2*atan2(rotAngle, filter.statePost.at<qreal>(0));
    if(rotAngle < EPSILON){
        rotAxis.setX(0.0f);
        rotAxis.setY(0.0f);
        rotAxis.setZ(0.0f);
        rotAngle = 0.0f;
    }
    else{
        qreal sTheta2 = sin(rotAngle/2);
        rotAxis.setX(filter.statePost.at<qreal>(1)*sTheta2);
        rotAxis.setY(filter.statePost.at<qreal>(2)*sTheta2);
        rotAxis.setZ(filter.statePost.at<qreal>(3)*sTheta2);
        rotAngle = qRadiansToDegrees(rotAngle);
    }

    emit rotationChanged();
}

QVector3D IMU::getRotAxis()
{
    return rotAxis;
}

qreal IMU::getRotAngle()
{
    return rotAngle;
}

void IMU::changeParent(QQuickItem* parent)
{
    //FIXME: we probably need to disconnect the previous parent
    //TODO: probably a good idea to stop the camera (and restart it if we are auto-starting in this context)
}


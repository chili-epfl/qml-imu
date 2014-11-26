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

#include<QtMath>

IMU::IMU(QQuickItem* parent) :
    QQuickItem(parent),
    gyroId(""),
    accId(""),
    gyro(nullptr),
    acc(nullptr),
    lastGyroTimestamp(0),
    lastAccTimestamp(0)
{
    foreach(const QByteArray &type, QSensor::sensorTypes()) {
        qDebug() << "Found type" << type;
        foreach (const QByteArray &identifier, QSensor::sensorsForType(type)) {
            qDebug() << "Found identifier" << identifier;
            // Don't put in sensors we can't connect to
            QSensor* sensor = new QSensor(type, this);
            sensor->setIdentifier(identifier);
            if(!sensor->connectToBackend()){
                qDebug() << "Couldn't connect to" << identifier;
                continue;
            }

            qDebug() << "Numdatarates: " << sensor->availableDataRates().size();
            foreach(qrange const& range, sensor->availableDataRates()){
                qDebug() << "Datarate: " << range.first << " " << range.second;
            }
            qDebug() << "Adding identifier" << identifier;
        }
    }
}

IMU::~IMU()
{
    delete gyro;
    delete acc;
}

QString IMU::getGyroId()
{
    return gyroId;
}

void IMU::setGyroId(QString const& newId)
{
    if(newId == gyroId)
        return;

    for(auto const& id : QSensor::sensorsForType("QGyroscope")){
        if(id == newId){
            QGyroscope* newGyro = new QGyroscope(this);
            newGyro->setIdentifier(id);

            //Sensor is fine
            if(newGyro->connectToBackend()){
                gyroId = newId;
                emit gyroIdChanged();
                delete gyro;
                gyro = newGyro;
                connect(gyro,&QGyroscope::readingChanged,this,&IMU::gyroReadingChanged);
                gyro->setDataRate(1000); //Probably will not go this high and will reach maximum
                gyro->start();
            }

            //Sensor could not be opened for some reason
            else{
                qDebug() << "Error: Could not open gyroscope with identifier " << id;
                delete newGyro;
            }

            return;
        }
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

    for(auto const& id : QSensor::sensorsForType("QAccelerometer")){
        if(id == newId){
            QAccelerometer* newAcc = new QAccelerometer(this);
            newAcc->setIdentifier(id);

            //Sensor is fine
            if(newAcc->connectToBackend()){
                accId = newId;
                emit accIdChanged();
                delete acc;
                acc = newAcc;
                connect(acc, &QAccelerometer::readingChanged, this, &IMU::accReadingChanged);
                acc->setDataRate(1000); //Probably will not go this high and will reach maximum
                acc->start();
            }

            //Sensor could not be opened for some reason
            else{
                qDebug() << "Error: Could not open accelerometer with identifier " << id;
                delete newAcc;
            }

            return;
        }
    }

    qDebug() << "Error: Accelerometer with identifier " << newId << " not found.";
}

void IMU::gyroReadingChanged()
{
    quint64 timestamp = gyro->reading()->timestamp();
    qreal wx = qDegreesToRadians(gyro->reading()->x()); //Angular velocity around x axis in rad/s
    qreal wy = qDegreesToRadians(gyro->reading()->y()); //Angular velocity around y axis in rad/s
    qreal wz = qDegreesToRadians(gyro->reading()->z()); //Angular velocity around z axis in rad/s
    if(lastGyroTimestamp > 0){
        qreal deltaT = ((qreal)(timestamp - lastGyroTimestamp))/1000000.0f;
        qDebug() << "Gyro delta t: " << deltaT; //Get deltaT in seconds
    }
    lastGyroTimestamp = timestamp;
    qDebug() << "Gyro: " << wx << " " << wy << " " << wz;
}

void IMU::accReadingChanged()
{
    quint64 timestamp = acc->reading()->timestamp();
    qreal ax = acc->reading()->x(); //Linear acceleration along x axis in m/s^2
    qreal ay = acc->reading()->y(); //Linear acceleration along y axis in m/s^2
    qreal az = acc->reading()->z(); //Linear acceleration along z axis in m/s^2
    if(lastAccTimestamp > 0){
        qreal deltaT = ((qreal)(timestamp - lastAccTimestamp))/1000000.0f;
        qDebug() << "Acc delta t: " << deltaT; //Get deltaT in seconds
    }
    lastAccTimestamp = timestamp;
    qDebug() << "Acc: " << ax << " " << ay << " " << az;
}

QVector3D IMU::getRotation()
{

}

QQuaternion IMU::getRotationQuat()
{

}

void IMU::changeParent(QQuickItem* parent)
{
    //FIXME: we probably need to disconnect the previous parent
    //TODO: probably a good idea to stop the camera (and restart it if we are auto-starting in this context)
}


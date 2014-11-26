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
    QGyroscopeReading* reading = gyro->reading();
    if(lastGyroTimestamp > 0)
        qDebug() << "Gyro delta t: " << (reading->timestamp() - lastGyroTimestamp)/1000000.0f; //Get deltaT in seconds
    lastGyroTimestamp = reading->timestamp();
    qDebug() << "Gyro: " << reading->x() << " " << reading->y() << " " << reading->z();
}

void IMU::accReadingChanged()
{
    QAccelerometerReading* reading = acc->reading();
    if(lastAccTimestamp > 0)
        qDebug() << "Acc delta t: " << (reading->timestamp() - lastAccTimestamp)/1000000.0f; //Get deltaT in seconds
    lastAccTimestamp = reading->timestamp();
    qDebug() << "Acc: " << reading->x() << " " << reading->y() << " " << reading->z();
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


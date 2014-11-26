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
    QQuickItem(parent)
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


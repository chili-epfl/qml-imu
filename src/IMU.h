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
 * @file cvcamera.h
 * @brief QML wrapper for IMU processing
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#ifndef IMU_H
#define IMU_H

#include<QQuickItem>
#include<QVector3D>
#include<QQuaternion>

#include<opencv2/highgui/highgui.hpp>

class IMU : public QQuickItem {
Q_OBJECT
    Q_DISABLE_COPY(IMU)
    Q_PROPERTY(QVector3D rotation READ getRotation NOTIFY rotationChanged)
    Q_PROPERTY(QQuaternion rotationQuat READ getRotationQuat NOTIFY rotationChanged)

public:

    /**
     * @brief Creates a new IMU processor with the given QML parent
     *
     * @param parent The QML parent
     */
    IMU(QQuickItem* parent = 0);

    /**
     * @brief Destroys this IMU processor
     */
    ~IMU();

    /**
     * @brief Returns the latest estimated rotation in angle-axis representation
     *
     * @return Latest estimated rotation w.r.t ground inertial frame
     */
    QVector3D getRotation();

    /**
     * @brief Returns the latest estimated rotation in quaternion representation
     *
     * @return Latest estimated rotation w.r.t ground inertial frame
     */
    QQuaternion getRotationQuat();

public slots:

    /**
     * @brief Callback for a parent change event
     *
     * @param parent New parent
     */
    void changeParent(QQuickItem* parent);

signals:

    /**
     * @brief Emitted when the estimated rotation is changed
     */
    void rotationChanged();

};

#endif /* IMU_H */


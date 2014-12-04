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
 * @file AccelerometerBiasEstimator.h
 * @brief QML wrapper for accelerometer bias estimation
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#ifndef ACCELEROMETERBIASESTIMATOR_H
#define ACCELEROMETERBIASESTIMATOR_H

#include<QQuickItem>
#include<QtSensors/QSensor>
#include<QtSensors/QAccelerometer>
#include<QtSensors/QAccelerometerReading>
#include<QVector3D>

#include"ExtendedKalmanFilter.h"

class AccelerometerBiasEstimator : public QQuickItem {
Q_OBJECT
    Q_DISABLE_COPY(AccelerometerBiasEstimator)
    Q_PROPERTY(QString accId READ getAccId WRITE setAccId NOTIFY accIdChanged)
    Q_PROPERTY(QVector3D bias READ getBias NOTIFY biasChanged)
    Q_PROPERTY(qreal covTrace READ getCovTrace NOTIFY biasChanged)

public:

    /**
     * @brief Creates a new accelerometer bias estimator with the given QML parent
     *
     * @param parent The QML parent
     */
    AccelerometerBiasEstimator(QQuickItem* parent = 0);

    /**
     * @brief Destroys this accelerometer bias estimator
     */
    ~AccelerometerBiasEstimator();

    /**
     * @brief Returns the current accelerometer identifier, if any
     *
     * @return Current accelerometer identifier if exists and is opened, empty string if not
     */
    QString getAccId();

    /**
     * @brief Sets the new accelerometer identifier and opens the corresponding device for data
     *
     * Identifier is set to empty string if device can't be opened
     *
     * @param accId New accelerometer identifier
     */
    void setAccId(QString const& accId);

    /**
     * @brief Returns the latest estimated accelerometer bias
     *
     * @return Latest estimated accelerometer bias
     */
    QVector3D getBias();

    /**
     * @brief Gets the covariance matrix estimate trace
     *
     * @return Covariance matrix estimate trace
     */
    qreal getCovTrace();

public slots:

    /**
     * @brief Callback for a parent change event
     *
     * @param parent New parent
     */
    void changeParent(QQuickItem* parent);

private slots:

    /**
     * @brief Called when a new accelerometer reading is available
     */
    void accReadingChanged();

signals:

    /**
     * @brief Emitted when the accelerometer identifier changes
     */
    void accIdChanged();

    /**
     * @brief Emitted when the estimated accelerometer bias changes
     */
    void biasChanged();

private:

    /**
     * @brief Attempts to open accelerometer with given id
     *
     * If successful, sets rate to maximum and starts sensor
     *
     * @param id Identifier of the sensor to be opened
     *
     * @return Whether successfully opened
     */
    bool openAcc(QByteArray const& id);

    /**
     * @brief Calculates and records predicted observation values
     *
     * Calculates the following:
     * Observation value z(k)
     * Predicted observation value h(x'(k|k+1))
     * Observation matrix H(k)
     */
    void calculateObservation();

    static const int CV_TYPE;       ///< CV_64F or CV_32f

    QString accId;                  ///< Accelerometer identifier, empty string when not open
    QAccelerometer* acc;            ///< Accelerometer sensor, nullptr when not open
    quint64 lastAccTimestamp;       ///< Most recent accelerometer measurement timestamp

    ExtendedKalmanFilter filter;    ///< Filter that estimates current tilt and linear acceleration in ground frame

    cv::Mat observation;            ///< Temporary matrix to hold the gravity observation, assumed to be accelerometer value
    cv::Mat predictedObservation;   ///< Temporary matrix to hold what we expect gravity vector is based on rotation

    qreal R_g_k_0;                  ///< Gravity observation constant noise coefficient
    qreal R_g_k_g;                  ///< Gravity observation gravity norm dependent noise coefficient

    QVector3D a;                    ///< Latest acceleration vector in local frame in m/s^2
    QVector3D bias;                 ///< Accelerometer bias in local frame in m/s^2
    qreal covTrace;                 ///< Trace of the covariance matrix estimate
};

#endif /* ACCELEROMETERBIASESTIMATOR_H */


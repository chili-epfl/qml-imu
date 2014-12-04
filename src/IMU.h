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
 * @file IMU.h
 * @brief QML wrapper for IMU processing
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-11-26
 */

#ifndef IMU_H
#define IMU_H

#include<QQuickItem>
#include<QtSensors/QSensor>
#include<QtSensors/QAccelerometer>
#include<QtSensors/QAccelerometerReading>
#include<QtSensors/QGyroscope>
#include<QtSensors/QGyroscopeReading>
#include<QtSensors/QMagnetometer>
#include<QtSensors/QMagnetometerReading>
#include<QVector3D>
#include<QQuaternion>
#include<QMatrix4x4>

#include"ExtendedKalmanFilter.h"

class IMU : public QQuickItem {
Q_OBJECT
    Q_DISABLE_COPY(IMU)
    Q_PROPERTY(QString gyroId READ getGyroId WRITE setGyroId NOTIFY gyroIdChanged)
    Q_PROPERTY(QString accId READ getAccId WRITE setAccId NOTIFY accIdChanged)
    Q_PROPERTY(QString magId READ getMagId WRITE setMagId NOTIFY magIdChanged)
    Q_PROPERTY(QVector3D rotAxis READ getRotAxis NOTIFY stateChanged)
    Q_PROPERTY(qreal rotAngle READ getRotAngle NOTIFY stateChanged)
    Q_PROPERTY(QVector3D linearAcceleration READ getLinearAcceleration NOTIFY stateChanged)
    Q_PROPERTY(bool startupComplete READ isStartupComplete NOTIFY startupCompleteChanged)

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
     * @brief Returns the current gyroscrope identifier, if any
     *
     * @return Current gyroscope identifier if exists and is opened, empty string if not
     */
    QString getGyroId();

    /**
     * @brief Sets the new gyroscope identifier and opens the corresponding device for data
     *
     * Identifier is set to empty string if device can't be opened
     *
     * @param gyroId New gyroscope identifier
     */
    void setGyroId(QString const& gyroId);

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
     * @brief Returns the current magnetometer identifier, if any
     *
     * @return Current magnetometer identifier if exists and is opened, empty string if not
     */
    QString getMagId();

     /**
     * @brief Sets the new magnetometer identifier and opens the corresponding device for data
     *
     * Identifier is set to empty string if device can't be opened
     *
     * @param accId New magnetometer identifier
     */
    void setMagId(QString const& magId);

    /**
     * @brief Returns the latest estimated rotation's axis in angle-axis representation
     *
     * @return Latest estimated rotation's axis w.r.t ground inertial frame
     */
    QVector3D getRotAxis();

    /**
     * @brief Returns the latest estimated rotation's angle in angle-axis representation
     *
     * @return Latest estimated rotation's angle w.r.t ground inertial frame
     */
    qreal getRotAngle();

    /**
     * @brief Returns the latest estimated linear acceleration in ground inertial frame
     *
     * @return Latest estimated linear acceleration in m/s^2
     */
    QVector3D getLinearAcceleration();

    /**
     * @brief Gets whether the startup time is complete
     *
     * @return Whether the startup time is complete and the data is stable
     */
    bool isStartupComplete();

    /**
     * @brief Sets the last pose as the current pose for the displacement calculation
     */
    Q_INVOKABLE void resetDisplacement();

    /**
     * @brief Gets the displacement of a point in the local frame since the last call of this function or resetDisplacement()
     *
     * @param r Vector from the IMU location to the desired location in local rigid body frame
     *
     * @return Transform from last pose to current pose in the ground inertial frame
     */
    Q_INVOKABLE QMatrix4x4 getDisplacement(QVector3D const& r);

public slots:

    /**
     * @brief Callback for a parent change event
     *
     * @param parent New parent
     */
    void changeParent(QQuickItem* parent);

private slots:

    /**
     * @brief Called when a new gyroscope reading is available
     */
    void gyroReadingChanged();

    /**
     * @brief Called when a new accelerometer reading is available
     */
    void accReadingChanged();

    /**
     * @brief Called when a new magnetometer reading is available
     */
    void magReadingChanged();

signals:

    /**
     * @brief Emitted when the gyroscope identifier changes
     */
    void gyroIdChanged();

    /**
     * @brief Emitted when the accelerometer identifier changes
     */
    void accIdChanged();

    /**
     * @brief Emitted when the magnetometer identifier changes
     */
    void magIdChanged();

    /**
     * @brief Emitted when the estimated rotation and linear acceleration changes
     */
    void stateChanged();

    /**
     * @brief Emitted when the startup time ends
     */
    void startupCompleteChanged();

private:

    /**
     * @brief Attemps to open gyroscope with given id
     *
     * If successful, sets rate to maximum and starts sensor
     *
     * @param id Identifier of the sensor to be opened
     *
     * @return Whether successfully opened
     */
    bool openGyro(QByteArray const& id);

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
     * @brief Attempts to open magnetometer with given id
     *
     * If successful, sets rate to maximum and starts sensor
     *
     * @param id Identifier of the sensor to be opened
     *
     * @return Whether successfully opened
     */
    bool openMag(QByteArray const& id);

    /**
     * @brief Normalizes given quaternion to unit norm
     *
     * @param quat Quaternion to normalize
     */
    void normalizeQuat(cv::Mat& quat);

    /**
     * @brief Ensures the sign of the quaternion is right so that we prevent quaternion unwinding
     *
     * @param prevQuat Previous value of the quaternion
     * @param quat Current value of the quaternion to be corrected
     */
    void shortestPathQuat(cv::Mat& prevQuat, cv::Mat& quat);

    /**
     * @brief Calculates and records the process values
     *
     * Calculates the following:
     * Process value f(x'(k-1|k-1), U(k-1))
     * Transition matrix F(k-1)
     * Process noise covariance matrix Q(k-1)
     */
    void calculateProcess();

    /**
     * @brief Calculates and records predicted observation values
     *
     * Calculates the following:
     * Observation value z(k)
     * Predicted observation value h(x'(k|k+1))
     * Observation matrix H(k)
     */
    void calculateObservation();

    /**
     * @brief Calculates and stores the rotation in angle-axis representation
     */
    void calculateOutputRotation(); //TODO: FIX NAME OF THIS

    /**
     * @brief Updates the displacement translation using the latest accelerometer data
     */
    void updateDisplacementTranslation();

    static const int CV_TYPE;       ///< CV_64F or CV_32f
    static const qreal EPSILON;     ///< FLT_EPSILON or DBL_EPSILON

    QString gyroId;                 ///< Gyroscope identifier, empty string when not open
    QString accId;                  ///< Accelerometer identifier, empty string when not open
    QString magId;                  ///< Magnetometer identifier, empty string when not open

    QGyroscope* gyro;               ///< Gyroscope sensor, nullptr when not open
    QAccelerometer* acc;            ///< Accelerometer sensor, nullptr when not open
    QMagnetometer* mag;             ///< Magnetometers sensor, nullptr when not open

    quint64 lastGyroTimestamp;      ///< Most recent gyroscope measurement timestamp
    quint64 lastAccTimestamp;       ///< Most recent accelerometer measurement timestamp
    quint64 lastMagTimestamp;       ///< Most recent magnetometer measurement timestamp

    unsigned int gyroSilentCycles;  ///< Data publish cycles without gyroscope data
    unsigned int accSilentCycles;   ///< Data publish cycles without accelerometer data
    unsigned int magSilentCycles;   ///< Data publish cycles without magnetometer data

    ExtendedKalmanFilter filter;    ///< Filter that estimates current tilt and linear acceleration in ground frame

    cv::Mat Q;                      ///< Base for process noise covariance matrix
    cv::Mat process;                ///< Temporary matrix to hold the calculated process value, the rotation
    cv::Mat observation;            ///< Temporary matrix to hold the gravity observation, assumed to be accelerometer value
    cv::Mat predictedObservation;   ///< Temporary matrix to hold what we expect gravity vector is based on rotation

    cv::Mat statePreHistory;        ///< Previous value of the a priori state for quaternion sign correction
    cv::Mat statePostHistory;       ///< Previous value of the a posteriori state for quaternion sign correction

    qreal startupTime;              ///< Time to spend with low R entries for a fast initial stabilization of absolute axes
    qreal R_g_startup;              ///< Diagonal entries of gravity obs noise during startup, must be lower than usual
    qreal R_y_startup;              ///< Diagonal entries of magnetometer obs noise during startup, must be lower than usual

    qreal R_g_k_0;                  ///< Gravity observation constant noise coefficient
    qreal R_g_k_w;                  ///< Gravity observation angular velocity dependent noise coefficient
    qreal R_g_k_g;                  ///< Gravity observation gravity norm dependent noise coefficient
    qreal R_y_k_0;                  ///< Unit y vector observation constant noise coefficient
    qreal R_y_k_w;                  ///< Unit y vector observation angular velocity dependent noise coefficient
    qreal R_y_k_g;                  ///< Unit y vector observation gravity norm dependent noise coefficient
    qreal R_y_k_n;                  ///< Unit y vector observation norm noise coefficient
    qreal R_y_k_d;                  ///< Unit y vector observation dip angle noise coefficient

    QVector3D w;                    ///< Latest angular velocity in local frame in rad/s
    qreal wDeltaT;                  ///< Latest time slice for angular velocity
    QVector3D a;                    ///< Latest acceleration vector in local frame in m/s^2
    qreal aDeltaT;                  ///< Latest time slice for linear acceleration
    QVector3D m;                    ///< Latest magnetic vector in local frame in milliTeslas
    bool magDataReady;              ///< Whether new magnetometer data arrived

    qreal w_norm;                   ///< Magnitude of the latest angular velocity, for noise calculation
    qreal a_norm;                   ///< Magnitude of the latest acceleration, for noise calculation
    qreal m_norm;                   ///< Magnitude of the latest magnetic vector, for noise calculation
    qreal m_norm_mean;              ///< Mean magnitude of the measured magnetic vector
    qreal m_dip_angle_mean;         ///< Mean dip angle between magnetic vector and floor vector
    qreal m_mean_alpha;             ///< Smoothing factor for magnetic mean and dip angle mean estimate

    QVector3D a_bias;               ///< Accelerometer bias

    /// @defgroup imuState State of the IMU frame w.r.t ground inertial frame
    /// @{
    QVector3D rotAxis;              ///< Rotation axis in axis-angle representation
    qreal rotAngle;                 ///< Rotation angle in axis-angle representation
    QVector3D linearAcceleration;   ///< Linear acceleration w.r.t ground inertial frame in m/s^2
    /// @}

    QVector3D velocity;             ///< Estimated linear velocity

    QQuaternion prevRotation;       ///< Rotation of IMU frame in the global frame at the last displacement request
    QVector3D dispTranslation;      ///< Translation of IMU frame in the global frame since the last displacement request
};

#endif /* IMU_H */


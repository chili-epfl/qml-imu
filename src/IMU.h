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

#include <QQuickItem>
#include <QtSensors/QSensor>
#include <QtSensors/QAccelerometer>
#include <QtSensors/QAccelerometerReading>
#include <QtSensors/QGyroscope>
#include <QtSensors/QGyroscopeReading>
#include <QtSensors/QMagnetometer>
#include <QtSensors/QMagnetometerReading>
#include <QVector3D>
#include <QQuaternion>

#include "ExtendedKalmanFilter.h"

/**
 * @brief Object that estimates the device's absolute orientation and linear acceleration through 9-axis IMU measurements in real time
 */
class IMU : public QQuickItem {
    /* *INDENT-OFF* */
    Q_OBJECT
    /* *INDENT-ON* */

    Q_DISABLE_COPY(IMU)

    /** @brief Gyroscope sensor ID, set to the first found gyroscope's ID at startup and can be changed later */
    Q_PROPERTY(QString gyroId READ getGyroId WRITE setGyroId NOTIFY gyroIdChanged)

    /** @brief Accelerometer sensor ID, set to the first found accelerometer's ID at startup and can be changed later */
    Q_PROPERTY(QString accId READ getAccId WRITE setAccId NOTIFY accIdChanged)

    /** @brief Magnetometer sensor ID, set to the first found magnetometer's ID at startup and can be changed later */
    Q_PROPERTY(QString magId READ getMagId WRITE setMagId NOTIFY magIdChanged)

    /** @brief Accelerometer bias to be subtracted from every raw measurement, default (0,0,0) */
    Q_PROPERTY(QVector3D accBias MEMBER a_bias)

    /** @brief Latest estimated rotations's unit axis in angle-axis representation */
    Q_PROPERTY(QVector3D rotAxis READ getRotAxis NOTIFY stateChanged)

    /** @brief Latest estimated rotation's angle in degrees in angle-axis representation */
    Q_PROPERTY(qreal rotAngle READ getRotAngle NOTIFY stateChanged)

    /** @brief Latest estimated rotation in unit quaternion representation */
    Q_PROPERTY(QQuaternion rotQuat READ getRotQuat NOTIFY stateChanged)

    /** @brief Latest estimated linear acceleration in m/s^2 */
    Q_PROPERTY(QVector3D linearAcceleration READ getLinearAcceleration NOTIFY stateChanged)

    /** @brief Displacement calculation target's translation in local rigid body frame, with respect to the IMU */
    Q_PROPERTY(QVector3D targetTranslation MEMBER targetTranslation)

    /** @brief Displacement calculation target's rotation in local rigid body frame, with respect to the IMU */
    Q_PROPERTY(QQuaternion targetRotation MEMBER targetRotation)

    /** @brief Unit floor vector in the displacement calculation target's frame */
    Q_PROPERTY(QVector3D targetFloorVector READ getTargetFloorVector NOTIFY stateChanged)

    /** @brief Length of the startup period in seconds where measurements affect the state more in order to settle quickly to the true orientation, default 1 */
    Q_PROPERTY(qreal startupTime WRITE setStartupTime READ getStartupTime)

    /** @brief Whether the startup period ended */
    Q_PROPERTY(bool startupComplete READ isStartupComplete NOTIFY startupCompleteChanged)

    /** @brief Diagonal entries of the measurement covariance matrix related to the gravity observation during startup, default 10^-1 */
    Q_PROPERTY(qreal R_g_startup MEMBER R_g_startup)

    /** @brief Diagonal entries of the measurement covariance matrix related to the magnetometer observation during startup, default 10^-3 */
    Q_PROPERTY(qreal R_y_startup MEMBER R_y_startup)

    /** @brief Constant coefficient in gravity measurement covariance diagonal entries, default 1.0 */
    Q_PROPERTY(qreal R_g_k_0 MEMBER R_g_k_0)

    /** @brief Angular velocity magnitude coefficient in gravity measurement covariance diagonal entries, default 7.5 */
    Q_PROPERTY(qreal R_g_k_w MEMBER R_g_k_w)

    /** @brief Acceleration magnitude deviation coefficient in gravity measurement covariance diagonal entries, default 10.0 */
    Q_PROPERTY(qreal R_g_k_g MEMBER R_g_k_g)

    /** @brief Constant coefficient in magnetometer measurement covariance diagonal entries, default 10.0 */
    Q_PROPERTY(qreal R_y_k_0 MEMBER R_y_k_0)

    /** @brief Angular velocity magnitude coefficient in magnetometer measurement covariance diagonal entries, default 7.5 */
    Q_PROPERTY(qreal R_y_k_w MEMBER R_y_k_w)

    /** @brief Acceleration magnitude deviation coefficient in magnetometer measurement covariance diagonal entries, default 10.0 */
    Q_PROPERTY(qreal R_y_k_g MEMBER R_y_k_g)

    /** @brief Magnetic vector magnitude deviation coefficient in magnetometer measurement covariance diagonal entries, default 20.0 */
    Q_PROPERTY(qreal R_y_k_n MEMBER R_y_k_n)

    /** @brief Magnetic vector dip angle deviation coefficient in magnetometer measurement covariance diagonal entries, default 15.0 */
    Q_PROPERTY(qreal R_y_k_d MEMBER R_y_k_d)

    /** @brief Smoothing factor when estimating magnetic vector mean magnitude and mean dip angle, between 0 and 1, default 0.99 */
    Q_PROPERTY(qreal m_mean_alpha MEMBER m_mean_alpha)

    /** @brief Angular velocity magnitude decay coefficient in velocity estimate, larger values make decay threshold smaller and decay sharper, default 15.0 */
    Q_PROPERTY(qreal velocityWDecay MEMBER velocityWDecay)

    /** @brief Acceleration magnitude decay coefficient in velocity estimate, larger values make decay threshold smaller and decay sharper, default 8.0 */
    Q_PROPERTY(qreal velocityADecay MEMBER velocityADecay)

public:

    /** @cond DO_NOT_DOCUMENT */

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
    QString getGyroId(){ return gyroId; }

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
    QString getAccId(){ return accId; }

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
    QString getMagId(){ return magId; }

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
     * @return Latest estimated rotation's axis (unit norm) w.r.t ground inertial frame
     */
    QVector3D getRotAxis(){ return rotAxis; }

    /**
     * @brief Returns the latest estimated rotation's angle in angle-axis representation
     *
     * @return Latest estimated rotation's angle in degrees w.r.t ground inertial frame
     */
    qreal getRotAngle(){ return rotAngle; }

    /**
     * @brief Returns the latest estimated rotation in unit quaternion representation
     *
     * @return Latest estimated rotation
     */
    QQuaternion getRotQuat(){ return rotQuat; }

    /**
     * @brief Returns the latest estimated linear acceleration in ground inertial frame
     *
     * @return Latest estimated linear acceleration in m/s^2
     */
    QVector3D getLinearAcceleration(){ return linearAcceleration; }

    /**
     * @brief Sets the startup time where measurements have much greater effect and restarts startup
     *
     * @param startupTime Startup time in seconds, must be larger than 0 to have an effect
     */
    void setStartupTime(qreal startupTime);

    /**
     * @brief Gets the current remaining portion of the startup time
     *
     * @return Remaining portion of the startup time
     */
    qreal getStartupTime();

    /**
     * @brief Gets whether the startup time is complete
     *
     * @return Whether the startup time is complete and the data is stable
     */
    bool isStartupComplete();

    /**
     * @brief Returns the latest floor vector
     *
     * @return Latest floor vector in the target frame
     */
    QVector3D getTargetFloorVector(){ return targetFloorVector; }

    /** @endcond */

public slots:

    /**
     * @brief Sets the last pose as the current pose for the displacement calculation
     */
    void resetDisplacement();

    /**
     * @brief Gets the position change of the target point since the last call to resetDisplacement()
     *
     * @return Position of current pose in the last pose frame
     */
    QVector3D getLinearDisplacement();

    /**
     * @brief Gets the rotation change of the local frame since the last call to resetDisplacement()
     *
     * @return Rotation of current pose in the last pose frame
     */
    QQuaternion getAngularDisplacement();

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

    /** @cond DO_NOT_DOCUMENT */

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

    /** @endcond */

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
     * @brief Calculates and stores the rotation and the linear acceleration in global ground inertial frame
     */
    void calculateOutput();

    /**
     * @brief Updates the displacement values
     *
     * Updates the displacement translation and velocity estimate.
     * Displacement rotation needs no update and can be calculated from the current device rotation.
     */
    void updateDisplacement();

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
    cv::Mat process;                ///< Temporary matrix to hold the calculated process value, i.e rotation and acceleration
    cv::Mat observation;            ///< Temporary matrix to hold gravity and magnetometer observation
    cv::Mat predictedObservation;   ///< Temporary matrix to hold gravity and magnetometer expectation based on current rotation

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

    // State of the IMU frame w.r.t ground inertial frame
    // {
    QVector3D rotAxis;              ///< Rotation axis in axis-angle representation
    qreal rotAngle;                 ///< Rotation angle in axis-angle representation
    QQuaternion rotQuat;            ///< Rotation in unit qutaernion representation
    QVector3D linearAcceleration;   ///< Linear acceleration w.r.t ground inertial frame in m/s^2
    // }

    QVector3D targetTranslation;    ///< Translation of target in local rigid body frame for which displacement will be calculated
    QQuaternion targetRotation;     ///< Rotation of target in local rigid body frame for which displacement will be calculated
    QVector3D targetFloorVector;    ///< Unit floor vector in the target frame
    QVector3D velocity;             ///< Estimated linear velocity

    qreal velocityWDecay;           ///< How quickly velocity estimate decays w.r.t angular velocity magnitude
    qreal velocityADecay;           ///< How quickly velocity estimate decays w.r.t linear acceleration magnitude

    QQuaternion prevRotation;       ///< Rotation of IMU frame in the global frame at the last displacement reset
    QVector3D dispTranslation;      ///< Translation of IMU frame in the global frame since the last displacement reset
};

#endif /* IMU_H */

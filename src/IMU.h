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
 *
 * Overview
 * --------
 *
 * This object is the center of sensor fusion and IMU processing. It consists
 * of an extended Kalman filter estimating the device orientation and linear
 * acceleration.
 *
 * Inputs to the filter are gyroscope data (mandatory), accelerometer data
 * (if present) and magnetometer data (if present). The main premise is that
 * gyroscope data can provide reliable short term orientation data if
 * integrated, but will drift with time. Accelerometer data provides the sum of
 * linear acceleration and the reaction due to gravitational acceleration.
 * Assuming that linear acceleration is small compared to gravity, drift in
 * orientation due to the integration of gyroscope data is corrected in the
 * long term since the acceleration vector will be pointing away from the floor
 * on average. While this corrects the drift in rotations around the x and y
 * axes, magnetometer data is used to correct the drift in rotation around the
 * z axis due to its pointing towards a linear combination of floor and
 * magnetic north, depending on the device location on the Earth.
 *
 * In addition to this, since the device orientation with respect to the ground
 * is known, the gravity component from the accelerometer data can be removed
 * (assuming it is always pointing away from the floor and has magnitude 9.81
 * m/s^2), leaving us with linear acceleration only.
 *
 * Without magnetometer data, the rotation estimate will drift around the z
 * axis. Without accelerometer data, the rotation estimate will drift around
 * the y axis and linear acceleration cannot be estimated. Without both, the
 * rotation estimate will drift around all axes and linear acceleration cannot
 * be estimated. Without gyroscope data, the filter cannot operate.
 *
 * Finally, this object also provides on-demand angular and linear displacement
 * values via respective API calls, calculated from the a posteriori state
 * estimates. These values represent the displacement of the device at demand
 * time t in the device frame at initial time t0. This initial time can be
 * indicated on demand via another API call.
 *
 * The State Vector and the Process
 * --------------------------------
 *
 * The state is described as the following 7x1 vector:
 *
 * X(t) = (q_w(t), q_x(t), q_y(t), q_z(t), a_x(t), a_y(t), a_z(t))^T
 *      = (q(t), a(t))^T
 *
 * Here, q(t) is the orientation of the local body frame with respect to the
 * global inertial ground frame. a(t) is the linear acceleration of the local
 * body frame in the global inertial ground frame; this linear acceleration
 * does not include and is separated from the reaction force against gravity.
 *
 * The transition process from state t-1 to state t is made according to the
 * gyroscope and accelerometer input and is nonlinear in terms of the state.
 * The following describes the process for the orientation (and is linear):
 *
 * q(t|t-1) = q(t-1|t-1) + deltaT*(dq(t-1|t-1)/dt)
 *          = q(t-1|t-1) + deltaT*(1/2)*q(t-1|t-1)*(0, w(t-1))
 *          = q(t-1|t-1) + deltaT*(1/2)*omega(t-1|t-1)*w(t-1)
 *
 * Here  w(t-1) is the measured angular velocity and deltaT is the time passed
 * since the last gyroscope measurement. In the second line, q(t-1)*(0, w(t-1))
 * represents Hamilton product of the orientation and the angular velocity,
 * considered as a quaternion with zero scalar part. omega(t) is a 4x3 matrix
 * that is defined as follows and implements the Hamilton product:
 *
 *            / -q_x(t)    -q_y(t)    -q_z(t) \
 * omega(t) = |  q_w(t)    -q_z(t)     q_y(t) |
 *            |  q_z(t)     q_w(t)    -q_x(t) |
 *            \ -q_y(t)     q_x(t)     q_w(t) /
 *
 * The following describes the process for the linear acceleration (and is
 * nonlinear):
 *
 * a(t|t-1) = rot(q(t-1|t-1)^-1)*a_m(t-1) - (0, 0, g)^T
 *
 * Here, rot() represents the orthogonal rotation matrix given the input
 * quaternion. a_m(t-1) represents the raw accelerometer measurement and g is
 * a constant equal to 9.81 m/s^2. This process, assuming that q(t|t) describes
 * the orientation of the local body with respect to the ground inertial frame
 * and is stable, rotates the total acceleration into the ground inertial
 * frame and removes the gravity component in order to give the linear
 * acceleration.
 *
 * Note that the process on linear acceleration does not depend on the previous
 * linear acceleration state.
 *
 * Given these, the process can be expressed as the following:
 *
 * X(t|t-1) = f(X(t-1|t-1), u(t-1))
 * u(t-1) = (w(t-1), a_m(t-1))^T
 *
 * where f implements the above processes. As per the extended Kalman filter,
 * the state transition matrix is 7x7 and is described as follows:
 *
 * F(t-1) = (df/dX)(X(t-1|t-1), u(t-1))
 *
 * The process noise is, as usual, described by a 7x7 covariance matrix (Q)
 * that should be tuned by the user. As a design choice, the components of this
 * matrix are multiplied by deltaT at each step. This is based on the premise
 * that the error involved in integrating the angular velocity increases with
 * an increased time gap between each angular velocity measurement.
 *
 * The Observation Vector
 * ----------------------
 *
 * The observation vector is a 6x1 vector and is described as:
 *
 * z(t) = (a_m_x(t), a_m_y(t), a_m_z(t), m_x(t), m_y(t), m_z(t))^T
 *      = (a_m(t), m(t))^T
 *
 * where a_m(t) is the measured acceleration vector with accelerometer bias
 * subtracted. This bias is to be measured/tuned by the user and is constant
 * throughout runtime. m(t) is the measured magnetic vector with z component
 * rejected and normalized. This is achieved by assuming that the local body
 * orientation estimate is accurate and stable. If this assumption is made,
 * then m(t) can be calculated as the following:
 *
 * m(t) = m~(t)/|m~(t)|
 * m~(t) = m_m(t) - (m_m(t).v_floor(t))*v_floor(t)
 * v_floor(t) = rot(q(t|t-1)^-1)*(0, 0, 1)^T
 *
 * where rot() represents the orthogonal rotation matrix given the input
 * quaternion, . represents dot product, v_floor(t) is the unit floor vector
 * in the local body frame and m_m(t) is the measured magnetic vector.
 *
 * The predicted observation vector is also a 6x1 vector, and using the a
 * priori state estimate, is described (non-linearly) as follows:
 *
 * h(X(t|t-1)) = (rot(q(t|t-1)^-1)*(0, 0, g)^T, rot(q(t|t-1)^-1)*(0, 1, 0)^T)
 *
 * where g is 9.81 m/s^2 and rot() represents the orthogonal rotation matrix
 * given the input quaternion. This design assumes that the measurement a_m(t),
 * on average, points away from the floor with magnitude 9.81 m/s^2 and the
 * measurement m(t), on average, points towards the magnetic north with
 * magnitude 1, and that they are stable in the long term. With these
 * assumptions, the local body frame's z axis aligns with the floor vector and
 * y axis aligns with the magnetic north, correcting the drift in orientation
 * due to the integration of angular velocity measurements. The x axis is
 * therefore also implicitly well defined as the cross product of y and z axes.
 *
 * As per the extended Kalman filter, the observation matrix is 6x7 and is
 * described as follows:
 *
 * H(t) = (dh/dX)(X(t|t-1))
 *
 * An important assumption made by the model is that magnetometer measurements
 * are less frequent than or at approximately equal frequency as accelerometer
 * measurements. Under this assumption, Kalman filter correction step is done
 * at every accelerometer reading. When there is no magnetometer reading
 * present during a correction step, the magnetic vector measurement and
 * predicted measurement are set to zero, causing the magnetometer part of the
 * observation to have no effect on the device y axis. This update model design
 * is purely due to computational performance reasons.
 *
 * The Observation Noise
 * ---------------------
 *
 * The observation noise is described by a 6x6 covariance matrix (R(t)) that is
 * dependent on the acceleration and magnetic vector measurements. It is
 * defined as follows:
 *
 *        /            |            \
 *        |  I*R_g(t)  |     0      |
 *        |       (3x3)|            |
 * R(t) = |-------------------------|
 *        |            |            |
 *        |      0     |  I*R_y(t)  |
 *        \            |       (3x3)/
 *
 * where I is a 3x3 identity matrix. R_g(t) is the gravity measurement noise
 * coefficient and is, by design, defined as follows:
 *
 * R_g(t) = R_g_k_0 +
 *          R_g_k_w*|w(t)| +
 *          R_g_k_g*|g - |a_m(t)||
 *
 * where R_g_k are coefficients to be tuned by the user, w(t) is the angular
 * velocity measurement, a_m(t) is the accelerometer measurement and g is 9.81
 * m/s^2. This design aims that more noise is attributed to accelerometer
 * measurements when the device is not stable, i.e angular velocity is present
 * and it is accelerated externally. This causes the measurements to
 * appropriately affect the state less.
 *
 * R_y(t) is the magnetic vector measurement noise coefficient and is, by
 * design, defined as follows:
 *
 * R_y(t) = R_y_k_0 +
 *          R_y_k_w*|w(t)| +
 *          R_y_k_g*|g - |a_m(t)|| +
 *          R_y_k_n*||m_m(t)| - m_norm_mean(t)| +
 *          R_y_k_d*|m_dip_angle(t) - m_dip_angle_mean(t)|
 *
 * where R_y_k are coefficients to be tuned by the user and w(t), a_m(t) and g
 * are the same as above. m_m(t) is the magnetic vector measurement and
 * m_norm_mean(t) is the magnitude mean of m_m(t) estimated by a low pass
 * filter as follows:
 *
 * m_norm_mean(t+1) = m_mean_alpha*m_norm_mean(t) + (1 - m_mean_alpha)*|m(t)|
 *
 * m_dip_angle(t) is defined as the angle between the estimated z axis and the
 * measured magnetic vector in radians. m_dip_angle_mean(t) is the mean of
 * m_dip_angle(t) estimated similarly by a low pass filter as follows:
 *
 * m_dip_angle_mean(t+1) = m_mean_alpha*m_dip_angle_mean(t) + (1 - m_mean_alpha)*m_dip_angle(t)
 *
 * In the above equations, the smoothing factor m_mean_alpha should be tuned
 * by the user and should be between 0 (meaning no smoothing) and 1 (meaning
 * no update).
 *
 * By design, similar to the accelerometer measurement noise, more noise is
 * attributed to the magnetic vector measurement when the device is not stable,
 * i.e angular velocity and linear acceleration are above base levels. In
 * addition to the previous case, more noise is attributed also when the
 * magnetic vector magnitude and the dip angle fluctuate. Since the magnitude
 * and the dip angle are ideally constant on any point on Earth (locally and in
 * short time scales), this fluctuation indicates the presence of non-white
 * magnetic noise due to e.g ferromagnetic materials nearby. This is reflected
 * onto the model by increasing the measurement noise estimate, causing the
 * measurements to affect the state less.
 *
 * Finally, during the initial time period (length of which is to be tuned by
 * the user) of the filter's runtime, R_g and R_y are not calculated as above
 * but are set to significantly lower values (to be tuned by the user). The
 * assumption during this period is that the device is stable and accelerometer
 * and magnetometer readings accurately describe the floor vector with
 * magnitude 9.81 m/s^2 and the magnetic north vector respectively. This
 * ensures that at startup, the orientation of the device settles quickly to
 * correct values instead of settling in a "slow drift correction" fashion that
 * is by design the regular operation of the observation measurements.
 *
 * Linear and Angular Displacement
 * -------------------------------
 *
 * The user can request the linear and angular displacement of a target local
 * reference frame on the rigid device body on-demand via respective API calls.
 * These calls return the translation and rotation of this frame in the same
 * frame in a previous point in time. This time is indicated by a displacement
 * resetting API call. In other words, the following can be requested on demand
 * by the user:
 *
 * T_target(t in t0)^T
 * R_target(t in t0)^T
 *
 * where T and R represent translation and rotation, and t0 indicates the last
 * displacement reset time.
 *
 * Calculation of the linear displacement requires the estimation of local body
 * velocity. With inertial sensors, this is only possible with the integration
 * of the linear acceleration and is impossible otherwise without any reference
 * external to the device. This causes the velocity to drift in a random walk
 * fashion (due to accelerometer noise) and causes this estimate to be
 * unbounded in magnitude.
 *
 * Because of this, an additional assumption is required. In this case, the
 * assumption is that the IMU resides on a handheld device and is stationary
 * (has velocity zero) when it is inertially stable, i.e the angular velocity
 * and the linear acceleration magnitudes are close to zero. Please note that
 * this is not necessarily true for an IMU on an arbitrary device; a good
 * example would be an aerial vehicle or a sattelite.
 *
 * This assumption is modeled by two sigmoid decay factors multiplied by the
 * velocity estimate at each time step. This is described as follows:
 *
 * v(t+1) = d_w(t)*d_a(t)*(v(t) + deltaT*a(t+1))
 *
 * where a(t+1) denotes the linear acceleration, deltaT denotes the time
 * between t and t+1 and d denote the decay factors. They are designed as
 * follows:
 *
 * d_w(t) = (1 - exp(-w_decay*|w(t)|))/(1 + exp(-w_decay*|w(t)|))
 * d_a(t) = (1 - exp(-a_decay*|a(t)|))/(1 + exp(-a_decay*|a(t)|))
 *
 * Here, w_decay and a_decay are coefficients that are to be tuned by the user.
 * The operation of these decay factors is such that they are essentially 1
 * when the magnitudes are above some "threshold" value and smoothly drop to 0
 * when the magnitudes drop below this threshold. This drop could be made sharp
 * or extended in time by adjusting this threshold via w_decay and a_decay.
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
    Q_PROPERTY(QVector3D targetTranslation WRITE setTargetTranslation)
    Q_PROPERTY(QQuaternion targetRotation WRITE setTargetRotation)
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
     * @brief Returns the latest estimated linear acceleration in ground inertial frame
     *
     * @return Latest estimated linear acceleration in m/s^2
     */
    QVector3D getLinearAcceleration(){ return linearAcceleration; }

    /**
     * @brief Sets the displacement calculation target's translation in rigid body inertial frame
     *
     * @param targetTranslation Vector from the IMU location to the desired location in local rigid body frame
     */
    void setTargetTranslation(QVector3D const& targetTranslation){ this->targetTranslation = targetTranslation; }

    /**
     * @brief Sets the displacement calculation target's rotation in rigid body inertial frame
     *
     * @param targetRotation Rotation of the desired target in local rigid body frame
     */
    void setTargetRotation(QQuaternion const& targetRotation){ this->targetRotation = targetRotation; }

    /**
     * @brief Gets whether the startup time is complete
     *
     * @return Whether the startup time is complete and the data is stable
     */
    bool isStartupComplete();

public slots:

    /**
     * @brief Sets the last pose as the current pose for the displacement calculation
     */
    void resetDisplacement();

    /**
     * @brief Gets the position change of the target point since the last call to resetDisplacement()
     *
     * @return Position from last pose to current pose in the ground inertial frame
     */
    QVector3D getLinearDisplacement();

    /**
     * @brief Gets the rotation change of the local frame since the last call to resetDisplacement()
     *
     * @return Rotation from last pose to current pose in the ground inertial frame
     */
    QQuaternion getAngularDisplacement();

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

    /// @defgroup imuState State of the IMU frame w.r.t ground inertial frame
    /// @{
    QVector3D rotAxis;              ///< Rotation axis in axis-angle representation
    qreal rotAngle;                 ///< Rotation angle in axis-angle representation
    QVector3D linearAcceleration;   ///< Linear acceleration w.r.t ground inertial frame in m/s^2
    /// @}

    QVector3D targetTranslation;    ///< Translation of target in local rigid body frame for which displacement will be calculated
    QQuaternion targetRotation;     ///< Rotation of target in local rigid body frame for which displacement will be calculated
    QVector3D velocity;             ///< Estimated linear velocity

    qreal velocityWDecay;           ///< How quickly velocity estimate decays w.r.t angular velocity magnitude
    qreal velocityADecay;           ///< How quickly velocity estimate decays w.r.t linear acceleration magnitude

    QQuaternion prevRotation;       ///< Rotation of IMU frame in the global frame at the last displacement reset
    QVector3D dispTranslation;      ///< Translation of IMU frame in the global frame since the last displacement reset
};

#endif /* IMU_H */


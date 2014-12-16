qml-imu
=======

qml-imu is a sensor fusion module that operates on gyroscope, accelerometer and
magnetometer sensor data to estimate the device orientation and linear
acceleration (without the gravity component) in the fixed global ground frame.
The ground frame is defined to have its z axis point away from the floor and
its y axis point towards magnetic north.

The following is required for qml-imu to work:

  - Ubuntu `14.04`
  - Qt `5.3.2`
  - OpenCV `3.0.0-alpha`

QML API
-------

Properties related to sensors themselves:

>  - **gyroId** :   `QString` - Gyroscope sensor ID, set to the first found gyroscope's ID at startup and can be changed later
>  - **accId** :    `QString` - Accelerometer sensor ID, set to the first found accelerometer's ID at startup and can be changed later
>  - **magID** :    `QString` - Magnetometer sensor ID, set to the first found magnetometer's ID at startup and can be changed later
>  - **accBias** :  `QVector3D`, default `(0,0,0)` - Accelerometer bias to be subtracted from every raw measurement

Startup related properties:

>  - **startupTime** :      `qreal`, default `1` - Length of the startup period in seconds where measurements affect the state more in order to settle quickly to the true orientation
>  - **startupComplete** :  `bool` - Whether the startup period ended

Measurement noise covariance related properties:

>  - **R\_g\_startup** :  `qreal`, default `10^-1` - Diagonal entries of the measurement covariance matrix related to the gravity observation during startup
>  - **R\_y\_startup** :  `qreal`, default `10^-3` - Diagonal entries of the measurement covariance matrix related to the magnetometer observation during startup
>  - **R\_g\_k\_0** :      `qreal`, default `1.0` - Constant coefficient in gravity measurement covariance diagonal entries
>  - **R\_g\_k\_w** :      `qreal`, default `7.5` - Angular velocity magnitude coefficient in gravity measurement covariance diagonal entries
>  - **R\_g\_k\_g** :      `qreal`, default `10.0` - Acceleration magnitude deviation coefficient in gravity measurement covariance diagonal entries
>  - **R\_y\_k\_0** :      `qreal`, default `10.0` - Constant coefficient in magnetometer measurement covariance diagonal entries
>  - **R\_y\_k\_w** :      `qreal`, default `7.5` - Angular velocity magnitude coefficient in magnetometer measurement covariance diagonal entries
>  - **R\_y\_k\_g** :      `qreal`, default `10.0` - Acceleration magnitude deviation coefficient in magnetometer measurement covariance diagonal entries
>  - **R\_y\_k\_n** :      `qreal`, default `20.0` - Magnetic vector magnitude deviation coefficient in magnetometer measurement covariance diagonal entries
>  - **R\_y\_k\_d** :      `qreal`, default `15.0` - Magnetic vector dip angle deviation coefficient in magnetometer measurement covariance diagonal entries
>  - **m\_mean\_alpha** : `qreal`, default `0.99` - Smoothing factor when estimating magnetic vector mean magnitude and mean dip angle, between `0` and `1`

Linear velocity estimation related properties:

>  - **velocityWDecay** : `qreal`, default `15.0` - Angular velocity magnitude decay coefficient in velocity estimate, larger values make decay threshold smaller and decay sharper
>  - **velocityADecay** : `qreal`, default `8.0` - Acceleration magnitude decay coefficient in velocity estimate, larger values make decay threshold smaller and decay sharper

Sensor fusion outputs, all in the fixed ground frame:

>  - **rotAxis** :              `QVector3D` - Latest estimated rotations's unit axis in angle-axis representation
>  - **rotAngle** :             `qreal` - Latest estimated rotation's angle in degrees in angle-axis representation
>  - **rotQuat** :              `QQuaternion` - Latest estimated rotation in unit quaternion representation
>  - **linearAcceleration** :   `QVector3D` - Latest estimated linear acceleration in m/s^2

Displacement related properties and invokables:

>  - **targetTranslation** :        `QVector3D` - Displacement calculation target's translation in local rigid body frame, with respect to the IMU
>  - **targetRotation** :           `QQuaternion` - Displacement calculation target's rotation in local rigid body frame, with respect to the IMU
>  - **resetDisplacement()** :      `void` - Sets the last pose as the current pose for the displacement calculation
>  - **getLinearDisplacement()** :  `QVector3D` - Gets the translation of the target point with respect to its pose at the last call to `resetDisplacement()`
>  - **getAngularDisplacement()** : `QQuaternion` - Gets the rotation of the target point with respect to its pose at the last call to `resetDisplacement()`

Operation
---------

### Overview

At the core of the sensor fusion is a an extended Kalman filter estimating the
device orientation and linear acceleration. [1] describes the fusion scheme
used in this library.

Inputs to the filter are gyroscope data (mandatory), accelerometer data
(if present) and magnetometer data (if present). The main premise is that
gyroscope data can provide reliable short term orientation data if
integrated, but will drift with time. Accelerometer data provides the sum of
linear acceleration and the reaction due to gravitational acceleration.
Assuming that linear acceleration is small compared to gravity, drift in
orientation due to the integration of gyroscope data is corrected in the
long term since the acceleration vector will be pointing away from the floor
on average. While this corrects the drift in rotations around the x and y
axes, magnetometer data is used to correct the drift in rotation around the
z axis due to its pointing towards a linear combination of floor and
magnetic north, depending on the device location on the Earth.

In addition to this, since the device orientation with respect to the ground
is known, the gravity component from the accelerometer data can be removed
(assuming it is always pointing away from the floor and has magnitude 9.81
m/s^2), leaving us with linear acceleration only.

Without magnetometer data, the rotation estimate will drift around the z
axis. Without accelerometer data, the rotation estimate will drift around
the y axis and linear acceleration cannot be estimated. Without both, the
rotation estimate will drift around all axes and linear acceleration cannot
be estimated. Without gyroscope data, the filter cannot operate.

Finally, this object also provides on-demand angular and linear displacement
values via respective API calls, calculated from the a posteriori state
estimates. These values represent the displacement of the device at demand
time `t` in the device frame at initial time `t0`. This initial time can be
indicated on demand via another API call.

### State vector and the process

The state is described as the following 7x1 vector:

```
X(t) = (q_w(t), q_x(t), q_y(t), q_z(t), a_x(t), a_y(t), a_z(t))^T
     = (q(t), a(t))^T
```

Here, `q(t)` is the orientation of the local body frame with respect to the
global inertial ground frame. `a(t)` is the linear acceleration of the local
body frame in the global inertial ground frame; this linear acceleration
does not include and is separated from the reaction force against gravity.

The transition process from state `t-1` to state `t` is made according to the
gyroscope and accelerometer input and is nonlinear in terms of the state.
The following describes the process for the orientation (and is linear):

```
q(t|t-1) = q(t-1|t-1) + deltaT*(dq(t-1|t-1)/dt)
         = q(t-1|t-1) + deltaT*(1/2)*q(t-1|t-1)*(0, w(t-1))
         = q(t-1|t-1) + deltaT*(1/2)*omega(t-1|t-1)*w(t-1)
```

Here `w(t-1)` is the measured angular velocity and `deltaT` is the time passed<F5>
since the last gyroscope measurement. In the second line, `q(t-1)*(0, w(t-1))`
represents Hamilton product of the orientation and the angular velocity,
considered as a quaternion with zero scalar part. `omega(t)` is a 4x3 matrix
that is defined as follows and implements the Hamilton product:

```
           / -q_x(t)    -q_y(t)    -q_z(t) \
omega(t) = |  q_w(t)    -q_z(t)     q_y(t) |
           |  q_z(t)     q_w(t)    -q_x(t) |
           \ -q_y(t)     q_x(t)     q_w(t) /
```

The following describes the process for the linear acceleration (and is
nonlinear):

```
a(t|t-1) = rot(q(t-1|t-1)^-1)*a_m(t-1) - (0, 0, g)^T
```

Here, `rot()` represents the orthogonal rotation matrix given the input
quaternion. `a_m(t-1)` represents the raw accelerometer measurement and `g` is
a constant equal to 9.81 m/s^2. This process, assuming that `q(t|t)` describes
the orientation of the local body with respect to the ground inertial frame
and is stable, rotates the total acceleration into the ground inertial
frame and removes the gravity component in order to give the linear
acceleration.

Note that the process on linear acceleration does not depend on the previous
linear acceleration state.

Given these, the process can be expressed as the following:

```
X(t|t-1) = f(X(t-1|t-1), u(t-1))
u(t-1) = (w(t-1), a_m(t-1))^T
```

where `f` implements the above processes. As per the extended Kalman filter,
the state transition matrix is 7x7 and is described as follows:

```
F(t-1) = (df/dX)(X(t-1|t-1), u(t-1))
```

The process noise is, as usual, described by a 7x7 covariance matrix (`Q`)
that should be tuned by the user. As a design choice, the components of this
matrix are multiplied by `deltaT` at each step. This is based on the premise
that the error involved in integrating the angular velocity increases with
an increased time gap between each angular velocity measurement.

### Observation vector

The observation vector is a 6x1 vector and is described as:

```
z(t) = (a_m_x(t), a_m_y(t), a_m_z(t), m_x(t), m_y(t), m_z(t))^T
     = (a_m(t), m(t))^T
```

where `a_m(t)` is the measured acceleration vector with accelerometer bias
subtracted. This bias is to be measured/tuned by the user and is constant
throughout runtime. `m(t)` is the measured magnetic vector with z component
rejected and normalized. This is achieved by assuming that the local body
orientation estimate is accurate and stable. If this assumption is made,
then `m(t)` can be calculated as the following:

```
m(t) = m~(t)/|m~(t)|
m~(t) = m_m(t) - (m_m(t).v_floor(t))*v_floor(t)
v_floor(t) = rot(q(t|t-1)^-1)*(0, 0, 1)^T
```

where `rot()` represents the orthogonal rotation matrix given the input
quaternion, `.` represents dot product, `v_floor(t)` is the unit floor vector
in the local body frame and `m_m(t)` is the measured magnetic vector.

The predicted observation vector is also a 6x1 vector, and using the a
priori state estimate, is described (non-linearly) as follows:

```
h(X(t|t-1)) = (rot(q(t|t-1)^-1)*(0, 0, g)^T, rot(q(t|t-1)^-1)*(0, 1, 0)^T)
```

where g is 9.81 m/s^2 and `rot()` represents the orthogonal rotation matrix
given the input quaternion. This design assumes that the measurement `a_m(t)`,
on average, points away from the floor with magnitude 9.81 m/s^2 and the
measurement `m(t)`, on average, points towards the magnetic north with
magnitude 1, and that they are stable in the long term. With these
assumptions, the local body frame's z axis aligns with the floor vector and
y axis aligns with the magnetic north, correcting the drift in orientation
due to the integration of angular velocity measurements. The x axis is
therefore also implicitly well defined as the cross product of y and z axes.

As per the extended Kalman filter, the observation matrix is 6x7 and is
described as follows:

```
H(t) = (dh/dX)(X(t|t-1))
```

An important assumption made by the model is that magnetometer measurements
are less frequent than or at approximately equal frequency as accelerometer
measurements. Under this assumption, Kalman filter correction step is done
at every accelerometer reading. When there is no magnetometer reading
present during a correction step, the magnetic vector measurement and
predicted measurement are set to zero, causing the magnetometer part of the
observation to have no effect on the device y axis. This update model design
is purely due to computational performance reasons.

### Observation noise

The observation noise is described by a 6x6 covariance matrix (`R(t)`) that is
dependent on the acceleration and magnetic vector measurements. It is
defined as follows:

```
       /            |            \
       |  I*R_g(t)  |     0      |
       |       (3x3)|            |
R(t) = |-------------------------|
       |            |            |
       |      0     |  I*R_y(t)  |
       \            |       (3x3)/
```

where `I` is a 3x3 identity matrix. The noise components in this matrix are
designed according to the methodology described in [2].

`R_g(t)` is the gravity measurement noise coefficient and is, by design,
defined as follows:

```
R_g(t) = R_g_k_0 +
         R_g_k_w*|w(t)| +
         R_g_k_g*|g - |a_m(t)||
```

where `R_g_k` are coefficients to be tuned by the user, `w(t)` is the angular
velocity measurement, `a_m(t)` is the accelerometer measurement and `g` is 9.81
m/s^2. This design aims that more noise is attributed to accelerometer
measurements when the device is not stable, i.e angular velocity is present
and it is accelerated externally. This causes the measurements to
appropriately affect the state less.

`R_y(t)` is the magnetic vector measurement noise coefficient and is, by
design, defined as follows:

```
R_y(t) = R_y_k_0 +
         R_y_k_w*|w(t)| +
         R_y_k_g*|g - |a_m(t)|| +
         R_y_k_n*||m_m(t)| - m_norm_mean(t)| +
         R_y_k_d*|m_dip_angle(t) - m_dip_angle_mean(t)|
```

where `R_y_k` are coefficients to be tuned by the user and `w(t)`, `a_m(t)` and
`g` are the same as above. `m_m(t)` is the magnetic vector measurement and
`m_norm_mean(t)` is the magnitude mean of `m_m(t)` estimated by a low pass
filter as follows:

```
m_norm_mean(t+1) = m_mean_alpha*m_norm_mean(t) + (1 - m_mean_alpha)*|m(t)|
```

`m_dip_angle(t)` is defined as the angle between the estimated z axis and the
measured magnetic vector in radians. `m_dip_angle_mean(t)` is the mean of
`m_dip_angle(t)` estimated similarly by a low pass filter as follows:

```
m_dip_angle_mean(t+1) = m_mean_alpha*m_dip_angle_mean(t) + (1 - m_mean_alpha)*m_dip_angle(t)
```

In the above equations, the smoothing factor `m_mean_alpha` should be tuned
by the user and should be between 0 (meaning no smoothing) and 1 (meaning
no update).

By design, similar to the accelerometer measurement noise, more noise is
attributed to the magnetic vector measurement when the device is not stable,
i.e angular velocity and linear acceleration are above base levels. In
addition to the previous case, more noise is attributed also when the
magnetic vector magnitude and the dip angle fluctuate. Since the magnitude
and the dip angle are ideally constant on any point on Earth (locally and in
short time scales), this fluctuation indicates the presence of non-white
magnetic noise due to e.g ferromagnetic materials nearby. This is reflected
onto the model by increasing the measurement noise estimate, causing the
measurements to affect the state less.

Finally, during the initial time period (length of which is to be tuned by
the user) of the filter's runtime, `R_g` and `R_y` are not calculated as above
but are set to significantly lower values (to be tuned by the user). The
assumption during this period is that the device is stable and accelerometer
and magnetometer readings accurately describe the floor vector with
magnitude 9.81 m/s^2 and the magnetic north vector respectively. This
ensures that at startup, the orientation of the device settles quickly to
correct values instead of settling in a "slow drift correction" fashion that
is by design the regular operation of the observation measurements.

### Linear and angular displacement

The user can request the linear and angular displacement of a target local
reference frame on the rigid device body on-demand via respective API calls.
These calls return the translation and rotation of this frame in the same
frame in a previous point in time. This time is indicated by a displacement
resetting API call. In other words, the following can be requested on demand
by the user:

```
T_target(t in t0)^T
R_target(t in t0)^T
```

where `T` and `R` represent translation and rotation, and `t0` indicates the
last displacement reset time.

Calculation of the linear displacement requires the estimation of local body
velocity. With inertial sensors, this is only possible with the integration
of the linear acceleration and is impossible otherwise without any reference
external to the device. This causes the velocity estimate to drift in a random
walk fashion (due to accelerometer noise) and causes it to be unbounded in
magnitude.

Because of this, an additional assumption is required. In this case, the
assumption is that the IMU resides on a handheld device and is stationary
(has velocity zero) when it is inertially stable, i.e the angular velocity
and the linear acceleration magnitudes are close to zero. Please note that
this is not necessarily true for an IMU on an arbitrary device; a good
example would be an aerial vehicle or a sattelite.

This assumption is modeled by two sigmoid decay factors multiplied by the
velocity estimate at each time step. This is described as follows:

```
v(t+1) = d_w(t)*d_a(t)*(v(t) + deltaT*a(t+1))
```

where `a(t+1)` denotes the linear acceleration, `deltaT` denotes the time
between `t` and `t+1` and `d` denote the decay factors. They are designed as
follows:

```
d_w(t) = (1 - exp(-w_decay*|w(t)|))/(1 + exp(-w_decay*|w(t)|))
d_a(t) = (1 - exp(-a_decay*|a(t)|))/(1 + exp(-a_decay*|a(t)|))
```

Here, `w(t)` is the angular velocity measurement, `a(t)` is the linear
acceleration estimated by the filter and `w_decay` and `a_decay` are
coefficients that are to be tuned by the user. The operation of these decay
factors is such that they are essentially 1 when the magnitudes are above some
"threshold" value and smoothly drop to 0 when the magnitudes drop below this
threshold. This drop could be made sharp or extended in time by adjusting this
threshold via `w_decay` and `a_decay`.

### References

[1] S. Sabatelli, M. Galgani, L. Fanucci, A. Rocchi, *"A Double-Stage Kalman
Filter for Orientation Tracking With an Integrated Processor in 9-D IMU"*,
Instrumentation and Measurement, IEEE Transactions on, vol.62, no.3,
pp.590-598, March 2013

[2] D. Jurman, M. Jankovec, R. Kamnik, M. Topič, *"Calibration and Data Fusion
Solution for the Miniature Attitude and Heading Reference System"*, Sensors and
Actuators A: Physical, vol.138, no.2, pp.411-420, August 2007


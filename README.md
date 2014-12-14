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
>  - **linearAcceleration** :   `QVector3D` - Latest estimated linear acceleration in m/s^2

Displacement related properties and invokables:

>  - **targetTranslation** :        `QVector3D` - Displacement calculation target's translation in local rigid body frame, with respect to the IMU
>  - **targetRotation** :           `QQuaternion` - Displacement calculation target's rotation in local rigid body frame, with respect to the IMU
>  - **resetDisplacement()** :      `void` - Sets the last pose as the current pose for the displacement calculation
>  - **getLinearDisplacement()** :  `QVector3D` - Gets the translation of the target point with respect to its pose at the last call to `resetDisplacement()`
>  - **getAngularDisplacement()** : `QQuaternion` - Gets the rotation of the target point with respect to its pose at the last call to `resetDisplacement()`





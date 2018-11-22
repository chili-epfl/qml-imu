accelerometer-bias-estimator
============================

Simple estimator tool for the accelerometer bias. Follow the build instructions in [the qml-imu README](../../README.md)
before trying to run this sample. It is tested with Qt 5.11.0 on the following:

  - Android 8.1.0 with Ubuntu 18.04 host with Android API 14, Android SDK Tools 26.1.1 and Android NDK r15c

build & run [Android]
--------------------------------------

Load the project into QtCreator and press the Run button with the big green arrow. Put the device on a stable surface
and record the estimates, which may later be used in an `IMU` object to increase accuracy of acceleration estimates.

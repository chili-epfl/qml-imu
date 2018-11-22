imu-gui
=======

Example use of `qml-imu`. Follow the build instructions in [the qml-imu README](../../README.md) before trying to run
this sample. It is tested with Qt 5.11.0 on the following:

  - Android 8.1.0 with Ubuntu 18.04 host with Android API 14, Android SDK Tools 26.1.1 and Android NDK r15c

build & run [Android]
--------------------------------------

Load the project into QtCreator and press the Run button with the big green arrow. The screen should show the global
frame of reference with the red, green and blue arrows (for x, y, z respectively) and the linear acceleration (without
gravity) with the cyan arrow. Depending on the natural orientation of the device (i.e orientation of the IMU with
respect to the screen), you might have to rotate the screen to a portrait or one of the landscape orientations so that
the global frame aligns with the screen, resulting in a natural augmented reality-like feel.

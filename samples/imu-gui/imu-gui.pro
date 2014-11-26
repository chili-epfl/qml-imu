TEMPLATE = app

QT += qml quick sensors

qtHaveModule(3d){
    QT += 3d
}

SOURCES += src/main.cpp

RESOURCES += qml.qrc

android {
    target.path = /libs/armeabi-v7a
    export(target.path)
    INSTALLS += target
    export(INSTALLS)

    ANDROID_EXTRA_LIBS = \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_core.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_imgproc.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_imgcodecs.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_flann.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_features2d.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_calib3d.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_video.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_videoio.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_highgui.so \
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libnative_camera_r4.4.0.so \ #Adapt this line to your device's Android version
    #    $$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/lib/libchilitags.so \
    #    $$[QT_INSTALL_QML]/Chilitags/libchilitagsplugin.so \
        $$[QT_INSTALL_QML]/IMU/libimuplugin.so
}


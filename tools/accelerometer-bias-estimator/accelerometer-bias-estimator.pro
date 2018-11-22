TEMPLATE = app

QT += qml quick sensors

SOURCES += src/main.cpp

RESOURCES += qml.qrc

android {
    target.path = /libs/armeabi-v7a
    export(target.path)
    INSTALLS += target
    export(INSTALLS)

    ANDROID_EXTRA_LIBS = \
        $$(ANDROID_NDK_ROOT)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/libopencv_core.so
}


TEMPLATE = lib
TARGET = imuplugin

CONFIG += qt plugin c++11 nostrip
CONFIG -= android_install

QT += qml quick sensors 3d

QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS_RELEASE -= -O2

QMAKE_CXXFLAGS += -O3
QMAKE_CXXFLAGS_RELEASE += -O3

TARGET = $$qtLibraryTarget($$TARGET)
uri = IMU

HEADERS += \
    src/ExtendedKalmanFilter.h \
    src/IMU.h \
    src/AccelerometerBiasEstimator.h \
    src/MatrixTransform3D.h \
    src/IMUPlugin.h

SOURCES += \
    src/ExtendedKalmanFilter.cpp \
    src/IMU.cpp \
    src/AccelerometerBiasEstimator.cpp \
    src/MatrixTransform3D.cpp \
    src/IMUPlugin.cpp

LIBS += -lopencv_core

android {

    #Enable automatic NEON vectorization
    QMAKE_CXXFLAGS -= -mfpu=vfp
    QMAKE_CXXFLAGS_RELEASE -= -mfpu=vfp
    QMAKE_CXXFLAGS += -mfpu=neon -ftree-vectorize -ftree-vectorizer-verbose=1 -mfloat-abi=softfp
    QMAKE_CXXFLAGS_RELEASE += -mfpu=neon -ftree-vectorize -ftree-vectorizer-verbose=1 -mfloat-abi=softfp

    INCLUDEPATH += $(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/jni/include
    LIBS += -L$(ANDROID_STANDALONE_TOOLCHAIN)/sysroot/usr/share/opencv/sdk/native/libs/armeabi-v7a/
}

OTHER_FILES += qmldir

#Install plugin library, qmldir
qmldir.files = qmldir
unix {
    installPath = $$[QT_INSTALL_QML]/$$replace(uri, \\., /)
    qmldir.path = $$installPath
    target.path = $$installPath
    INSTALLS += target qmldir
}


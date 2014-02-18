QT += core gui multimedia

CONFIG += qt staticlib

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tracking
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    FrameLabel.cpp \
    capturethread.cpp \
    imagebuffer.cpp \
    cimage.cpp \
    processingthread.cpp \
    controller.cpp \
    tracker.cpp

HEADERS  += mainwindow.h \
    FrameLabel.h \
    capturethread.h \
    imagebuffer.h \
    cimage.h \
    processingthread.h \
    controller.h \
    config.h \
    tracker.h

FORMS    += mainwindow.ui

RESOURCES += \
    resources.qrc

#ANDROID_EXTRA_LIBS = ../../../../../opt/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_java.so ../../../../../opt/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libopencv_info.so ../../../../../opt/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a/libnative_camera_r4.4.0.so

#A_OPENCV = /opt/OpenCV-android-sdk/sdk/native
#LIBS += -L$$A_OPENCV/libs/armeabi-v7a/

LIBS += -lopencv_highgui \
        -lopencv_video \
#        -lopencv_androidcamera \
        -lopencv_imgproc \
        -lopencv_core


#LIBS += -L$$A_OPENCV/3rdparty/libs/armeabi-v7a/ \
#        -ltbb \
#        -llibpng \
#        -llibjpeg \
#        -llibtiff \
#        -llibjasper \
#        -lIlmImf

#INCLUDEPATH += $$A_OPENCV/jni/include/opencv
#INCLUDEPATH += $$A_OPENCV/jni/include/opencv2
#INCLUDEPATH += $$A_OPENCV/jni/include

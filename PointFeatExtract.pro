TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

INCLUDEPATH += /home/hupan/download/opencv-2.4.9/release/usr/local/include/


LIBS += /home/hupan/download/opencv-2.4.9/release/usr/local/lib/libopencv_core.so \
		/home/hupan/download/opencv-2.4.9/release/usr/local/lib/libopencv_imgproc.so \
		/home/hupan/download/opencv-2.4.9/release/usr/local/lib/libopencv_highgui.so

SOURCES += main.cpp \
	interestPointLocate.cpp \
    imgMath.cpp \
    imgIO.cpp

HEADERS += \
	interestPointLocate.h \
    imgMath.h \
    imgIO.h


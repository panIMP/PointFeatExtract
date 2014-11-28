TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

INCLUDEPATH += G:\opencv_mingw_hpbuild\install\include\


LIBS += G:\opencv_mingw_hpbuild\install\lib\libopencv_core244d.dll.a \
        G:\opencv_mingw_hpbuild\install\lib\libopencv_imgproc244d.dll.a \
        G:\opencv_mingw_hpbuild\install\lib\libopencv_highgui244d.dll.a \
        G:\opencv_mingw_hpbuild\install\lib\libopencv_features2d244d.dll.a \
        G:\opencv_mingw_hpbuild\install\lib\libopencv_nonfree244d.dll.a \
        G:\opencv_mingw_hpbuild\install\lib\libopencv_legacy244d.dll.a

SOURCES += main.cpp \
	interestPointLocate.cpp \
	imgMath.cpp \
	imgIO.cpp \
	error.cpp

HEADERS += \
	interestPointLocate.h \
	imgMath.h \
	imgIO.h \
	error.h

OTHER_FILES += \
    backup.txt


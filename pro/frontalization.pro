TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += ../main.cpp \
    ../frontalization.cpp \
    ../calib/calib.cpp \
    ../calib/POSIT.cpp \
    ../calib/util/calib_util.cpp \
    ../frontalUtil.cpp


INCLUDEPATH += /usr/local/include/opencv -I/usr/local/include

LIBS += -L/usr/local/lib -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d\
        -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml \
        -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab

HEADERS += \
    ../base.h \
    ../frontalization.h \
    ../calib/POSIT.h \
    ../calib/util/calib_util.h \
    ../calib/calib.h \
    ../frontalUtil.h

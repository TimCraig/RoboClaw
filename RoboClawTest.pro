TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

TARGET = RoboClawTest

include(D:/Projects/Workspace/ProjectsCommon/Boost.pri)

win32: DEFINES += _USE_MATH_DEFINES

SOURCES += main.cpp \
    RoboClawFS.cpp \
    PanTilt.cpp \
    DiffDrive.cpp \
    Odometry.cpp

HEADERS += \
    RoboClawFS.h \
    PanTilt.h \
    DiffDrive.h \
    Odometry.h



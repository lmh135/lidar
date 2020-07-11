#-------------------------------------------------
#
# Project created by QtCreator 2018-06-25T19:37:00
#
#-------------------------------------------------

QT       += core gui
QT       += opengl
LIBS += -lGLU
LIBS += -llas
LIBS += -lboost_system
LIBS += /usr/local/lib/libpcl_*.so
LIBS += -larmadillo

#LIBS += /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_*.so.*


INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/pcl-1.8
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/boost

#INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QTgroundStation
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    widget.cpp \
    select.cpp \
    collectionrecover.cpp \
    point_receive.cpp \
    shm.cpp \
    filtercloud.cpp \
    contour.cpp \
    triangulate.cpp \
    predicates.cpp \
    points_array.cpp \
    registration.cpp \
    fusion.cpp \
    checkout_rpy.cpp

HEADERS += \
        mainwindow.h \
    widget.h \
    select.h \
    collectionrecover.h \
    point_receive.h \
    shm.h \
    data_construct.h \
    filtercloud.h \
    contour.h \
    predicates.h \
    points_array.h \
    triangulate.h \
    registration.h \
    fusion.h \
    checkout_rpy.h

FORMS += \
        mainwindow.ui



RESOURCES += \
    icon.qrc

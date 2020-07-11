#-------------------------------------------------
#
# Project created by QtCreator 2018-04-14T22:16:47
#11111
#-------------------------------------------------

QT       += core gui
QT       += opengl
LIBS += -lGLU
LIBS += -llas
LIBS += -lboost_system
LIBS += -larmadillo
LIBS += /usr/local/lib/libpcl_*.so*
############

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GroundStation
TEMPLATE = app

SOURCES += main.cpp\
    mainwindow.cpp \
    showwindow.cpp \
    collectrecoverwindow.cpp \
    widget.cpp \
    point_receive.cpp \
    shm.cpp \
    triangulate.cpp \
    predicates.cpp \
    points_array.cpp \
    select.cpp \
    registration.cpp \
    widget2.cpp \
    widget3.cpp \
    filterwindow.cpp \
    contour.cpp

HEADERS  += mainwindow.h \
    showwindow.h \
    collectrecoverwindow.h \
    widget.h \
    point_receive.h \
    shm.h \
    data_construct.h \
    predicates.h \
    points_array.h \
    triangulate.h \
    select.h \
    registration.h \
    widget2.h \
    widget3.h \
    filterwindow.h \
    contour.h

FORMS    += mainwindow.ui \
    showwindow.ui \
    collectrecoverwindow.ui \
    registration.ui \
    filterwindow.ui

#INCLUDEPATH += /home/liminghui/桌面/激光/离线地面站加删减功能/
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/pcl-1.8
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/boost
DISTFILES +=





TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/driver.cpp

INCLUDEPATH += \
    /opt/ros/melodic/include \

HEADERS += \
    src/driver.h

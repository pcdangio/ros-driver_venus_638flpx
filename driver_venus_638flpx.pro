TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/driver.cpp \
    src/main_rpi.cpp \
    src/ros_node.cpp \
    src/rpi_driver.cpp

INCLUDEPATH += \
    /opt/ros/melodic/include \

HEADERS += \
    src/driver.h \
    src/ros_node.h \
    src/rpi_driver.h

DISTFILES += \
    CMakeLists.txt \
    README.md \
    package.xml

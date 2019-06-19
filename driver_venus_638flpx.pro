TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/driver.cpp \
    src/main.cpp \
    src/ros_node.cpp

INCLUDEPATH += \
    /opt/ros/melodic/include \

HEADERS += \
    src/driver.h \
    src/ros_node.h

DISTFILES += \
    CMakeLists.txt \
    README.md \
    package.xml

TARGET = grasper-gui

CONFIG += \
    c++14 \
    qt \
    plugin \
    nostrip \
    link_pkgconfig

PKGCONFIG += \
    roscpp \
    tf \
    image_transport \
    visualization_msgs \
    grasper_msg

QT += gui qml widgets charts core

SOURCES += \
    src/main.cpp \
    src/main_controller.cpp \
    src/force_controller_worker.cpp \
    src/thermistor_worker.cpp \
    src/ultrasound_worker.cpp \
    src/pulseox_worker.cpp

INCLUDEPATH += inc

HEADERS += \
    inc/main_controller.hpp \
    inc/force_controller_worker.hpp \
    inc/thermistor_worker.hpp \
    inc/ultrasound_worker.hpp \
    inc/pulseox_worker.hpp

RESOURCES += \
    resources.qrc

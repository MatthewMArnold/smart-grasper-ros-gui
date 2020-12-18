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

QT += gui qml widgets core printsupport quick

SOURCES += \
    src/main.cpp \
    src/main_controller.cpp \
    src/force_controller_worker.cpp \
    src/thermistor_worker.cpp \
    src/pulseox_worker.cpp \
    src/ultrasonic_worker.cpp \
    src/bioimpedance_worker.cpp \
    src/qcustomplot.cpp \
    src/custom_plot_item.cpp \
    src/error_controller.cpp

INCLUDEPATH += inc

HEADERS += \
    inc/main_controller.hpp \
    inc/force_controller_worker.hpp \
    inc/thermistor_worker.hpp \
    inc/pulseox_worker.hpp \
    inc/ultrasonic_worker.hpp \
    inc/bioimpedance_worker.hpp \
    inc/qcustomplot.h \
    inc/custom_plot_item.hpp \
    inc/error_controller.hpp

RESOURCES += \
    resources.qrc

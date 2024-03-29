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
    grasper_msg \
    cv_bridge

QT += gui qml widgets core printsupport quick

SOURCES += \
    src/main.cpp \
    src/main_controller.cpp \
    src/force_controller_worker.cpp \
    src/qcustomplot.cpp \
    src/custom_plot_item.cpp \
    src/error_controller.cpp \
    src/image_displayer.cpp \
    src/value_updater.cpp \
    src/sensor_measurement.cpp \
    src/sensor_measurement_instances.cpp \
    src/image_display_button.cpp

INCLUDEPATH += \
    inc \
    /usr/local/include

HEADERS += \
    inc/main_controller.hpp \
    inc/force_controller_worker.hpp \
    inc/qcustomplot.h \
    inc/custom_plot_item.hpp \
    inc/error_controller.hpp \
    inc/image_displayer.hpp \
    inc/error_reporter.hpp \
    inc/value_updater.hpp \
    inc/sensor_measurement.hpp \
    inc/sensor_measurement_instances.hpp \
    inc/image_display_button.hpp

RESOURCES += \
    resources.qrc

LIBS += \
     -lopencv_core \
     -lopencv_highgui \
     -lopencv_imgproc \
     -lopencv_features2d \
     -lopencv_flann \
     -lopencv_ml \
     -lopencv_objdetect \
     -lopencv_photo \
     -lopencv_stitching \
     -lopencv_superres \
     -lopencv_ts \
     -lopencv_video \
     -lopencv_videostab \
     -lopencv_calib3d \
     -lopencv_imgcodecs

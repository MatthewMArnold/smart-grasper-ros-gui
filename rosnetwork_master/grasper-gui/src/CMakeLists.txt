set(SRCS
    main.cpp
)

qt5_add_resources(RESOURCES ../resources.qrc)

add_executable(grasper-gui ${SRCS} ${RESOURCES})
target_link_libraries(grasper-gui Qt5::Core Qt5::Qml Qt5::Widgets)

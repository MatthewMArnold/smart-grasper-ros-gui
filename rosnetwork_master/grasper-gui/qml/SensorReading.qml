import QtQuick 2.9
import QtQuick.Controls 2.2
import ValueUpdater 1.0
import "qrc:/qml"

Column {
    property alias sensor_heading: header.text
    property alias sensor_reading: displayBorder.display_text

    spacing: Constants.component_margin

    Text {
        id: header
        color: Constants.primary_font_color
        font.weight: Font.Bold
        anchors.horizontalCenter: parent.horizontalCenter
        font.pixelSize: Constants.small_text_size
        horizontalAlignment: Text.AlignHCenter
    }

    Rectangle {
        id: displayBorder
        objectName: "displayBorder"
        height: Constants.default_small_text_rect_height
        color: Constants.light_purple
        radius: Constants.default_small_edge_radius
        anchors.horizontalCenter: parent.horizontalCenter
        width: Constants.measurement_reading_box_width
        property alias display_text: sensorReading.value

        ValueUpdater {
            id: sensorReading
            objectName:"sensorReading"
            height: parent.height
            width: parent.width
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }
}

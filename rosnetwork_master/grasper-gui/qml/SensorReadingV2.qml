import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import ValueUpdater 1.0
import "qrc:/qml"

/*
 * Use the "onClicked" signal to see when this radio button has been pressed.
 */
RadioButton {
    readonly property int button_radius: Constants.default_large_edge_radius
    property alias sensor_heading: header.text
    property alias sensor_reading: displayBorder.display_text

    signal onSelected(bool selected)

    onCheckedChanged: { onSelected(checked) }

    style: RadioButtonStyle {
        indicator: Rectangle {
            implicitWidth: Constants.default_measurement_panel_wdith
            implicitHeight: header.height +
                            displayBorder.height +
                            Constants.component_margin * 5
            border.width: 0
            color: Constants.subsection_background_color
            radius: Constants.default_large_edge_radius
            Rectangle {
                anchors.fill: parent
                visible: control.checked
                color: Constants.background_color
                radius: Constants.default_large_edge_radius
                anchors.margins: Constants.component_margin
            }
        }
    }

    Text {
        id: header
        color: Constants.primary_font_color
        font.weight: Font.Bold
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: Constants.component_margin * 2
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
        anchors.top: header.bottom
        anchors.topMargin: Constants.component_margin
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

import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Column {
    id: sensorControlPanelColumn

    function forceSwitchOn() {
        forceMeasureSwitch.switchOn()
        forceMeasureSwitch.disable()
    }
    function forceSwitchOff() {
        forceMeasureSwitch.switchOff()
        forceMeasureSwitch.enable()
    }

    width: Constants.default_panel_width

    function turnOnAllMeasurements() {
        pulseOxSwitch.switchOn()
        temperatureSwitch.switchOn()
        forceMeasureSwitch.switchOn()
        velOfSoundSwitch.switchOn()
        impedanceSwitch.switchOn()
    }

    function turnOffAllMeasurements() {
        pulseOxSwitch.switchOff()
        temperatureSwitch.switchOff()
        forceMeasureSwitch.switchOff()
        velOfSoundSwitch.switchOff()
        impedanceSwitch.switchOff()
    }

    function disableMeasurementButtons() {
        pulseOxSwitch.disable()
        temperatureSwitch.disable()
        forceMeasureSwitch.disable()
        velOfSoundSwitch.disable()
        impedanceSwitch.disable()
    }

    function enableMeasurementButtons() {
        pulseOxSwitch.enable()
        temperatureSwitch.enable()
        forceMeasureSwitch.enable()
        velOfSoundSwitch.enable()
        impedanceSwitch.enable()
    }

    Rectangle {
        id: sensorControlPanelHeader
        width: parent.width
        height: 50
        color: Constants.background_color
        radius: Constants.default_small_edge_radius

        Text {
            color: Constants.primary_font_color
            text: "Sensor Control Panel"
            font.weight: Font.ExtraBold
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            horizontalAlignment: Text.AlignHCenter
            fontSizeMode: Text.HorizontalFit
            font.pixelSize: Constants.small_text_size
        }
    }

    Rectangle {
        id: sensorControlPanel
        objectName: "sensorControlPanel"

        width: parent.width
        height: switchCol.height + Constants.component_margin * 2
        color: Constants.subsection_background_color
        radius: Constants.default_large_edge_radius

        Column {
            id: switchCol
            objectName: "switchCol"
            width: parent.width
            anchors.verticalCenter: parent.verticalCenter

            SwitchDelegateWithText {
                id: pulseOxSwitch
                objectName: "pulseOxSwitch"
                anchors.right: parent.right
                anchors.rightMargin: Constants.component_margin
                side_text: "Pulse Ox"
            }

            SwitchDelegateWithText {
                id: temperatureSwitch
                objectName: "temperatureSwitch"
                anchors.right: parent.right
                anchors.rightMargin: Constants.component_margin
                side_text: "Temperature"
            }

            SwitchDelegateWithText {
                id: forceMeasureSwitch
                objectName: "forceMeasurementSwitch"
                anchors.right: parent.right
                anchors.rightMargin: Constants.component_margin
                side_text: "Force"
            }

            SwitchDelegateWithText {
                id: velOfSoundSwitch
                objectName: "velOfSoundSwitch"
                anchors.right: parent.right
                anchors.rightMargin: Constants.component_margin
                side_text: "Velocity of Sound"
            }

            SwitchDelegateWithText {
                id: impedanceSwitch
                objectName: "impedanceSwitch"
                anchors.right: parent.right
                anchors.rightMargin: Constants.component_margin
                side_text: "Impedance"
            }
        }
    }
}

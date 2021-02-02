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

        function enableAll() {
            if (pulseOxSwitch.position === 0) {
                pulseOxSwitch.toggle()
            }
            if (temperatureSwitch.position === 0) {
                temperatureSwitch.toggle()
            }
            if (forceSwitch.position === 0) {
                forceSwitch.toggle()
            }
            if (velocityOfSoundSwitch.position === 0) {
                velocityOfSoundSwitch.toggle()
            }
            if (impedanceSwitch.position === 0) {
                impedanceSwitch.toggle()
            }

            onForceRequestChanged(true)
            onTemperatureRequestChanged(true)
            onPulseOxRequestChanged(true)
            onVelocityOfSoundRequestChanged(true)
            onImpedanceRequestChanged(true)

            pulseOxSwitch.enabled = false
            temperatureSwitch.enabled = false
            forceSwitch.enabled = false
            velocityOfSoundSwitch.enabled = false
            impedanceSwitch.enabled = false

            pulseOxText.opacity = 0.5
            temperatureSwitchText.opacity = 0.5
            forceText.opacity = 0.5
            velocitySwitchText.opacity = 0.5
            impedanceSwitchText.opacity = 0.5
        }

        function disableAll() {
            if (pulseOxSwitch.position === 1) {
                pulseOxSwitch.toggle()
            }
            if (temperatureSwitch.position === 1) {
                temperatureSwitch.toggle()
            }
            if (forceSwitch.position === 1) {
                onForceRequestChanged(false)
            }
            if (velocityOfSoundSwitch.position === 1) {
                velocityOfSoundSwitch.toggle()
            }
            if (forceSwitch.position === 1) {
                forceSwitch.toggle()
            }
            if (impedanceSwitch.position === 1) {
                impedanceSwitch.toggle()
            }

            onForceRequestChanged(false)
            onTemperatureRequestChanged(false)
            onPulseOxRequestChanged(false)
            onVelocityOfSoundRequestChanged(false)
            onImpedanceRequestChanged(false)

            pulseOxSwitch.enabled = true
            temperatureSwitch.enabled = true
            forceSwitch.enabled = true
            velocityOfSoundSwitch.enabled = true
            impedanceSwitch.enabled = true

            pulseOxText.opacity = 1
            temperatureSwitchText.opacity = 1
            forceText.opacity = 1
            velocitySwitchText.opacity = 1
            impedanceSwitch.opacity = 1
            impedanceSwitchText.opacity = 1
        }

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

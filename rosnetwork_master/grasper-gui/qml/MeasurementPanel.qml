import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Column {
    id: sensorControlPanelColumn

    width: Constants.default_panel_width

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
        width: parent.width
        height: switchCol.height
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
            width: parent.width

            SwitchDelegateWithText {
                id: pulseOxSwitch
                anchors.right: parent.right
                side_text: "Pulse Ox"
                onSliderToggled: onPulseOxRequestChanged(toggled)  // TODO replace
            }

            SwitchDelegateWithText {
                id: temperatureSwitch
                anchors.right: parent.right
                side_text: "Temperature"
                onSliderToggled: onTemperatureRequestChanged(toggled)  // TODO replace
            }

            SwitchDelegateWithText {
                id: forceMeasureSwitch
                anchors.right: parent.right
                side_text: "Force"
                onSliderToggled: onForceRequestChanged(toggled)  // TODO replace
            }

            SwitchDelegateWithText {
                id: velOfSoundSwitch
                anchors.right: parent.right
                side_text: "Velocity of Sound"
                onSliderToggled: onVelocityOfSoundRequestChanged(toggled)  // TODO replace
            }

            SwitchDelegateWithText {
                id: impedanceSwitch
                anchors.right: parent.right
                side_text: "Impedance"
                onSliderToggled: onImpedanceRequestChanged(toggled)  // TODO replace
            }
        }
    }
        }

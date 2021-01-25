import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Column {
    property bool motorControllerEnabled: false

    width: Constants.default_panel_width

    Rectangle {
        id: motorControlPanelHeader
        width: parent.width
        height: 50
        color: Constants.background_color
        Text {
            color: Constants.primary_font_color
            text: "Motor Control Panel"
            font.weight: Font.ExtraBold
            anchors.verticalCenter: parent.verticalCenter
            font.pixelSize: Constants.small_text_size
            anchors.horizontalCenter: parent.horizontalCenter
            fontSizeMode: Text.HorizontalFit
            horizontalAlignment: Text.AlignHCenter
        }
    }

    Rectangle {
        id: motorControlPanel
        width: parent.width
        height: parent.height - motorControlPanelHeader.height
        color: Constants.subsection_background_color
        radius: 16
        border.width: 0

        readonly property int default_motor_button_width: 80
        readonly property int default_motor_button_height: 40

        function onMotorControlEnabled() {
            console.log("motor control enabled")
            closeButton.enabled = false
            releaseButton.enabled = true
            desiredForceSlider.enabled = true
            desiredForceText.opacity = 1
            desiredForceSpecified.opacity = 1
            forceSwitch.enabled = 0
            if (forceSwitch.position === 0) {
                forceSwitch.toggle()
            }
            onForceRequestChanged(true)
            onMotorClosedRequested(true)
        }

        function onMotorControlDisabled() {
            console.log("motor control disabled")
            closeButton.enabled = true
            releaseButton.enabled = false
            desiredForceSlider.enabled = false
            desiredForceText.opacity = 0.25
            desiredForceSpecified.opacity = 0.25
            desiredForceSlider.value = 0
            onDesiredForceChanged(0)
            desiredForceSpecified.text = forceController.forceDesired
            forceSwitch.enabled = 1
            if (forceSwitch.position === 1) {
                forceSwitch.toggle()
            }
            onForceRequestChanged(false)
            onMotorClosedRequested(false)
        }

        RoundButton {
            id: releaseButton
            anchors.top: parent.top
            anchors.topMargin: parent.height / 8
            anchors.left: parent.left
            anchors.leftMargin: parent.width / 8
            width: motorControlPanel.default_motor_button_width
            height: motorControlPanel.default_motor_button_height
            enabled: false

            Text {
                text: "Release"
                font.weight: Font.Medium
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                color: Constants.primary_font_color
                font.family: "Courier"
                opacity: releaseButton.enabled ? 1.0 : Constants.default_disabled_opacity
            }

            background: Rectangle {
                id: releaseButtonBackground
                color: Constants.dark_red
                radius: releaseButton.radius
            }

            // TODO fix
            onClicked: {
                if (runAllSensorsButton.isToggled) {
                    // All sensors are disabled when motor control is turned off
                    runAllSensorsButton.onAllSensorsReleased()
                }
                motorControlPanel.onMotorControlDisabled()
            }
        }

        RoundButton {
            id: closeButton
            anchors.top: parent.top
            anchors.topMargin: parent.height / 8
            anchors.right: parent.right
            anchors.rightMargin: parent.width / 8
            width: motorControlPanel.default_motor_button_width
            height: motorControlPanel.default_motor_button_height

            onClicked: motorControlPanel.onMotorControlEnabled()

            background: Rectangle {
                id: closeButtonBackground
                color: Constants.dark_green
                radius: closeButton.radius
            }

            Text {
                id: closeButtonText
                text: "Close"
                font.weight: Font.Medium
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                color: Constants.primary_font_color
                font.family: "Courier"
                opacity: closeButton.enabled ? 1.0 : Constants.default_disabled_opacity
            }
        }

        Text {
            id: desiredForceText
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            color: Constants.primary_font_color
            text: "Desired Force, N"
            font.weight: Font.Bold
            font.pixelSize: Constants.small_text_size
            opacity: Constants.default_disabled_opacity
        }

        Slider {
            id: desiredForceSlider
            anchors.top: desiredForceText.bottom
            anchors.topMargin: Constants.component_margin
            anchors.horizontalCenter: parent.horizontalCenter
            value: 0
            enabled: false

            property double maxDesiredForce: 10.0

            onMoved: {
                onDesiredForceChanged(Math.round(value * maxDesiredForce * 100) / 100)
                desiredForceSpecified.text = forceController.forceDesired
            }

            Text {
                id: desiredForceSpecified
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: Constants.component_margin
                color: Constants.primary_font_color
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: Constants.small_text_size
                opacity: Constants.default_disabled_opacity
                text: "0"
            }
        }
    }
}

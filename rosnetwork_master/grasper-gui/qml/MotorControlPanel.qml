import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Column {
    property bool motorControllerEnabled: false

    width: Constants.default_motor_control_panel_width

    signal motorControllerTurnedOn
    signal motorControllerTurnedOff

    function turnOnMotorController() {
        releaseButton.enabled = true
        closeButton.enabled = false
        desiredForceSlider.enabled = true
        desiredForceText.opacity = 1
        desiredForceSpecified.opacity = 1
        motorControllerTurnedOn()
    }

    function turnOffMotorController() {
        releaseButton.enabled = false
        closeButton.enabled = true
        desiredForceSlider.enabled = false
        desiredForceSlider.reset()
        desiredForceText.opacity = Constants.default_disabled_opacity
        desiredForceSpecified.opacity = Constants.default_disabled_opacity
        motorControllerTurnedOff()
    }

    function disableMotorControlButtons() {
        releaseButton.enabled = false
        closeButton.enabled = false
    }

    function enableMotorControlButtons() {
        releaseButton.enabled = true
        closeButton.enabled = true
    }

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

        RoundButton {
            id: releaseButton
            anchors.top: parent.top
            anchors.topMargin: parent.height / 8
            anchors.left: parent.left
            anchors.leftMargin: parent.width / 8
            width: motorControlPanel.default_motor_button_width
            height: motorControlPanel.default_motor_button_height
            enabled: false

            onClicked: {
                curr_state = 0
                turnOffMotorController()
            }

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
        }

        RoundButton {
            id: closeButton
            anchors.top: parent.top
            anchors.topMargin: parent.height / 8
            anchors.right: parent.right
            anchors.rightMargin: parent.width / 8
            width: motorControlPanel.default_motor_button_width
            height: motorControlPanel.default_motor_button_height

            onClicked:{
                curr_state = 3
                turnOnMotorController()
            }

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

            readonly property double maxDesiredForce: 10.0

            function reset() {
                if (value !== 0) {
                    onDesiredForceChanged(0)
                    value = 0
                    desiredForceSpecified.text = "0"
                }
            }

            onMoved: {
                var f = Math.round(value * maxDesiredForce * 100) / 100
                onDesiredForceChanged(f)
                desiredForceSpecified.text = f
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

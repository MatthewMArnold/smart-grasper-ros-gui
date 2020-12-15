import QtQuick 2.9
import QtQuick.Controls 2.2
import Ros 1.0
import CustomPlot 1.0

ApplicationWindow {
    property int screenHeight: 850
    property int screenWidth: 1048
    property int screenMargin: 32
    property int componentMargin: 8
    property int sensorReadingBoxWidth: 100

    property double actualForce: 0
    property double temperature: 0
    property double velocityOfSound: 0
    property double oxygen: 0
    property double impedance: 0

    id: window
    title: "grasper-gui"
    visible: true
    maximumHeight: screenHeight
    minimumHeight: screenHeight
    maximumWidth: screenWidth
    minimumWidth: screenWidth

    // motor control panel signals
    signal onDesiredForceChanged(double force)
    signal onMotorClosedRequested(bool closeMotor)

    // sensor control panel signals
    signal onPulseOxRequestChanged(bool pulseOxRequested)
    signal onTemperatureRequestChanged(bool temperatureRequested)
    signal onForceRequestChanged(bool forceRequested)
    signal onVelocityOfSoundRequestChanged(bool velocityOfSoundRequested)
    signal onImpedanceRequestChanged(bool impedanceRequested)

    Rectangle {
        id: mainScreen
        width: parent.width
        height: parent.height

        color: "#1c163c"

        RoundButton {
            id: runAllSensorsButton

            z: 100

            property bool isToggled: false

            anchors.bottom: sensorControlPanelColumn.top
            anchors.bottomMargin: componentMargin
            anchors.left: parent.left
            anchors.leftMargin: screenMargin

            width: sensorControlPanelColumn.width + componentMargin + motorControlPanelCol.width
            height: sensorReadingsLeftCol.height - sensorControlPanelColumn.height
            font.pointSize: 36

            function onAllSensorsReleased() {
                isToggled = !isToggled
                if (isToggled) {
                    runAllSensorsButtonShadow.color = "#5a0705"
                    runSensorsText.text = "STOP RUNNING"
                    runSensorsBacground.color = "#fb8784"
                    motorControlPanel.onMotorControlEnabled()
                    sensorControlPanel.enableAll()
                } else {
                    runAllSensorsButtonShadow.color = "#011809"
                    runSensorsText.text = "RUN ALL SENSORS"
                    runSensorsBacground.color = "#0ca543"
                    motorControlPanel.onMotorControlDisabled()
                    sensorControlPanel.disableAll()
                }
            }

            onReleased: onAllSensorsReleased()

            Text{
                id: runSensorsText
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                text: "RUN ALL SENSORS"
                color: "#011809"
                font.weight: Font.ExtraBold
                font.pointSize: 32
            }

            background: Rectangle {
                id: runSensorsBacground
                color: "#0ca543"
                radius: runAllSensorsButton.radius
            }
        }

        Rectangle {
            id: runAllSensorsButtonShadow
            z: 99
            anchors.top: runAllSensorsButton.top
            anchors.topMargin: 10
            anchors.left: runAllSensorsButton.left
            anchors.leftMargin: 10

            width: runAllSensorsButton.width
            height: runAllSensorsButton.height
            color: "#011809"
            radius: runAllSensorsButton.radius
        }

        Column {
            id: sensorControlPanelColumn

            width: 240
            height: sensorControlPanelHeader.height + sensorControlPanel.height
            spacing: 0

            anchors.left: parent.left
            anchors.leftMargin: screenMargin
            anchors.bottom: parent.bottom
            anchors.bottomMargin: screenMargin

            Rectangle {
                id: sensorControlPanelHeader
                y: 155
                width: parent.width
                height: 50
                color: "#1c163c"
                radius: 2
                opacity: 1
                border.width: 0

                Text {
                    id: sensorControlPanelText
                    x: 15
                    y: 19
                    color: "#ffffff"
                    text: qsTr("Sensor Control Panel")
                    font.weight: Font.ExtraBold
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    horizontalAlignment: Text.AlignHCenter
                    fontSizeMode: Text.HorizontalFit
                    font.pixelSize: 20
                }
            }

            Rectangle {
                id: sensorControlPanel
                y: 359
                width: parent.width
                height: pulseOxRow.height +
                        temperatureRow.height +
                        forceSwitchRow.height +
                        velocityOfSoundRow.height +
                        impedanceRow.height

                color: "#2d2d57"
                radius: 16
                border.width: 0

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

                Row {
                    id: pulseOxRow

                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.top: parent.top
                    anchors.topMargin: 0

                    Text {
                        id: pulseOxText
                        text: "Pulse Ox"
                        anchors.verticalCenter: parent.verticalCenter
                        horizontalAlignment: Text.AlignRight
                        color: "#ffffff"
                        width: parent.width / 2
                    }

                    SwitchDelegate {
                        id: pulseOxSwitch
                        onToggled: {
                            if (position === 0) {
                                onPulseOxRequestChanged(false)
                            } else {
                                onPulseOxRequestChanged(true)
                            }
                        }
                    }
                }

                Row {
                    id: temperatureRow

                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.top: pulseOxRow.bottom
                    anchors.topMargin: 0

                    Text {
                        id: temperatureSwitchText
                        text: "Temperature"
                        horizontalAlignment: Text.AlignRight
                        anchors.verticalCenter: parent.verticalCenter
                        color: "#ffffff"
                        width: parent.width / 2
                    }

                    SwitchDelegate {
                        id: temperatureSwitch
                        onToggled: {
                            if (position === 0) {
                                onTemperatureRequestChanged(false)
                            } else {
                                onTemperatureRequestChanged(true)
                            }
                        }

                        x: 73
                    }
                }

                Row {
                    id: forceSwitchRow

                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.top: temperatureRow.bottom
                    anchors.topMargin: 0

                    Text {
                        id: forceText

                        text: "Force"
                        horizontalAlignment: Text.AlignRight
                        anchors.verticalCenter: parent.verticalCenter
                        color: "#ffffff"
                        width: parent.width / 2
                    }

                    SwitchDelegate {
                        id: forceSwitch
                        onToggled: {
                            if (position === 0) {
                                onForceRequestChanged(false)
                            } else {
                                onForceRequestChanged(true)
                            }
                        }
                    }
                }

                Row {
                    id: velocityOfSoundRow

                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.top: forceSwitchRow.bottom
                    anchors.topMargin: 0

                    Text {
                        id: velocitySwitchText
                        text: "Velocity of Sound"
                        horizontalAlignment: Text.AlignRight
                        anchors.verticalCenter: parent.verticalCenter
                        color: "#ffffff"
                        width: parent.width / 2
                    }

                    SwitchDelegate {
                        id: velocityOfSoundSwitch
                        onToggled: {
                            if (position === 0) {
                                onVelocityOfSoundRequestChanged(false)
                            } else {
                                onVelocityOfSoundRequestChanged(true)
                            }
                        }
                    }
                }

                Row {
                    id: impedanceRow

                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.top: velocityOfSoundRow.bottom
                    anchors.topMargin: 0

                    Text {
                        id: impedanceSwitchText
                        text: "Impedance"
                        horizontalAlignment: Text.AlignRight
                        anchors.verticalCenter: parent.verticalCenter
                        color: "#ffffff"
                        width: parent.width / 2
                    }

                    SwitchDelegate {
                        id: impedanceSwitch
                        onToggled: {
                            if (position === 0) {
                                onImpedanceRequestChanged(false)
                            } else {
                                onImpedanceRequestChanged(true)
                            }
                        }
                    }
                }
            }
        }

        Column {
            id: motorControlPanelCol

            property bool motorControllerEnabled: false

            width: 240
            height: motorControlPanelHeader.height + motorControlPanel.height
            spacing: 0

            anchors.left: sensorControlPanelColumn.right
            anchors.leftMargin: componentMargin
            anchors.bottom: parent.bottom
            anchors.bottomMargin: screenMargin

            Rectangle {
                id: motorControlPanelHeader

                width: parent.width
                height: 50
                color: "#1c163c"
                radius: 2
                Text {
                    id: motorControllerText
                    x: 15
                    y: 19
                    color: "#ffffff"
                    text: qsTr("Motor Control Panel")
                    font.weight: Font.ExtraBold
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 20
                    anchors.horizontalCenter: parent.horizontalCenter
                    fontSizeMode: Text.HorizontalFit
                    horizontalAlignment: Text.AlignHCenter
                }
            }

            Rectangle {
                id: motorControlPanel
                width: parent.width
                height: sensorControlPanel.height
                color: "#2d2d57"
                radius: 16
                border.width: 0

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
                    x: 37
                    y: 21
                    width: 80
                    height: 40
                    font.family: "Courier"
                    focusPolicy: Qt.NoFocus
                    spacing: 1
                    font.weight: Font.Light
                    enabled: false

                    Text {
                        text: "Release"
                        font.weight: Font.Medium
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        color: "#ffffff"
                        font.family: "Courier"
                        opacity: releaseButton.enabled ? 1.0 : 0.2
                    }

                    background: Rectangle {
                        id: releaseButtonBackground
                        color: "#5a0705"
                        radius: releaseButton.radius
                    }

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
                    x: 122
                    y: 21
                    width: 80
                    height: 40

                    onClicked: motorControlPanel.onMotorControlEnabled()

                    background: Rectangle {
                        id: closeButtonBackground
                        color: "#011809"
                        radius: closeButton.radius
                    }

                    Text {
                        id: closeButtonText
                        text: "Close"
                        font.weight: Font.Medium
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        color: "#ffffff"
                        font.family: "Courier"
                        opacity: closeButton.enabled ? 1.0 : 0.2
                    }
                }

                Text {
                    id: desiredForceText
                    x: 26
                    y: 108
                    color: "#ffffff"
                    text: qsTr("Desired Force, N")
                    font.weight: Font.Bold
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 18
                    opacity: 0.25
                }

                Slider {
                    id: desiredForceSlider
                    x: 26
                    y: 135
                    width: 156
                    height: 40
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
                        x: 57
                        y: 42
                        color: "#ffffff"
                        text: "0"
                        anchors.horizontalCenter: parent.horizontalCenter
                        horizontalAlignment: Text.AlignHCenter
                        font.pixelSize: 20
                        opacity: 0.25
                    }
                }
            }
        }

        Column {
            id: sensorReadingsLeftCol

            anchors.left: motorControlPanelCol.right
            anchors.leftMargin: componentMargin
            anchors.bottom: parent.bottom
            anchors.bottomMargin: screenMargin

            width: 240
            height: actualForceText.height + actualForceDisplay.height +
                    spacer1.height + velocityOfSoundDisplay.height + velocityOfSoundText.height +
                    spacer2.height + impedanceDisplay.height + impedanceText.height + 10 * 7
            spacing: 10

            Text {
                id: actualForceText
                color: "#ffffff"
                text: qsTr("Actual Force, N")
                font.weight: Font.Bold
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
            }

            Rectangle {
                id: actualForceDisplay
                height: 50
                color: "#d0caf9"
                radius: 8
                anchors.horizontalCenter: parent.horizontalCenter
                width: sensorReadingBoxWidth

                Text {
                    id: text13
                    x: 53
                    y: 8
                    text: actualForce
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 20
                }
            }

            Rectangle {
                id: spacer1
                height: 40
                color: "#ffffff"
                radius: 0
                opacity: 0
                anchors.leftMargin: 0
                anchors.right: parent.right
                anchors.left: parent.left
                border.width: 0
                anchors.rightMargin: 0
            }

            Text {
                id: velocityOfSoundText
                color: "#ffffff"
                text: qsTr("Velocity of Sound, m/sec")
                font.weight: Font.Bold
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
            }

            Rectangle {
                id: velocityOfSoundDisplay
                height: 50
                color: "#d0caf9"
                radius: 8
                anchors.horizontalCenter: parent.horizontalCenter
                width: sensorReadingBoxWidth

                Text {
                    id: text14
                    x: -2
                    y: 143
                    text: velocityOfSound
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 20
                }
            }

            Rectangle {
                id: spacer2
                height: 40
                color: "#ffffff"
                radius: 0
                opacity: 0
                anchors.leftMargin: 0
                anchors.left: parent.left
                anchors.right: parent.right
                border.width: 0
                anchors.rightMargin: 0
            }

            Text {
                id: impedanceText
                color: "#ffffff"
                text: qsTr("Impedance,\nmagnitude/frequency")
                horizontalAlignment: Text.AlignCenter
                font.weight: Font.Bold
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
            }

            Rectangle {
                id: impedanceDisplay
                height: 50
                color: "#d0caf9"
                radius: 8
                anchors.horizontalCenter: parent.horizontalCenter
                width: sensorReadingBoxWidth

                Text {
                    id: text15
                    x: -4
                    y: 51
                    text: impedance
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 20
                }
            }
        }

        Column {
            id: sensorReadingsRightCol

            width: 240
            height: temperatureText.height + temperatureDisplay.height +
                    spacer3.height + oxygenText.height + oxygenDisplay.height +
                    10 * 6

            anchors.left: sensorReadingsLeftCol.right
            anchors.leftMargin: componentMargin
            anchors.top: sensorReadingsLeftCol.top
            anchors.topMargin: 0

            spacing: 10

            Text {
                id: temperatureText
                color: "#ffffff"
                text: qsTr("Temperature, C")
                font.weight: Font.Bold
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
            }

            Rectangle {
                id: temperatureDisplay
                height: 50
                color: "#d0caf9"
                radius: 8
                anchors.horizontalCenter: parent.horizontalCenter
                width: sensorReadingBoxWidth

                Text {
                    id: text16
                    x: -169
                    y: 191
                    text: temperature
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 20
                }
            }

            Rectangle {
                id: spacer3
                height: 40
                color: "#ffffff"
                radius: 0
                opacity: 0
                anchors.leftMargin: 0
                anchors.left: parent.left
                anchors.right: parent.right
                border.width: 0
                anchors.rightMargin: 0
            }

            Text {
                id: oxygenText
                color: "#ffffff"
                text: qsTr("Oxygen Level, %")
                font.weight: Font.Bold
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
            }

            Rectangle {
                id: oxygenDisplay
                height: 50
                color: "#d0caf9"
                radius: 8
                anchors.horizontalCenter: parent.horizontalCenter
                width: sensorReadingBoxWidth

                Text {
                    id: text17
                    x: 109
                    y: -76
                    text: oxygen
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 20
                }
            }
        }

        Rectangle {
            id: cameraPanelTextBox

            anchors.horizontalCenter: cameraPanel.horizontalCenter
            anchors.top: parent.top
            anchors.topMargin: screenMargin

            Text {
                id: cameraPanelText

                anchors.centerIn: parent
                width: 104
                height: 32
                color: "#ffffff"
                text: qsTr("Camera")
                font.weight: Font.ExtraBold
                anchors.bottom: cameraPanel.top
                anchors.bottomMargin: 20
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 20
            }
        }

        Rectangle {
            id: cameraPanel

            anchors.left: parent.left
            anchors.leftMargin: screenMargin
            anchors.top: cameraPanelTextBox.top
            anchors.topMargin: screenMargin

            width: (parent.width - screenMargin) / 2 - screenMargin
            height: 300
            color: "#ffffff"
            radius: 16
            anchors.rightMargin: 63
            border.width: 0
        }

        Rectangle {
            id: pulsePanelTextBox
            anchors.horizontalCenter: pulsePanel.horizontalCenter
            anchors.top: parent.top
            anchors.topMargin: screenMargin

            Text {
                id: pulsePanelText

                anchors.centerIn: parent
                width: 104
                height: 32
                color: "#ffffff"
                text: qsTr("Pulse")
                font.weight: Font.ExtraBold
                anchors.bottom: cameraPanel.top
                anchors.bottomMargin: 20
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 20
            }
        }

        Rectangle {
            id: pulsePanel

            anchors.right: parent.right
            anchors.rightMargin: screenMargin
            anchors.top: pulsePanelTextBox.top
            anchors.topMargin: screenMargin

            width: (parent.width - screenMargin) / 2 - screenMargin
            height: 300
            color: "#ffffff"
            radius: 16
            border.width: 0

            CustomPlotItem {
                id: pulseoxPlot
                timeToDisplay: 5000
                xAxisLabel: "MCU time (ms)"
                yAxisLabel: "Oxygen level (SpO2%)"
                anchors.fill: parent
                objectName: "pulsePlot"

                Component.onCompleted: initCustomPlot()
            }
        }
    }
}

import QtQuick 2.9
import QtQuick.Controls 1.4
import CustomPlot 1.0
import ImageDisplayer 1.0
import "qrc:/qml"

Rectangle {
    color: Constants.background_color

    property int curr_state: 0
    property int prev_state: 0
    states: [
        State {
            name: "hidden"
            when: (curr_state === -1)
            StateChangeScript {
                name: "hidden"
                script: {
                    visible = false
                    measurementPanel.turnOffAllMeasurements()
                    measurementPanel.enableMeasurementButtons()
                    motorControlPanel.enableMotorControlButtons()
                    motorControlPanel.turnOffMotorController()
                    runAllSensorsButton.unsetButtonToggled()
                    prev_state = curr_state
                }
            }
        },
        State {
            name: "no sensors running"
            when: (curr_state === 0)
            StateChangeScript {
                name: "no sensors running"
                script: {
                    if (prev_state === -1) {
                        visible = true
                    }
                    measurementPanel.turnOffAllMeasurements()
                    measurementPanel.disableMeasurementButtons()
                    motorControlPanel.enableMotorControlButtons()
                    motorControlPanel.turnOffMotorController()
                    runAllSensorsButton.unsetButtonToggled()
                    runAllSensorsButton.enable()
                    prev_state = curr_state
                }
            }
        },
        State {
            name: "run all sensors pressed"
            when: (curr_state === 1)
            StateChangeScript {
                name:  "run all sensors pressed"
                script: {
                    measurementPanel.turnOnAllMeasurements()
                    measurementPanel.disableMeasurementButtons()
                    motorControlPanel.turnOnMotorController()
                    motorControlPanel.disableMotorControlButtons()
                    prev_state = curr_state
                }
            }
        },
        State {
            name: "error"
            when: (curr_state === 2)
            StateChangeScript {
                name: "error"
                script: {
                    prev_state = curr_state
                    measurementPanel.disableMeasurementButtons()
                    motorControlPanel.disableMotorControlButtons()
                    runAllSensorsButton.unsetButtonToggled()
                    runAllSensorsButton.disable()
                }
            }
        },
        State {
            name: "motor controller running"
            when: (curr_state === 3)
            StateChangeScript {
                name: "motor controller running"
                script: {
                    prev_state = curr_state
                    measurementPanel.enableMeasurementButtons()
                }
            }
        }
    ]

    MeasurementPanel {
        id: measurementPanel
        objectName: "measurementPanel"
        anchors.left: parent.left
        anchors.leftMargin: Constants.screen_margin
        anchors.bottom: parent.bottom
        anchors.bottomMargin: Constants.screen_margin
    }

    MotorControlPanel {
        id: motorControlPanel

        onMotorControllerTurnedOn: {
            measurementPanel.forceSwitchOn()
        }

        anchors.left: measurementPanel.right
        anchors.leftMargin: Constants.component_margin
        anchors.bottom: parent.bottom
        anchors.bottomMargin: Constants.screen_margin
        height: measurementPanel.height
    }

    ExclusiveGroup {
        id: graphDisplayButtonGroup
    }

    Column {
        id: sensorReadingsLeftCol
        objectName: "sensorReadingsLeftCol"
        anchors.left: motorControlPanel.right
        anchors.leftMargin: Constants.screen_margin
        anchors.bottom: parent.bottom
        anchors.bottomMargin: Constants.screen_margin
        width: Constants.default_measurement_panel_wdith
        spacing: Constants.component_margin

        SensorReadingV2 {
            objectName: "actualForce"
            anchors.horizontalCenter: parent.horizontalCenter
            sensor_heading: "Actual Force, N"
            sensor_reading: "0"
            exclusiveGroup: graphDisplayButtonGroup
        }

        SensorReadingV2 {
            objectName: "velOfSound"
            anchors.horizontalCenter: parent.horizontalCenter
            sensor_heading: "Velocity of Sound,\nm/sec"
            sensor_reading: "0"
            exclusiveGroup: graphDisplayButtonGroup
        }

        SensorReadingV2 {
            objectName: "impedance"
            anchors.horizontalCenter: parent.horizontalCenter
            sensor_heading: "Impedance,\nmagnitude/frequency"
            sensor_reading: "0"
            exclusiveGroup: graphDisplayButtonGroup
        }
        Rectangle {
            id: spacer
            height: 10
            width: parent.width
            opacity: 0
        }
    }

    Column {
        id: sensorReadingsRightCol
        objectName: "sensorReadingsRightCol"
        width: Constants.default_measurement_panel_wdith
        anchors.left: sensorReadingsLeftCol.right
        anchors.leftMargin: Constants.component_margin
        anchors.top: sensorReadingsLeftCol.top
        anchors.topMargin: 0
        spacing: Constants.component_margin

        SensorReadingV2 {
            objectName: "jawPosition"
            anchors.horizontalCenter: parent.horizontalCenter
            sensor_heading: "Jaw Position, mm"
            sensor_reading: "0"
            exclusiveGroup: graphDisplayButtonGroup
        }

        SensorReadingV2 {
            objectName: "temperature"
            anchors.horizontalCenter: parent.horizontalCenter
            sensor_heading: "Temperature, C"
            sensor_reading: "0"
            exclusiveGroup: graphDisplayButtonGroup
        }

        SensorReadingV2 {
            checked: true
            objectName: "oxygen"
            anchors.horizontalCenter: parent.horizontalCenter
            sensor_heading: "Oxygen Level, %"
            sensor_reading: "0"
            exclusiveGroup: graphDisplayButtonGroup
        }
    }

    ColoredButton {
        id: runAllSensorsButton
        z: 100

        anchors.bottom: measurementPanel.top
        anchors.bottomMargin: Constants.component_margin
        anchors.left: parent.left
        anchors.leftMargin: Constants.screen_margin

        width: measurementPanel.width +
               Constants.component_margin +
               motorControlPanel.width -
               runAllSensorsButtonShadow.shadow_offset
        height: sensorReadingsLeftCol.height - measurementPanel.height
        button_corner_radius: height / 2

        primary_color: Constants.light_green
        selected_color: Constants.light_red

        button_text: "RUN ALL SENSORS"
        text_size: Constants.medium_text_size
        text_bold: true
        primary_text_color: Constants.dark_green
        selected_text_color: Constants.dark_red

        onToggledChanged: {
            if (toggled) {
                curr_state = 1
                runAllSensorsButtonShadow.color = Constants.dark_red
            } else {
                if (curr_state === 1) curr_state = 0
                runAllSensorsButtonShadow.color = Constants.dark_green
            }
        }
    }

    Rectangle {
        id: runAllSensorsButtonShadow
        z: 99
        property int shadow_offset: 10
        anchors.top: runAllSensorsButton.top
        anchors.topMargin: shadow_offset
        anchors.left: runAllSensorsButton.left
        anchors.leftMargin: shadow_offset

        width: runAllSensorsButton.width
        height: runAllSensorsButton.height
        color: Constants.dark_green
        radius: parent.height / 2
    }

    Rectangle {
        id: cameraPanelTextBox
        anchors.horizontalCenter: cameraPanel.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: Constants.screen_margin

        Text {
            anchors.centerIn: parent
            color: Constants.primary_font_color
            text: qsTr("Camera")
            font.weight: Font.ExtraBold
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: Constants.small_text_size
        }
    }

    Rectangle {
        id: cameraPanel

        anchors.left: parent.left
        anchors.leftMargin: Constants.screen_margin
        anchors.top: cameraPanelTextBox.top
        anchors.topMargin: Constants.screen_margin

        width: (parent.width - Constants.screen_margin) / 2 - Constants.screen_margin
        height: 300
        color: Constants.primary_font_color
        anchors.rightMargin: 63
        border.width: 0

        ImageDisplayer {
            id: cameraDisplay
            objectName: "cameraDisplay"
            anchors.fill: parent
            Component.onCompleted: initImageDisplayer()
        }
    }

    Rectangle {
        id: pulsePanelTextBox
        anchors.horizontalCenter: pulsePanel.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: Constants.screen_margin

        Text {
            anchors.centerIn: parent
            color: Constants.primary_font_color
            text: "Pulse"
            font.weight: Font.ExtraBold
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 20
        }
    }

    Rectangle {
        id: pulsePanel

        anchors.right: parent.right
        anchors.rightMargin: Constants.screen_margin
        anchors.top: pulsePanelTextBox.top
        anchors.topMargin: Constants.screen_margin

        width: (parent.width - Constants.screen_margin) / 2 - Constants.screen_margin
        height: 300
        color: Constants.primary_font_color

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

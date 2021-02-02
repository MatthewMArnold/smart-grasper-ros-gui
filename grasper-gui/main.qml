import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import Ros 1.0
import CustomPlot 1.0
import ImageDisplayer 1.0
import ValueUpdater 1.0
import "qrc:/qml"

ApplicationWindow {
    id: window
    title: "grasper-gui"
    visible: true

    // Set the screen to not be resizable
    maximumHeight: Constants.total_screen_height
    minimumHeight: Constants.total_screen_height
    maximumWidth: Constants.total_screen_width
    minimumWidth: Constants.total_screen_width

    // motor control panel signals
    signal onDesiredForceChanged(double force)
    signal onMotorClosedRequested(bool closeMotor)

    // sensor control panel signals
    signal onPulseOxRequestChanged(bool pulseOxRequested)
    signal onTemperatureRequestChanged(bool temperatureRequested)
    signal onForceRequestChanged(bool forceRequested)
    signal onVelocityOfSoundRequestChanged(bool velocityOfSoundRequested)
    signal onImpedanceRequestChanged(bool impedanceRequested)

    Sidebar {
        id: sidebar
        anchors.top: parent.top
        anchors.right: parent.right
        onEnterMainScreen: {
            homeScreen.curr_state = 0
            sandboxScreen.state = 0
            sandboxScreen.visible = true
        }
        onEnterSandboxScreen: {
            homeScreen.curr_state = -1
            sandboxScreen.visible = true
        }
    }

    NoncriticalErrorBanner {
        id: noncriticalErrorBanner
        objectName: "noncriticalErrorBanner"

        function triggerNonurgentErrorDialogue(msg) {
            visible = true
            text = msg
        }

        function hideNonurgentErrorDialogue() {
            visible = false
        }
    }

    CriticalErrorBanner {
        id: criticalErrorBanner
        objectName: "criticalErrorBanner"
        function triggerCriticalErrorDialogue(msg, dismissable) {
            visible = true
            text = msg
            ok_button_visible = dismissable
            if (homeScreen.curr_state !== -1) homeScreen.curr_state = 2
        }
        function hideCriticalErrorDialogue() {
            visible = false
            if (homeScreen.curr_state !== -1) homeScreen.curr_state = 0
        }
        signal criticalErrorOKPressed()
    }

    Rectangle {
        id: sandboxScreen
        width: Constants.main_screen_width
        height: parent.height
//        SensorReadingV2 {
//            id: test
//            anchors.left:parent.left
//            anchors.top:parent.top
//            width: 240
//            sensor_heading: "pulse ox"
//        }

        RadioButton {
            text: "Radio Button"
            style: RadioButtonStyle {
                indicator: Rectangle {
                        implicitWidth: 100
                        implicitHeight: 40
                        border.color: control.activeFocus ? "pink" : "green"
                        border.width: 1
                        Rectangle {
                            anchors.fill: parent
                            visible: control.checked
                            color: "#555"
                            radius: 9
                            anchors.margins: 4
                        }
                }
            }
         }
    }

    HomeScreen {
        id: homeScreen
        objectName: "homeScreen"
        height: Constants.main_screen_height
        width: Constants.main_screen_width
        anchors.left: parent.left
        anchors.top: parent.top
    }
}

import QtQuick 2.9
import QtQuick.Controls 2.2
import Ros 1.0
import CustomPlot 1.0
import ImageDisplayer 1.0
import "qrc:/qml"

ApplicationWindow {
    property int sensorReadingBoxWidth: 100

    property double actualForce: 0
    property double temperature: 0
    property double velocityOfSound: 0
    property double oxygen: 0
    property double impedance: 0

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

    function triggerCriticalErrorDialogue(msg, dismissable) {
        console.log("triggerCriticalErrorDialogue")
        criticalErrorIndicator.visible = true
        criticalErrorMessage.text = msg
        dismissErrorButton.visible = dismissable
    }

    function hideCriticalErrorDialogue() {
        console.log("hideCriticalErrorDialogue")
        criticalErrorIndicator.visible = false
        criticalErrorMessage.text = "none"
    }

    signal criticalErrorOKPressed()

    function triggerNonurgentErrorDialogue(msg) {
        noncriticalErrorIndicator.visible = true
        noncriticalErrorText.text = msg
    }

    function hideNonurgentErrorDialogue() {
        noncriticalErrorIndicator.visible = false
    }

    Sidebar {
        id: sidebar
        anchors.top: parent.top
        anchors.right: parent.right
    }

    NoncriticalErrorBanner {
        id: noncriticalErrorBanner
    }

    CriticalErrorBanner {
        id: criticalErrorBanner
    }

    Rectangle {
        id: sandboxScreen
        width: Constants.main_screen_width
        height: parent.height
    }

    HomeScreen {
        id: homeScreen
        height: Constants.main_screen_height
        width: Constants.main_screen_width
        anchors.left: parent.left
        anchors.top: parent.top
    }
}

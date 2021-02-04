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

    /*
     * This is for me to mess around with new stuff in.
     */
    Rectangle {
        id: sandboxScreen
        width: Constants.main_screen_width
        height: parent.height
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

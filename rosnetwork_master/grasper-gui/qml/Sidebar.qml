import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Rectangle {
    width: Constants.side_bar_width
    height: Constants.main_screen_height
    color: Constants.background_color

    property string curr_state: "mainButton"

    /**
     * define states and state transitions for the different screens.
     */
    states: [
        State {
            name: "mainButton"
            when: (curr_state == "mainButton")
            StateChangeScript {
                name: "mainButtonScript"
                script: sandboxButton.unsetButtonToggled()
            }
        },
        State {
            name: "sandboxButton"
            when: (curr_state == "sandboxButton")
            StateChangeScript {
                name:  "sandboxButtonScript"
                script: {
                    mainButton.unsetButtonToggled()
                }
            }
        }
    ]

    Rectangle {
        color: Constants.subsection_background_color
        radius: Constants.default_large_edge_radius
        anchors.topMargin: Constants.screen_margin
        anchors.bottomMargin: Constants.screen_margin
        anchors.rightMargin: Constants.screen_margin
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom

        Column {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.topMargin: Constants.component_margin
            anchors.leftMargin: Constants.component_margin
            anchors.rightMargin: Constants.component_margin
            spacing: Constants.component_margin

            ColoredButton {
                id: mainButton
                // Start in the main page state
                Component.onCompleted: setButtonToggled()
                // When toggled, change state
                onToggledChanged: {
                    if (toggled) {
                        curr_state = "mainButton"
                    }
                }
                height: Constants.side_bar_button_height
                width: parent.width
                button_corner_radius: Constants.default_large_edge_radius
                anchors.horizontalCenter: parent.horizontalCenter
                primary_color: Constants.dark_purple
                selected_color: Constants.light_purple
                selected_text_color: Constants.dark_purple
                text_size: Constants.medium_text_size
                primary_text_color: Constants.light_purple
                button_text: "main"
                user_untoggable: false
            }

            ColoredButton {
                id: sandboxButton
                onToggledChanged: {
                    if (toggled) {
                        curr_state = "sandboxButton"
                    }
                }
                height: Constants.side_bar_button_height
                width: parent.width
                anchors.horizontalCenter: parent.horizontalCenter
                button_corner_radius: Constants.default_large_edge_radius
                primary_color: Constants.dark_purple
                selected_color: Constants.light_purple
                selected_text_color: Constants.dark_purple
                text_size: Constants.medium_text_size
                primary_text_color: Constants.light_purple
                button_text: "sandbox"
                user_untoggable: false
            }
        }
    }
}

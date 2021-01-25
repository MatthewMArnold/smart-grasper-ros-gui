import QtQuick 2.9
import QtQuick.Controls 2.2

import "qrc:/qml"

Button {
    /**
     * User settable properties.
     * primary_color: The button's color when not toggled.
     * selected_color: The button's color when toggled.
     * primary_text_color: The color of the button's text when not toggled.
     * selected_text_color: The color of the button's text when toggled.
     * button_text: The text to be displayed. Use this instead of the Button's "text".
     * text_size: The height of the text.
     * user_untoggable: If true, the user can press on the toggled button to untoggle
     * it, if false, they cannot.
     * button_corner_radius: The radius of the corners of the button.
     */
    property color primary_color: "white"
    property color selected_color: primary_color
    property color primary_text_color: "black"
    property color selected_text_color: primary_text_color
    property string button_text: ""
    property int text_size: Constants.small_text_size
    property bool user_untoggable: false
    property int button_corner_radius: 0

    property bool toggled: false

    function onToggledChanged() {
        console.log("toggled changed for button " + button_text + ", " + toggled)
        if (toggled) {
            background.color = selected_color
            colored_text.color = selected_text_color
        } else {
            background.color = primary_color
            colored_text.color = primary_text_color
        }
    }

    function toggleButton() {
        toggled = !toggled
        onToggledChanged()
    }

    function setButtonToggled() {
        if (!toggled) {
            toggleButton()
        }
    }

    function unsetButtonToggled() {
        console.log("call unsettoggle for name " + button_text)
        if (toggled) {
            toggleButton()
        }
    }

    onReleased: {
        if (!toggled || user_untoggable) {
            toggleButton()
        }
    }

    Text {
        id: colored_text
        anchors.centerIn: parent
        text: button_text
        color: primary_text_color
        font.pixelSize: text_size
    }

    background: Rectangle {
        id: background
        color: primary_color
        radius: button_corner_radius
    }
}

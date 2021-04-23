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
    property alias button_text: colored_text.text
    property alias text_size: colored_text.font.pixelSize
    property bool user_untoggable: true
    property alias button_corner_radius: background.radius
    property alias text_bold: colored_text.font.bold

    property bool toggled: false

    function onToggledChanged() {
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
        if (toggled) {
            toggleButton()
        }
    }

    function disable() {
        colored_text.opacity = Constants.default_disabled_opacity
        enabled = false
    }

    function enable() {
        colored_text.opacity = 1.0
        enabled = true
    }

    onReleased: {
        if (!toggled || user_untoggable) {
            toggleButton()
        }
    }

    Text {
        id: colored_text
        text: ""
        anchors.centerIn: parent
        color: primary_text_color
        font.bold: false
    }

    background: Rectangle {
        id: background
        color: primary_color
        radius: 0
    }
}

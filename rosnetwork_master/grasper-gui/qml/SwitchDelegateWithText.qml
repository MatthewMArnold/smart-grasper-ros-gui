import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Row {
    property string side_text: ""
    
    signal sliderToggled(bool toggled, int index)

    function switchOff() {
        if (s.position !== 0) {
            s.toggle()
            sliderToggled(false, i.currentIndex)
        }
    }

    function switchOn() {
        if (s.position === 0) {
            s.toggle()
            sliderToggled(true, i.currentIndex)
        }
    }

    function disable() {
        t.opacity = Constants.default_disabled_opacity
        s.enabled = false
    }

    function enable() {
        t.opacity = 1.0
        s.enabled = true
    }

    Text {
        id: t
        text: side_text
        anchors.verticalCenter: parent.verticalCenter
        horizontalAlignment: Text.AlignRight
        color: Constants.primary_font_color
        width: parent.width / 2
    }

    SwitchDelegate {
        id: s
        anchors.verticalCenter: parent.verticalCenter
        onToggled: {
            sliderToggled(position !== 0, i.currentIndex)
        }
    }

    ComboBox {
        id: i
        width: 60
        anchors.verticalCenter: parent.verticalCenter
        model: ["0", "1", "2", "3", "4"]
        onCurrentIndexChanged: {
            if (s.position !== 0) {
                sliderToggled(true, currentIndex)
            }
        }
    }
}

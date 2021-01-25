import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Row {
    property string side_text: ""
    
    signal sliderToggled(bool toggled)

    Text {
        text: side_text
        anchors.verticalCenter: parent.verticalCenter
        horizontalAlignment: Text.AlignRight
        color: Constants.primary_font_color
        width: parent.width / 2
    }

    SwitchDelegate {
        onToggled: sliderToggled(position !== 0)
    }
}

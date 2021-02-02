import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Rectangle {
    z: 999
    visible: false
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    height: Constants.noncritical_error_banner_height
    color: Constants.noncrit_warn_yellow

    MouseArea {
        anchors.fill: parent
        propagateComposedEvents: false
    }

    Text {
        id: noncriticalErrorText
        horizontalAlignment: Text.AlignHCenter
        text: "None"
        anchors.centerIn: parent
        font.pointSize: Constants.small_text_size
    }
}

import QtQuick 2.9
import QtQuick.Controls 2.2
import "qrc:/qml"

Rectangle {
    z: 1000
    visible: false
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: parent.top
    anchors.topMargin: 100
    height: 200
    color: "yellow"

    onVisibleChanged: {
        if (visible) {
            mainScreen.disableAll()
        }
    }

    Row {
        height: 200
        width: parent.width
        spacing: 20
        Image {
            source: "images/warning.png"
            width: height
            height: parent.height
        }

        Text {
            id: criticalErrorMessage
            // This is an example message
            text: "none"
            font.pixelSize: 36
            anchors.verticalCenter: parent.verticalCenter
        }

        Button {
            id: dismissErrorButton
            width: parent.width / 5
            height: (parent.height) / 2
            onReleased: criticalErrorOKPressed()
            Text {
                id: dismissErrorButtonText
                text: "OK"
                anchors.centerIn: parent
                font.pixelSize: 48
            }
            anchors.verticalCenter: parent.verticalCenter

            background: Rectangle {
                color: "white"
            }
        }
    }
}

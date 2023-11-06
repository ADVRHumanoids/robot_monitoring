import QtQuick
import Qt3D.Render

Item {

    property Camera camera
    property real fovMax: 130
    property real fovMin: 10
    property real panTiltSensitivity: 5
    property real pinchSensitivity: 0.5
    property real wheelSensitivity: 0.001

    id: root

    // handle pinch-to-zoom
    PinchArea {

        id: pinchArea

        anchors.fill: parent

        function updateFov(newFov) {
            if (newFov > fovMax) {
                newFov = fovMax
            } else if (newFov < fovMin) {
                newFov = fovMin
            }
            var eps = 1.0 / 60.0
            if (Math.abs(camera.fieldOfView - newFov) > eps) {
                camera.fieldOfView = newFov
            }
        }

        function updatePinch(pinch) {
            updateFov(camera.fieldOfView * (1.0 + (pinch.previousScale - pinch.scale) * pinchSensitivity))
        }

        // focal length at the pinch beginning
        property real initialF

        onPinchStarted: (pinch) => {
                            initialF = 1./Math.tan(camera.fieldOfView/180*Math.PI/2.)
                        }

        onPinchUpdated: (pinch) => {
                            let f = pinch.scale * initialF
                            let fov = 2*Math.atan(1/f)/Math.PI*180
                            updateFov(fov)
                        }

        onPinchFinished: (pinch) => {
                             let f = pinch.scale * initialF
                             let fov = 2*Math.atan(1/f)/Math.PI*180
                             updateFov(fov)
                         }

        // handle pan-tilt
        MouseArea {

            anchors.fill: parent

            propagateComposedEvents: true
            preventStealing: true

            property point startPoint

            function updateView(mouse) {
                camera.panAboutViewCenter((startPoint.x - mouse.x) * panTiltSensitivity * camera.fieldOfView / width, Qt.vector3d(0.0, 0.0, 1.0))
                camera.tiltAboutViewCenter((mouse.y - startPoint.y) * panTiltSensitivity * camera.fieldOfView / height)
                startPoint = Qt.point(mouse.x, mouse.y)
            }

            onPressed: (mouse) => {
                           startPoint = Qt.point(mouse.x, mouse.y)
                       }

            onPositionChanged: (mouse) => {
                                   updateView(mouse)
                               }

            onReleased: (mouse) => {
                            updateView(mouse)
                        }

            onWheel: (event) => {
                         event.accepted = true
                         let zoomFactor = Math.exp(event.angleDelta.y * wheelSensitivity)
                         let initialF = 1./Math.tan(camera.fieldOfView/180*Math.PI/2.)
                         let f = zoomFactor*initialF
                         let fov = 2*Math.atan(1/f)/Math.PI*180
                         pinchArea.updateFov(fov)
                     }
        }
    }
}

import QtQuick
import QtQuick.Controls
import QtCharts

import ".."
import xbot2_gui.RobotModel

Item {

    property ClientEndpoint client

    RobotModel {

        id: model

        onJointNamesChanged: {
            console.log(jointNames)
        }

    }

    Component.onCompleted: {

        client.doRequestAsync('GET', '/joint_states/urdf', '')
            .then((response) =>
                  {
                      model.setUrdf(response.urdf, false)
                      let q = Array(model.ndof).fill(0)
                      model.setJointPosition(q)
                      let pose = model.getPose('wheel_A')
                      console.log(`${pose.translation} ${pose.rotation}`)
                  })
            .catch((err) => { console.error(err) } )

    }

}

//Item {

//    ChartView {
//           id: chart
//           anchors.fill: parent
//           theme: ChartView.ChartThemeBrownSand
//           antialiasing: true

//           LineSeries {
//               name: "LineSeries"
//               XYPoint { x: 0; y: 0 }
//               XYPoint { x: 1.1; y: 2.1 }
//               XYPoint { x: 1.9; y: 3.3 }
//               XYPoint { x: 2.1; y: 2.1 }
//               XYPoint { x: 2.9; y: 4.9 }
//               XYPoint { x: 3.4; y: 3.0 }
//               XYPoint { x: 4.1; y: 3.3 }
//           }
//           PinchArea{
//               id: pa
//               anchors.fill: parent
//               property real currentPinchScaleX: 1
//               property real currentPinchScaleY: 1
//               property real pinchStartX : 0
//               property real pinchStartY : 0

//               onPinchStarted: {
//                   // Pinching has started. Record the initial center of the pinch
//                   // so relative motions can be reversed in the pinchUpdated signal
//                   // handler
//                   pinchStartX = pinch.center.x;
//                   pinchStartY = pinch.center.y;
//               }

//               onPinchUpdated: {
//                   chart.zoomReset();

//                   // Reverse pinch center motion direction
//                   var center_x = pinchStartX + (pinchStartX - pinch.center.x);
//                   var center_y = pinchStartY + (pinchStartY - pinch.center.y);

//                   // Compound pinch.scale with prior pinch scale level and apply
//                   // scale in the absolute direction of the pinch gesture
//                   var scaleX = currentPinchScaleX * (1 + (pinch.scale - 1) * Math.abs(Math.cos(pinch.angle * Math.PI / 180)));
//                   var scaleY = currentPinchScaleY * (1 + (pinch.scale - 1) * Math.abs(Math.sin(pinch.angle * Math.PI / 180)));

//                   // Apply scale to zoom levels according to pinch angle
//                   var width_zoom = height / scaleX;
//                   var height_zoom = width / scaleY;

//                   var r = Qt.rect(center_x - width_zoom / 2, center_y - height_zoom / 2, width_zoom, height_zoom);
//                   chart.zoomIn(r);
//               }

//               onPinchFinished: {
//                   // Pinch finished. Record compounded pinch scale.
//                   currentPinchScaleX = currentPinchScaleX * (1 + (pinch.scale - 1) * Math.abs(Math.cos(pinch.angle * Math.PI / 180)));
//                   currentPinchScaleY = currentPinchScaleY * (1 + (pinch.scale - 1) * Math.abs(Math.sin(pinch.angle * Math.PI / 180)));
//               }

//               MouseArea{
//                   anchors.fill: parent
//                   drag.target: dragTarget
//                   drag.axis: Drag.XAndYAxis

//                   onDoubleClicked: {
//                       chart.zoomReset();
//                       parent.currentPinchScaleX = 1;
//                       parent.currentPinchScaleY = 1;
//                   }
//               }

//               Item {
//                   // A virtual item to receive drag signals from the MouseArea.
//                   // When x or y properties are changed by the MouseArea's
//                   // drag signals, the ChartView is scrolled accordingly.
//                   id: dragTarget

//                   property real oldX : x
//                   property real oldY : y

//                   onXChanged: {
//                       chart.scrollLeft( x - oldX );
//                       oldX = x;
//                   }
//                   onYChanged: {
//                       chart.scrollUp( y - oldY );
//                       oldY = y;
//                   }
//                }
//            }
//        }

//}

import QtQuick 2.0

Item {

    id: client

    signal jointStateReceived(var msg)
    signal auxReceived(var msg)
    signal faultReceived(var msg)

    Timer {
        interval: 1000
        repeat: true
        onTriggered: {

            // fake msg
            var jsmsg = {
                type: 'joint_states',
                names: ['ankle_pitch_1', 'ankle_pitch_2', 'ankle_pitch_3', 'ankle_pitch_4', 'ankle_yaw_1', 'ankle_yaw_2,
                    ankle_yaw_3', 'ankle_yaw_4', 'd435_head_joint', 'hip_pitch_1', 'hip_pitch_2', 'hip_pitch_3,
                    hip_pitch_4', 'hip_yaw_1', 'hip_yaw_2', 'hip_yaw_3', 'hip_yaw_4', 'j_arm1_1', 'j_arm1_2', 'j_arm1_3,
                    j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7', 'j_arm2_1', 'j_arm2_2', 'j_arm2_3', 'j_arm2_4,
                    j_arm2_5', 'j_arm2_6', 'j_arm2_7', 'j_wheel_1', 'j_wheel_2', 'j_wheel_3', 'j_wheel_4', 'knee_pitch_1,
                    knee_pitch_2', 'knee_pitch_3', 'knee_pitch_4', 'torso_yaw', 'velodyne_joint']

            }

            jointStateReceived(msg)
        }
    }
}

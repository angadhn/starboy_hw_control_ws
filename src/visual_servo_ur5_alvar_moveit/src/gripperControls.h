#include <robotiq_c_model_control/CModel_robot_output.h>
ros::Publisher *gripperPubPtr;

void initializeGripperMsg()// Function for gripper controller
{
    robotiq_c_model_control::CModel_robot_output closeGripperMsg;
    closeGripperMsg.rACT = 0;
    closeGripperMsg.rGTO = 0;
    closeGripperMsg.rATR = 0;
    closeGripperMsg.rPR = 0;
    closeGripperMsg.rSP = 0;
    closeGripperMsg.rFR = 0;
    gripperPubPtr->publish(closeGripperMsg);

    closeGripperMsg.rACT = 1;
    closeGripperMsg.rGTO = 1;
    closeGripperMsg.rSP = 255;
    closeGripperMsg.rFR = 150;
    gripperPubPtr->publish(closeGripperMsg);

}

void sendGripperMsg(int position, int speed=100, int force=150)// Function for gripper controller
{
    robotiq_c_model_control::CModel_robot_output closeGripperMsg;
    closeGripperMsg.rACT = 1;
    closeGripperMsg.rGTO = 1;
    closeGripperMsg.rATR = 0;
    closeGripperMsg.rPR = max(0, min(255, position));
    closeGripperMsg.rSP = speed;
    closeGripperMsg.rFR = force;
    gripperPubPtr->publish(closeGripperMsg);
}

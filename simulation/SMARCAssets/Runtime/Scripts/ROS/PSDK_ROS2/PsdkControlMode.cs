using RosMessageTypes.PsdkInterfaces;
using dji;
using ROS.Core;
using UnityEngine;



namespace M350.PSDK_ROS2
{
    [AddComponentMenu("Smarc/PSDK_ROS/PsdkControlMode")]
    public class PsdkControlMode : ROSPublisher<ControlModeMsg>
    {
        DJIController controller;
        protected override void InitPublisher()
        {
            controller = GetComponentInParent<DJIController>();
            base.InitPublisher();
        }

        protected override void UpdateMessage()
        {
            if (controller.GotControl) ROSMsg.control_auth = 1;
            else ROSMsg.control_auth = 0;
            ROSMsg.device_mode = 4;
        }
    }
}
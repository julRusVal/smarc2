using UnityEngine;

using RosMessageTypes.Geometry;
using GimbalAct = VehicleComponents.Actuators.Gimbal;

namespace ROS.Subscribers
{

    [RequireComponent(typeof(GimbalAct))]
    public class GimbalCommand_Sub : Actuator_Sub<Vector3Msg>
    {
        GimbalAct gimbal;

        void Awake()
        {
            gimbal = GetComponent<GimbalAct>();
        }

        protected override void UpdateVehicle(bool reset)
        {
            if(gimbal == null)
            {
                Debug.Log("No Gimbal found! Disabling.");
                enabled = false;
                rosCon.Unsubscribe(topic);
                return;
            }
            if (reset)
            {
                gimbal.roll = 0;
                gimbal.pitch = 0;
                gimbal.yaw = 0;
                return;
            }
            gimbal.roll = (float)ROSMsg.x;
            gimbal.pitch = (float)ROSMsg.y;
            gimbal.yaw = (float)ROSMsg.z;
        }
    }
}

using UnityEngine;
using RosMessageTypes.Sensor;

using ROS.Core;
using Unity.Robotics.Core;
using dji;


namespace M350.PSDK_ROS2
{
    [AddComponentMenu("Smarc/PSDK_ROS/PsdkJoySubscriber")]
    public class PsdkJoySubscriber : ROSBehaviour
    {
        protected string tf_prefix;
        public float joy_timeout = 0.5f;
        public float time_since_joy;

        bool registered = false;
        DJIController controller;

        JoyMsg joy;


        protected override void StartROS()
        {
            if (controller == null) controller = GetComponentInParent<DJIController>();

            JoyMsg ROSMsg = new();
            if (!registered)
            {
                rosCon.Subscribe<JoyMsg>(topic, _joy_sub_callback);
                registered = true;
            }
        }

        void _joy_sub_callback(JoyMsg msg)
        {
            joy = msg;
        }

        void FixedUpdate()
        {
            if (joy == null) return;
            if (controller == null)
            {
                controller = GetComponentInParent<DJIController>();
            }
            if (controller != null && joy != null)
            {
                time_since_joy = (float)Clock.time - joy.header.stamp.sec - joy.header.stamp.nanosec / Mathf.Pow(10f, 9f);
                if (time_since_joy < joy_timeout && joy.axes.Length >= 3)
                {
                    controller.CommandFLUYawRate(joy.axes[0], joy.axes[1], joy.axes[2], 0f);
                }
                else
                {
                    controller.CommandFLUYawRate(0f, 0f, 0f, 0f);
                    joy = null;
                }
            }
        }

    }
}
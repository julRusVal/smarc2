using UnityEngine;
using VehicleComponents.Actuators;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


namespace ROS.Subscribers
{

    [RequireComponent(typeof(IGenericTwistActuator))]
    public class GenericTwistCommand_Sub : Actuator_Sub<TwistStampedMsg>
    {
        IGenericTwistActuator twistAct;

        void Awake()
        {
            twistAct = GetComponent<IGenericTwistActuator>();
        }

        protected override void UpdateVehicle(bool reset)
        {
            if(twistAct == null)
            {
                Debug.Log($"GenericTwistCommand_Sub found no IGenericTwistActuator to command! Disabling.");
                enabled = false;
                rosCon.Unsubscribe(topic);
                return;
            }

            if(reset)
            {
                twistAct.SetTwist(twistAct.GetResetValue().Item1, twistAct.GetResetValue().Item2);
                return;
            }

            // ROS twist to Unity twist
            // FLU (ROS) to RUF (Unity)
            var linear = ROSMsg.twist.linear;
            var angular = ROSMsg.twist.angular;
            twistAct.SetTwist(
                FLU.ConvertToRUF(new Vector3(
                    (float)linear.x,
                    (float)linear.y,
                    (float)linear.z
                )),
                FLU.ConvertAngularVelocityToRUF(new Vector3(
                    (float)angular.x,
                    (float)angular.y,
                    (float)angular.z
                ))
            );


        }
    }
}
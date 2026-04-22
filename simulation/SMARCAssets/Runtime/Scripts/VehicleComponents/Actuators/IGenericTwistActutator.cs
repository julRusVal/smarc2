using UnityEngine;
using ROS.Core;

namespace VehicleComponents.Actuators
{
    /// <summary>
    /// An interface for actuators that can be controlled by setting a desired linear and angular velocity (twist). 
    /// This is useful for high-level controllers that want to abstract away the details of how the actuator achieves the desired motion.
    /// IMPORTANT: It is the sub/pub's duty to convert from unity to ros coordinate frames!
    /// So the vectors here are RUF!
    /// </summary>
    public interface IGenericTwistActuator : IROSPublishable
    {
        public void SetTwist(Vector3 LinearVelocity, Vector3 AngularVelocity);
        public (Vector3, Vector3) GetResetValue();
        public (Vector3, Vector3) GetCurrentValue();
    }
}
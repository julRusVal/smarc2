using UnityEngine;

namespace VehicleComponents.Actuators
{
    [AddComponentMenu("Smarc/Actuator/Gimbal")]
    public class Gimbal : MonoBehaviour
    {

        [Header("Gimbal Rotation (degrees)")]
        public float roll = 0f;
        public float pitch = 0f;
        public float yaw = 0f;

        void LateUpdate()
        {
            Transform parent = transform.parent;
            Vector3 worldUp = Vector3.up;

            // Parent forward projected onto the gravity-horizontal plane.
            Vector3 parentForward = parent ? parent.forward : Vector3.forward;
            Vector3 yawForward = Vector3.ProjectOnPlane(parentForward, worldUp);

            // Fallback in case parent is pointing straight up/down.
            if (yawForward.sqrMagnitude < 0.0001f)
                yawForward = Vector3.ProjectOnPlane(parent ? parent.up : Vector3.forward, worldUp);

            if (yawForward.sqrMagnitude < 0.0001f)
                yawForward = Vector3.forward;

            yawForward.Normalize();

            // Base rotation: yaw 0 matches parent's yaw, but ignores parent's pitch/roll.
            Quaternion baseYaw = Quaternion.LookRotation(yawForward, worldUp);

            // Yaw input is relative to parent yaw, around gravity up.
            Quaternion yawRot = Quaternion.AngleAxis(yaw, worldUp);

            // After yaw, define local axes for pitch/roll in the gravity-based frame.
            Quaternion yawFrame = yawRot * baseYaw;
            Vector3 rightAxis   = yawFrame * Vector3.right;
            Vector3 forwardAxis = yawFrame * Vector3.forward;

            // Pitch around local right, roll around local forward.
            Quaternion pitchRot = Quaternion.AngleAxis(pitch, rightAxis);
            Quaternion rollRot  = Quaternion.AngleAxis(roll, forwardAxis);

            transform.rotation = rollRot * pitchRot * yawFrame;
        }
    }
}
using UnityEngine;
using Force;


namespace Smarc.GenericControllers
{
    public enum HorizontalControlMode
    {
        UnityPosition,
        Velocity
    }

    [AddComponentMenu("Smarc/Generic Controllers/Horizontal Controller")]
    public class HorizontalController : MonoBehaviour
    {
        public ArticulationBody RobotAB;
        public Rigidbody RobotRB;
        MixedBody robotBody;
        public HorizontalControlMode ControlMode = HorizontalControlMode.UnityPosition;
        public float MaxForce = 0f;
        public float MaxSpeed = 5.0f;

        [Header("Velocity Controller")]
        public Vector3 TargetVelocity = Vector3.zero;

        [Header("Unity Position Controller")]
        public Vector3 TargetUnityPosition = Vector3.zero;
        public float PositionTolerance = 0.5f;


        [Header("Velocity PID")]
        public float VelKp = 5.0f;
        public float VelKi = 0.0f;
        public float VelKd = 0.0f;
        public float VelIntegratorLimit = 5f;
        PID velPID;
        
        public Vector3 LastAppliedForce { get; private set; }
        public Vector3 LastAppliedForceLocal {get; private set;}



        // Use a generated object to apply force at center of mass, parented to the robot transform
        // This way, we don't have to recalculate the world position of the COM every frame
        Transform COM;


        void Start()
        {
            robotBody = new MixedBody(RobotAB, RobotRB);
            velPID = new PID(VelKp, VelKi, VelKd, VelIntegratorLimit, maxOutput:MaxForce);

            var globalCom = robotBody.GetTotalConnectedCenterOfMass();
            COM = new GameObject("HorizontalController_COM").transform;
            COM.parent = robotBody.transform;
            COM.position = globalCom;

            // set to current position so it doesnt try to fly away to (usually) origin lol
            if (ControlMode == HorizontalControlMode.UnityPosition)
            {
                TargetUnityPosition = COM.position;
            }

        }

        void FixedUpdate()
        {
            // check if the robot is upright enough to control horizontal movement
            var upDot = Vector3.Dot(robotBody.transform.up, Vector3.up);
            if (upDot < 0.5f)
            {
                Debug.Log($"Robot too tilted for horizontal control! upDot: {upDot}");
                return;
            }

            var currentSpeed = robotBody.localVelocity.magnitude;
            if (currentSpeed > MaxSpeed*10f)
            {
                Debug.Log($"Robot moving too fast for horizontal control! currentSpeed: {currentSpeed}");
                return;
            }

           

            if (ControlMode == HorizontalControlMode.UnityPosition && currentSpeed <= MaxSpeed)
            {
                if (currentSpeed > MaxSpeed) TargetVelocity = Vector3.zero;
                else
                {
                    Vector3 diff = TargetUnityPosition - COM.position;
                    if (diff.magnitude <= PositionTolerance) TargetVelocity = Vector3.zero;
                    else TargetVelocity = diff.normalized * MaxSpeed;
                    Debug.DrawLine(COM.position, TargetUnityPosition, Color.green);
                }
            }
            
            TargetVelocity.y = 0;
            VelocityControl();
        }

        void VelocityControl()
        {
            Vector3 currentVelocity = robotBody.localVelocity;
            currentVelocity.y = 0;
            if(TargetVelocity.magnitude > MaxSpeed)
            {
                TargetVelocity = TargetVelocity.normalized * MaxSpeed;
            }

            Vector3 force = velPID.UpdateVector3(TargetVelocity, currentVelocity, Time.fixedDeltaTime);
            force.y = 0;

            LastAppliedForceLocal = force;
            
            force = robotBody.transform.TransformVector(force);
            robotBody.AddForceAtPosition(force, COM.position, ForceMode.Force);
            LastAppliedForce = force;
        }

    }
}

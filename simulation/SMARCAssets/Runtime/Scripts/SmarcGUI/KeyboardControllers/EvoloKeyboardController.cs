using UnityEngine;
using UnityEngine.InputSystem;
using Evolo;
using ROS.Subscribers;

namespace SmarcGUI.KeyboardControllers
{

    [RequireComponent(typeof(EvoloController))]
    [RequireComponent(typeof(GenericTwistCommand_Sub))]
    public class EvoloKeyboardController : KeyboardControllerBase
    {
        InputAction forwardAction, strafeAction, verticalAction;
        EvoloController evoloCtrl;
        GenericTwistCommand_Sub twistSub;
        bool twistSubState;

        public float SpeedChangeRate = 0.1f;
        public float YawChangeRate = 0.1f;
        public float AltitudeChangeRate = 0.1f;

        void OnEnable()
        {
            twistSubState = twistSub.enabled;
            twistSub.enabled = false;
        }

        void OnDisable()
        {
            twistSub.enabled = twistSubState;
        }



        void Awake()
        {
            forwardAction = InputSystem.actions.FindAction("Robot/Forward");
            strafeAction = InputSystem.actions.FindAction("Robot/Strafe");
            verticalAction = InputSystem.actions.FindAction("Robot/UpDown");
            
            evoloCtrl = GetComponent<EvoloController>();
            twistSub = GetComponent<GenericTwistCommand_Sub>();
        }

        void Update()
        {
            var forwardValue = forwardAction.ReadValue<float>();
            var strafeValue = strafeAction.ReadValue<float>();
            var verticalValue = verticalAction.ReadValue<float>();

            if (Mathf.Sign(forwardValue) == Mathf.Sign(evoloCtrl.Speed)) evoloCtrl.Speed += SpeedChangeRate * forwardValue;
            else evoloCtrl.Speed = 0;

            evoloCtrl.YawRate += YawChangeRate * strafeValue;
            evoloCtrl.Altitude += verticalValue * AltitudeChangeRate;
        }

        public override void OnReset()
        {
            evoloCtrl.YawRate = 0f;
            evoloCtrl.Speed = 0f;
            evoloCtrl.Altitude = 0f;
        }

    }
}
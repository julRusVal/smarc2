using UnityEngine;
using Smarc.Rope;

namespace Smarc.Rope
{
    public class TwoSegmentWinchPulley : MonoBehaviour
    {
        [Header("Loads")]
        public Transform GlobalLoadTfOne;
        public Transform GlobalLoadTfTwo;

        [Header("Winches")]
        public TwoSegmentWinch WinchOne;
        public TwoSegmentWinch WinchTwo;

        [Header("Settings")]
        public float RopeLength = 5f;
        public float PulleySpeed = 0.5f;

        [Header("Current State")]
        public float loadOne;
        public float loadTwo;
        public bool tenseOne, tenseTwo;
        public bool onePulls, twoPulls;
        public float speed;

        public void ApplySettings()
        {
            // gotta disable/enable because articulation bodies
            WinchOne.gameObject.SetActive(false);
            WinchTwo.gameObject.SetActive(false);
            WinchOne.RopeLength = RopeLength;
            WinchTwo.RopeLength = RopeLength;

            Vector3 posOne = GlobalLoadTfOne == null ? transform.position + new Vector3(RopeLength/2, 0f, 0f) : GlobalLoadTfOne.position;
            Vector3 posTwo = GlobalLoadTfTwo == null ? transform.position + new Vector3(-RopeLength/2, 0f, 0f) : GlobalLoadTfTwo.position;
            
            // first rotate the winches to match the direction towards the targets
            WinchOne.transform.LookAt(posOne);
            WinchOne.transform.Rotate(-90f, 0f, 0f); // because the winches are "negative Y forward"
            WinchTwo.transform.LookAt(posTwo);
            WinchTwo.transform.Rotate(-90f, 0f, 0f);

            // Then set the starting rope length of winch one
            // to match the distance to the target, limited by the total rope length
            // and give the remaining rope length to winch two
            var toTargetOne = Vector3.Distance(transform.position, posOne);
            WinchOne.StartingRopeLength = Mathf.Clamp(toTargetOne, WinchOne.MinRopeLength, RopeLength);
            WinchTwo.StartingRopeLength = Mathf.Clamp(RopeLength - WinchOne.StartingRopeLength, WinchTwo.MinRopeLength, RopeLength);

            WinchOne.ApplySettings();
            WinchTwo.ApplySettings();
            WinchOne.gameObject.SetActive(true);
            WinchTwo.gameObject.SetActive(true);
        }

        void Awake()
        {
            ApplySettings();   
            // Ignore all self-collisions
            Collider[] colliders = GetComponentsInChildren<Collider>();
            foreach(var col in colliders)
                foreach(var col2 in colliders)
                    if (col != col2)
                        Physics.IgnoreCollision(col, col2);
        }

        void FixedUpdate()
        {
            // we want to move change the length of the ropes (by adjusting winch speeds) 
            // such that the loaded side lengthens and the other side shortens, but the total length of the rope remains constant
            // the speed of length change should be proportional to the load difference
            loadOne = WinchOne.GetTopLoad();
            loadTwo = WinchTwo.GetTopLoad();
            // these two are mutually exclusive but not exhaustive, both can be false.
            // ex: one is extended, and load one is larger. So nothing should move, because the rope is already extended on the side with more load.
            tenseOne = WinchOne.IsTense();
            tenseTwo = WinchTwo.IsTense();
            onePulls = !tenseOne && loadOne > loadTwo;
            twoPulls = !tenseTwo && loadTwo > loadOne;
            
            if (onePulls)
            {
                speed = PulleySpeed;
            }
            else if (twoPulls)
            {
                speed = -PulleySpeed;
            }
            else
            {
                speed = 0f;
            }

            WinchOne.PullSpeed = -speed;
            WinchTwo.PullSpeed = speed;
            
        }
    }
}
using UnityEngine;
using Smarc.Rope;

namespace Smarc.Alars
{
    [RequireComponent(typeof(Collider))]
    public class AlarsSequencer : MonoBehaviour
    {
        public Collider BuoyCollider;
        public GameObject SamRopeBuoySystem;
        public TwoSegmentWinch Winch;
        public TwoSegmentWinchPulley WinchPulley;

        void Awake()
        {
            if (SamRopeBuoySystem == null) 
            {
                Debug.LogWarning("SamRopeBuoySystem is not assigned in AlarsSequencer! Please assign it in the inspector.");
                Debug.LogWarning("Using Find to locate it in the scene, but this is not recommended and may cause issues if there are multiple SamRopeBuoySystems in the scene.");
                SamRopeBuoySystem = GameObject.Find("SamWithRopeAndBuoy");
                if (SamRopeBuoySystem == null) Debug.LogError("Could not find \"SamWithRopeAndBuoy\" in the scene! Please assign the SamRopeBuoySystem in the inspector.");
            }

            if (BuoyCollider == null)
            {
                Debug.LogWarning("BuoyCollider is not assigned in AlarsSequencer! Please assign it in the inspector.");
                Debug.LogWarning("Using Find to locate it in the scene, but this is not recommended and may cause issues if there are multiple colliders in the scene.");
                GameObject buoy = GameObject.Find("SamWithRopeAndBuoy/Buoy");
                if (buoy == null) Debug.LogError("Could not find \"SamWithRopeAndBuoy/Buoy\" in the scene! Please assign the BuoyCollider in the inspector.");
                else BuoyCollider = buoy.GetComponent<Collider>();
            } 
        }

        void OnTriggerExit(Collider other)
        {
            if (other == BuoyCollider)
            {
                SamRopeBuoySystem.SetActive(false);
                WinchPulley.ApplySettings();
                Winch.EnableLoad();
                enabled = false;
                Debug.Log("Buoy left hook, replacing sam-rope-buoy with load of the winch!");
            }
        }
        
    }
}
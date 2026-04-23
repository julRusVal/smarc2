using System.Collections.Generic;
using UnityEngine;

namespace VehicleComponents.Sensors
{


    [AddComponentMenu("Smarc/Sensor/LoadCell")]
    public class LoadCell : Sensor
    {
        [Header("LoadCell")]
        public bool PositiveOnly = true;
        public int SmoothOverFrames = 5;
        Queue<float> forceReadings = new();
        public float Force;
        public float Weight;
        public Joint joint;
        public ArticulationBody body;



        public override bool UpdateSensor(double deltaTime)
        {
            float instantForce = 0;
            if (body != null && body.transform.parent != null)
            {
                ArticulationReducedSpace forces = body.driveForce;
                if (forces.dofCount == 1) instantForce = forces[0];
                else Debug.LogWarning($"LoadCell only supports 1 DOF joints (prismatic, revolute), but joint has {forces.dofCount} DOFs.");
            }

            if (joint != null) instantForce = joint.currentForce.magnitude;

            if (PositiveOnly && instantForce < 0) instantForce = 0;

            forceReadings.Enqueue(instantForce);
            
            while (forceReadings.Count > SmoothOverFrames) forceReadings.Dequeue();
            float averageForce = 0;
            foreach (float f in forceReadings) averageForce += f;
            averageForce /= forceReadings.Count;

            Force = averageForce;
            Weight = Force / Physics.gravity.magnitude;
            return true;

        }

    }

}
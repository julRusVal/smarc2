using UnityEngine;
using Smarc.Rope;

namespace Smarc.Rope
{
    public class TwoSegmentWinch : MonoBehaviour
    {
        [Header("Components")]
        public ArticulationBody BaseSpherical;
        public ArticulationBody MiddlePrismatic;
        public BoxCollider MiddlePrismaticCollider;
        ArticulationDrive midPrismaticDrive;
        public ArticulationBody MiddleSpherical;
        public ArticulationBody EndPrismatic;
        public BoxCollider EndPrismaticCollider;
        ArticulationDrive endPrismaticDrive;
        public GameObject LoadTree;

        [Header("Controls")]
        [Range(-2f, 2f)]
        public float PullSpeed = 0f;

        [Header("Load Controls")]
        [Tooltip("If true, the loadTree object will be disabled at Awake. Enable it when the load is attached later on")]
        public bool DisableLoadAtAwake = true;
        public float LoadedDamping = 10f;
        public float LoadedAngularDamping = 10f;

        [Header("Settings")]
        public Vector3 Direction = Vector3.down;
        public float RopeLength = 5f;
        public float MinRopeLength = 0.2f;
        public float StartingRopeLength = 2f;
        public float ColliderThickness = 0.1f;
        public Material RopeMaterial;
        public float RopeVisualThickness = 0.05f;
        LineRenderer lineRenderer;

        [Header("Current State")]
        public float CurrentRopeLength;


        void Awake()
        {
            // Make sure the targets start at 0, so that they dont snap and break physics.
            // The lower/upper limits are set by ApplySettings assuming their starting position is at 0 already.
            midPrismaticDrive = MiddlePrismatic.xDrive;
            midPrismaticDrive.target = 0f;
            MiddlePrismatic.xDrive = midPrismaticDrive;
            endPrismaticDrive = EndPrismatic.xDrive;
            endPrismaticDrive.target = 0f;
            EndPrismatic.xDrive = endPrismaticDrive;

            // Ignore all self-collisions in the winch
            Collider[] colliders = GetComponentsInChildren<Collider>();
            foreach(var col in colliders)
                foreach(var col2 in colliders)
                    if (col != col2)
                        Physics.IgnoreCollision(col, col2);
                
            if (DisableLoadAtAwake) LoadTree.SetActive(false);

            // Add a line renderer that goes between the ends and middle joint
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.positionCount = 3; 
            lineRenderer.material = RopeMaterial;
            lineRenderer.startWidth = RopeVisualThickness;
            lineRenderer.endWidth = RopeVisualThickness;
        }

        void FixedUpdate()
        {
            midPrismaticDrive.target += PullSpeed/2f * Time.fixedDeltaTime;
            midPrismaticDrive.target = Mathf.Clamp(midPrismaticDrive.target, midPrismaticDrive.lowerLimit, midPrismaticDrive.upperLimit);
            MiddlePrismatic.xDrive = midPrismaticDrive;

            endPrismaticDrive.target += PullSpeed/2f * Time.fixedDeltaTime;
            endPrismaticDrive.target = Mathf.Clamp(endPrismaticDrive.target, endPrismaticDrive.lowerLimit, endPrismaticDrive.upperLimit);
            EndPrismatic.xDrive = endPrismaticDrive;

            CurrentRopeLength = -MiddlePrismatic.transform.localPosition.y + -EndPrismatic.transform.localPosition.y;
            if (CurrentRopeLength <= MinRopeLength) PullSpeed = Mathf.Min(0, PullSpeed);
            if (IsTense()) PullSpeed = Mathf.Max(0, PullSpeed);

            if (LoadTree.activeSelf)
            {
                // if the load is attached, we want the joints (which the end now is too) to have some damping
                // otherwise under heavy loads, the middle joints oscillate a lot and can cause physics issues
                MiddlePrismatic.linearDamping = LoadedDamping;
                MiddlePrismatic.angularDamping = LoadedAngularDamping;
                EndPrismatic.linearDamping = LoadedDamping;
                EndPrismatic.angularDamping = LoadedAngularDamping;
            }

            SetColliders();

            // Update the line renderer to visually connect the joints
            lineRenderer.SetPosition(0, BaseSpherical.transform.position);
            lineRenderer.SetPosition(1, MiddlePrismatic.transform.position);
            lineRenderer.SetPosition(2, EndPrismatic.transform.position);
        }

        void SetColliders()
        {
            var midDist = -MiddlePrismatic.transform.localPosition.y;
            if (midDist < 0.1f) midDist = 0.1f;
            MiddlePrismaticCollider.size = new Vector3(ColliderThickness, midDist, ColliderThickness);
            MiddlePrismaticCollider.center = new Vector3(0, midDist/2, 0);

            var endDist = -EndPrismatic.transform.localPosition.y;
            if (endDist < 0.1f) endDist = 0.1f;
            EndPrismaticCollider.size = new Vector3(ColliderThickness, endDist, ColliderThickness);
            EndPrismaticCollider.center = new Vector3(0, endDist/2, 0);
        }

        public void ApplySettings()
        {
            // Sphericals are always at the same position as prismatics
            BaseSpherical.transform.localPosition = Vector3.zero;
            MiddleSpherical.transform.localPosition = Vector3.zero;

            var l = RopeLength - StartingRopeLength;
            // First, put the prismatics where they need to be when the winch is fully extended
            var midLen = Direction.normalized * (RopeLength-l) / 2f;
            MiddlePrismatic.transform.localPosition = midLen;
            EndPrismatic.transform.localPosition = midLen;
            SetColliders();

            // Then, set the prismatics to have the correct min/max limits
            var midPrismaticDrive = MiddlePrismatic.xDrive;
            midPrismaticDrive.lowerLimit = -l/2f;
            midPrismaticDrive.upperLimit = (RopeLength-l) / 2f;
            MiddlePrismatic.xDrive = midPrismaticDrive;

            var endPrismaticDrive = EndPrismatic.xDrive;
            endPrismaticDrive.lowerLimit = -l/2f;
            endPrismaticDrive.upperLimit = (RopeLength-l) / 2f;
            EndPrismatic.xDrive = endPrismaticDrive;
        }

        void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(BaseSpherical.transform.position, MiddlePrismatic.transform.position);
            Gizmos.DrawLine(MiddleSpherical.transform.position, EndPrismatic.transform.position);
            Gizmos.DrawSphere(MiddleSpherical.transform.position, 0.05f);   
        }

        public float GetTopLoad()
        {
            ArticulationReducedSpace force = MiddlePrismatic.driveForce;
            return force[0];
        }

        public bool IsTense()
        {
            return CurrentRopeLength >= RopeLength - 0.05f;
        }

        public void EnableLoad()
        {
            LoadTree.SetActive(true);
        }


    }
}
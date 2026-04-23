using UnityEngine;
using DefaultNamespace.Water;
using Force;
using Smarc.Rope;

/// <summary>
/// Source: https://github.com/GaryMcWhorter/Verlet-Chain-Unity/tree/main
/// Modified to interact with rigid/arti bodies on the two ends
/// </summary>

namespace Smarc.Rope
{
    [RequireComponent(typeof(LineRenderer))]
    public class RopeVerlet : MonoBehaviour
    {
        [Header("Physical Properties")]
        public Rigidbody StartRB;
        public ArticulationBody StartAB;
        MixedBody StartBody;

        public Rigidbody EndRB;
        public ArticulationBody EndAB;
        MixedBody EndBody;

        public bool Float = true;
        [Tooltip("Set to the layer that you want the rope to collide with.")]
        public int RopeLayer = 6;


        [Min(0f)]
        public float RopeLength = 1f;
        public float RopeWidth = 0.1f;
        [Min(2), Tooltip("The number of chain links. Decreases performance with high values and high iteration")]
        public int SegmentCount = 10;
        [Min(0f), Tooltip("How much does the rope transfer forces across its length between the two ends.")]
        public float BodyPullMultiplier = 1f;


        [Space]

        [Header("Instanced Mesh Details")]
        [Tooltip("The Mesh of chain link to render")] 
        public Mesh Link;

        [Tooltip("The chain link material, must have gpu instancing enabled!")] 
        public Material RopeMaterial;


        [Header("Verlet Parameters")]
        [Tooltip("The distance between each link in the chain")] 
        float nodeDistance{ get { return RopeLength / (SegmentCount - 1); } }

        [SerializeField, Tooltip("The radius of the sphere collider used for each chain link")] 
        float nodeColliderRadius = 0.2f;

        [SerializeField, Tooltip("Works best with a lower value")] 
        float gravityStrength = 2f;


        [SerializeField, Range(0, 1), Tooltip("Modifier to dampen velocity so the simulation can stabilize")] 
        float velocityDampen = 0.95f;

        [SerializeField, Range(0, 0.99f), Tooltip("The stiffness of the simulation. Set to lower values for more elasticity")] 
        float stiffness = 0.8f;

        [SerializeField, Tooltip("Setting this will test collisions for every n iterations. Possibly more performance but less stable collisions")] 
        int iterateCollisionsEvery = 1;

        [SerializeField, Tooltip("Iterations for the simulation. More iterations is more expensive but more stable")] 
        int iterations = 100;

        [SerializeField, Tooltip("How many colliders to test against for every node.")] 
        int colliderBufferSize = 1;


        [Header("Diagnostic")]
        public float CurrentRopeLength;


        Collider[] colliderHitBuffer;

        WaterQueryModel waterQueryModel;
        Vector3 gravity;

        LineRenderer lineRenderer;
        Vector3[] linePositions;

        Vector3[] previousNodePositions;

        Vector3[] currentNodePositions;
        Quaternion[] currentNodeRotations;

        SphereCollider nodeCollider;
        GameObject nodeTester;
        Matrix4x4[] matrices;


    

        void Awake()
        {
            StartBody = new(StartAB, StartRB);
            EndBody = new(EndAB, EndRB);

            currentNodePositions = new Vector3[SegmentCount];
            previousNodePositions = new Vector3[SegmentCount];
            currentNodeRotations = new Quaternion[SegmentCount];

            colliderHitBuffer = new Collider[colliderBufferSize];
            gravity = new Vector3(0, -gravityStrength, 0);
            lineRenderer = GetComponent<LineRenderer>();
            lineRenderer.material = RopeMaterial;
            lineRenderer.enabled = RopeMaterial != null;

            // using a single dynamically created GameObject to test collisions on every node
            nodeTester = new GameObject
            {
                name = "Node Tester",
                layer = RopeLayer
            };
            nodeCollider = nodeTester.AddComponent<SphereCollider>();
            nodeCollider.radius = nodeColliderRadius;


            matrices = new Matrix4x4[SegmentCount];

            Vector3 startPosition = Vector3.zero;
            if (StartBody.isValid && EndBody.isValid)
            {
                startPosition = (StartBody.position + EndBody.position) * 0.5f;
            }
            else if (StartBody.isValid)
            {
                startPosition = StartBody.position;
            }
            else if (EndBody.isValid)
            {
                startPosition = EndBody.position;
            }
            for (int i = 0; i < SegmentCount; i++)
            {
                currentNodePositions[i] = startPosition;
                currentNodeRotations[i] = Quaternion.identity;
                previousNodePositions[i] = startPosition;
                matrices[i] = Matrix4x4.TRS(startPosition, Quaternion.identity, Vector3.one);
                startPosition.y -= nodeDistance;
            }

            // for line renderer data
            linePositions = new Vector3[SegmentCount];

            if (Float) waterQueryModel = WaterQueryModel.GetWaterQueryModel();

        }


        void Update()
        {
            DrawRope();

            if (Link != null)
            {
                Mesh scaledLink = Link;
                if (Link.bounds.size.z != nodeDistance)
                {
                    // if the link mesh isn't the correct length, scale it in the shader using instancing data
                    scaledLink = Instantiate(Link);
                    float s = nodeDistance / Link.bounds.size.z;
                    Vector3 scale = new Vector3(s,s,s);
                    Vector3[] vertices = scaledLink.vertices;
                    for (int i = 0; i < vertices.Length; i++)
                    {
                        vertices[i] = Vector3.Scale(vertices[i], scale);
                    }
                    scaledLink.vertices = vertices;
                    scaledLink.RecalculateBounds();
                }
                // Instanced drawing here is really performant over using GameObjects
                Graphics.DrawMeshInstanced(scaledLink, 0, RopeMaterial, matrices, SegmentCount);
            }

            CurrentRopeLength = 0f;
            for (int i = 0; i < SegmentCount - 1; i++)
            {
                CurrentRopeLength += Vector3.Distance(currentNodePositions[i], currentNodePositions[i + 1]);
            }
        }

        void FixedUpdate()
        {
            Simulate();
            if (Float) SetOnWaterSurface();

            for (int i = 0; i < iterations; i++)
            {
                ApplyConstraint();

                if(i % iterateCollisionsEvery == 0)
                {
                    AdjustCollisions();
                }
            }
            PullOnEnd(StartBody);
            PullOnEnd(EndBody);


            SetAngles();
            TranslateMatrices();
        }

        void SetOnWaterSurface()
        {
            if (waterQueryModel == null) return;

            for (int i = 0; i < SegmentCount; i++)
            {
                float waterLevel = waterQueryModel.GetWaterLevelAt(currentNodePositions[i]);
                if (currentNodePositions[i].y < waterLevel)
                {
                    currentNodePositions[i] = new Vector3(currentNodePositions[i].x, waterLevel, currentNodePositions[i].z);
                }   
            }
        }

        void Simulate()
        {
            var fixedDt = Time.fixedDeltaTime;
            for (int i = 0; i < SegmentCount; i++)
            {
                Vector3 velocity = currentNodePositions[i] - previousNodePositions[i];
                velocity *= velocityDampen;

                previousNodePositions[i] = currentNodePositions[i];

                // calculate new position
                Vector3 newPos = currentNodePositions[i] + velocity;
                newPos += gravity * fixedDt;
                Vector3 direction = currentNodePositions[i] - newPos;

                currentNodePositions[i] = newPos;
            }
        }

        Transform GetTopParent(Transform t)
        {
            if (t.parent == null) return t;
            return GetTopParent(t.parent);
        }
        
        void AdjustCollisions()
        {
            for (int i = 0; i < SegmentCount; i++)
            {
                if(i % 2 == 0) continue;

                int result = -1;
                result = Physics.OverlapSphereNonAlloc(currentNodePositions[i], nodeColliderRadius + 0.01f, colliderHitBuffer, ~(1 << 8));
                for (int n = 0; n < result; n++)
                {
                    Collider coll = colliderHitBuffer[n];
                    // check if this collider is one of our end locks, if so skip it
                    if (StartBody.isValid && GetTopParent(coll.transform) == GetTopParent(StartBody.transform)) continue;
                    if (EndBody.isValid && GetTopParent(coll.transform) == GetTopParent(EndBody.transform)) continue;
                    if (coll.isTrigger) continue;
                    Vector3 colliderPosition = coll.transform.position;
                    Quaternion colliderRotation = coll.gameObject.transform.rotation;

                    Physics.ComputePenetration(nodeCollider, currentNodePositions[i], Quaternion.identity, coll, colliderPosition, colliderRotation, out Vector3 dir, out float distance);

                    currentNodePositions[i] += dir * distance;
                }
            }

        }

        void PullOnEnd(MixedBody body)
        {
            if (!body.isValid) return;

            int nodeIndex = body == StartBody ? 0 : SegmentCount - 1;
            
            // we want the rope to apply force to the ends based on
            // 1) how far the stretch is
            Vector3 endPosition = currentNodePositions[nodeIndex];
            Vector3 forceDirection = (endPosition - body.position).normalized;
            Debug.DrawRay(body.position, forceDirection, Color.red);
            float stretch = Vector3.Distance(endPosition, body.position);
            // 2) the mass difference. The heavier one gets pulled _by the lighter one_.
            float endMass = body == StartBody ? StartBody.mass : EndBody.mass;
            float otherEndMass = body == StartBody ? EndBody.mass : StartBody.mass;
            float pullRatio = endMass / (endMass + otherEndMass);

            float forceMagnitude = stretch * pullRatio * BodyPullMultiplier * 100f;
            Vector3 force = forceDirection * forceMagnitude;

            body.AddForceAtPosition(force, endPosition, ForceMode.Force);
        }


        void ApplyConstraint()
        {
            if(StartBody.isValid) currentNodePositions[0] = StartBody.position;
            if(EndBody.isValid) currentNodePositions[SegmentCount - 1] = EndBody.position;

            for (int i = 0; i < SegmentCount - 1; i++)
            {
                var node1 = currentNodePositions[i];
                var node2 = currentNodePositions[i + 1];

                // Get the current distance between rope nodes
                float currentDistance = (node1 - node2).magnitude;
                float difference = Mathf.Abs(currentDistance - nodeDistance);
                Vector3 direction = Vector3.zero;

                // determine what direction we need to adjust our nodes
                if (currentDistance > nodeDistance)
                {
                    direction = (node1 - node2).normalized;
                }
                else if (currentDistance < nodeDistance)
                {
                    direction = (node2 - node1).normalized;
                }

                // calculate the movement vector, split equally between nodes by default
                Vector3 totalMovement = difference * stiffness * direction;
                Vector3 movement_i = totalMovement * 0.5f;
                Vector3 movement_i1 = totalMovement * 0.5f;
                
                // apply correction
                currentNodePositions[i] -= movement_i;
                currentNodePositions[i + 1] += movement_i1;
            }
        }

        void SetAngles()
        {
            for (int i = 0; i < SegmentCount - 1; i++)
            {
                var node1 = currentNodePositions[i];
                var node2 = currentNodePositions[i + 1];

                var dir = (node2 - node1).normalized;
                if(dir != Vector3.zero)
                {
                    if( i > 0)
                    {
                        Quaternion desiredRotation = Quaternion.LookRotation(dir, Vector3.right);
                        currentNodeRotations[i + 1] = desiredRotation;
                    }
                    else if( i < SegmentCount - 1)
                    {
                        Quaternion desiredRotation = Quaternion.LookRotation(dir, Vector3.right);
                        currentNodeRotations[i + 1] = desiredRotation;
                    }
                    else
                    {
                        Quaternion desiredRotation = Quaternion.LookRotation(dir, Vector3.right);
                        currentNodeRotations[i] = desiredRotation;
                    }
                }

                if( i % 2 == 0 && i != 0)
                {
                    currentNodeRotations[i + 1] *= Quaternion.Euler(0, 0, 90);
                }
            }
        }

        void TranslateMatrices()
        {
            for(int i = 0; i < SegmentCount; i++)
            {
                matrices[i].SetTRS(currentNodePositions[i], currentNodeRotations[i], Vector3.one);
            }
        }

        void DrawRope()
        {
            lineRenderer.startWidth = RopeWidth;
            lineRenderer.endWidth = RopeWidth;

            for (int n = 0; n < SegmentCount; n++)
            {
                linePositions[n] = currentNodePositions[n];
            }

            lineRenderer.positionCount = linePositions.Length;
            lineRenderer.SetPositions(linePositions);
        }

        void OnDrawGizmos()
        {
            Vector3 p1 = Vector3.zero, p2 = Vector3.zero;
            if (StartAB != null) p1 = StartAB.transform.position;
            else if (StartRB != null) p1 = StartRB.transform.position;
            if (EndAB != null) p2 = EndAB.transform.position;
            else if (EndRB != null) p2 = EndRB.transform.position;

            if (p1 != Vector3.zero && p2 != Vector3.zero)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(p1, p2);
            }
        }

    }
    
}

using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.Core;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

namespace ROS.Publishers
{
    class TransformTreeNode
    {
        public readonly GameObject SceneObject;
        public readonly List<TransformTreeNode> Children;
        public Transform Transform => SceneObject.transform;
        public string name => SceneObject.name;
        public bool IsALeafNode => Children.Count == 0;

        public TransformTreeNode(GameObject sceneObject)
        {
            SceneObject = sceneObject;
            Children = new List<TransformTreeNode>();
            PopulateChildNodes(this);
        }


        public static TransformStampedMsg ToTransformStamped(TransformTreeNode node)
        {
            return node.Transform.ToROSTransformStamped(Clock.time);
        }

        static void PopulateChildNodes(TransformTreeNode tfNode)
        {
            var parentTransform = tfNode.Transform;
            for (var childIndex = 0; childIndex < parentTransform.childCount; ++childIndex)
            {
                var childTransform = parentTransform.GetChild(childIndex);
                var childGO = childTransform.gameObject;

                // If game object has a URDFLink attached, it's a link in the transform tree
                if (childGO.TryGetComponent(out UrdfLink _))
                {
                    // Unless it has its own ROSTranformTreePublisher. Then we stop here and let that publisher handle the subtree
                    // This allows for partial publishing of transform trees, which is useful for modular robots where each module may have its own publisher
                    // or the TF is published by a driver on a real robot, and we dont want to run the driver for sim, but still want the TF that the driver
                    // _would_ publish.
                    if (childGO.TryGetComponent(out ROSTransformTreePublisher _)) continue;
                    var childNode = new TransformTreeNode(childGO);
                    tfNode.Children.Add(childNode);
                }
            }
        }
    }
}
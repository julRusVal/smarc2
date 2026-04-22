using RosMessageTypes.Geometry;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using ROS.Core;
using Force;
using UnityEngine;


namespace M350.PSDK_ROS2
{
    [AddComponentMenu("Smarc/PSDK_ROS/PsdkAttitude")]
    public class PsdkAttitude : ROSPublisher<QuaternionStampedMsg>
    {
        
        MixedBody body;

        protected override void InitPublisher()
        {
            GetMixedBody(out body);
        }
        
        protected override void UpdateMessage()
        {
            var quaternion = body.transform.rotation;
            ROSMsg.quaternion = quaternion.To<ENU>();
            ROSMsg.header.frame_id = "odom";
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
        }
    }
}
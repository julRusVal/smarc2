using Force;
using ROS.Core;
using RosMessageTypes.Std;
using UnityEngine;

namespace M350.PSDK_ROS2
{
    [AddComponentMenu("Smarc/PSDK_ROS/PsdkAltitude")]
    public class PsdkAltitude : ROSPublisher<Float32Msg>
    {
        MixedBody body;

        protected override void InitPublisher()
        {
            GetMixedBody(out body);
        }
        
        protected override void UpdateMessage()
        {
            ROSMsg.data = body.transform.position.y;
        }
        
    }
}
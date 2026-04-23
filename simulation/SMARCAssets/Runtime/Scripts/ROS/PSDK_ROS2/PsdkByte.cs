using ROS.Core;
using RosMessageTypes.Std;
using UnityEngine;

namespace M350.PSDK_ROS2
{
    [AddComponentMenu("Smarc/PSDK_ROS/PsdkByte")]
    public class PsdkByte : ROSPublisher<UInt8Msg>
    {
        public byte level = 5;
        protected override void UpdateMessage()
        {
            ROSMsg.data = level;
        }
        
    }
}
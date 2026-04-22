using ROS.Core;
using RosMessageTypes.Std;

namespace M350.PSDK_ROS2
{
    [UnityEngine.AddComponentMenu("Smarc/PSDK_ROS/PsdkInt")]
    public class PsdkInt : ROSPublisher<UInt16Msg>
    {
        public byte level = 5;
        protected override void UpdateMessage()
        {
            ROSMsg.data = level;
        }
        
    }
}
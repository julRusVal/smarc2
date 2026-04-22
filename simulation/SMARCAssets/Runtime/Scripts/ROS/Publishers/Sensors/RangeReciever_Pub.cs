using UnityEngine;
using RosMessageTypes.Std;

using RangeReciever = VehicleComponents.Sensors.RangeReciever;
using ROS.Core;

namespace ROS.Publishers
{
    [AddComponentMenu("Smarc/ROS/RangeReciever_Pub")]
    [RequireComponent(typeof(RangeReciever))]
    class RangeReciever_Pub : ROSSensorPublisher<Float32Msg, RangeReciever>
    { 
        protected override void UpdateMessage()
        {
            ROSMsg.data =  DataSource.distance ;
        }
    }
}
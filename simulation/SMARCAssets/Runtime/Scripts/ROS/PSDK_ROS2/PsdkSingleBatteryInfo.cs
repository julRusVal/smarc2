using Unity.Robotics.Core;
using UnityEngine;
using RosMessageTypes.PsdkInterfaces;
using ROS.Core;
using VehicleComponents.Sensors;



namespace M350.PSDK_ROS2
{
    [AddComponentMenu("Smarc/PSDK_ROS/PsdkSingleBatteryInfo")]
    [RequireComponent(typeof(Battery))]
    public class PsdkSingleBatteryInfo : ROSPublisher<SingleBatteryInfoMsg>
    {
        Battery batt;
        protected override void InitPublisher()
        {
            batt = GetComponent<Battery>();
        }

        protected override void UpdateMessage()
        {
            ROSMsg.voltage = batt.currentVoltage;
            ROSMsg.capacity_percentage = batt.currentPercent/100f;
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
        }
    }
}
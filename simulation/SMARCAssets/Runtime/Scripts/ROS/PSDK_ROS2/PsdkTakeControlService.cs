using RosMessageTypes.Std;
using dji;

using ROS.Core;


namespace M350.PSDK_ROS2
{
    [UnityEngine.AddComponentMenu("Smarc/PSDK_ROS/PsdkTakeControlService")]
    public class PsdkTakeControlService : ROSBehaviour
    {
        bool registered = false;
        DJIController controller = null;


        protected override void StartROS()
        {
            if(controller == null){
                controller = GetComponentInParent<DJIController>();
            }
            if (!registered)
            {
                rosCon.ImplementService<TriggerRequest, TriggerResponse>(topic, _take_control_callback);
                registered = true;
            }
        }

        private TriggerResponse _take_control_callback(TriggerRequest request){
            TriggerResponse response = new();

            if(controller == null){
                controller = GetComponentInParent<DJIController>();
            }
            if(controller != null){
                controller.TakeControl();
                response.success = true;
                return response;
            }
            else{
                response.success = false;
                return response;
            }
        }

    }
}
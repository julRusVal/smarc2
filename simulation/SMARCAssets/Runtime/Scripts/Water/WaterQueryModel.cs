using UnityEngine;

namespace DefaultNamespace.Water
{
    public abstract class WaterQueryModel: MonoBehaviour
    {

        public abstract float GetWaterLevelAt(Vector3 position);
        public static WaterQueryModel GetWaterQueryModel()
        {
            var waterModels = FindObjectsByType<WaterQueryModel>(FindObjectsSortMode.None);
            if(waterModels.Length > 0) return waterModels[0];
            else 
            {
                Debug.LogError("WaterQueryModel: No WaterQueryModel found in the scene.");
                return null;
            }
        }
    }
}

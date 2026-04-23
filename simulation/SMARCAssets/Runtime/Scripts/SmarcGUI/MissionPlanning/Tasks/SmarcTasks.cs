using System.Collections.Generic;
using SmarcGUI.MissionPlanning.Params;

namespace SmarcGUI.MissionPlanning.Tasks
{
    public class SmarcStartGeofence : Task
    {
        public override void SetParams()
        {
            Name = "smarc-start-geofence";
            Description = "Start a geofence task";
            Params.Add("waypoints", new List<GeoPoint>());
            Params.Add("ceiling_altitude", -1.0);
            Params.Add("floor_altitude", 1.0);
            Params.Add("stay_inside", true);
        }
    }

    public class SmarcStopGeofence : Task
    {
        public override void SetParams()
        {
            Name = "smarc-stop-geofence";
            Description = "Stop a geofence task";
            Params.Add("reset_geofence", true);
            Params.Add("reset_islands", true);
        }
    }

    public class Z1ProCmd : Task
    {
        public override void SetParams()
        {
            Name = "z1-pro-cmd";
            Description = "Send a command to the Z1 Pro";
            Params.Add("frame", 0);
            Params.Add("roll", 0f);
            Params.Add("pitch", 0f);
            Params.Add("yaw", 0f);
            Params.Add("poi", new GeoPoint());
            Params.Add("channel", 0);
            // Params.Add("resolution", 0);
        }
    }
}

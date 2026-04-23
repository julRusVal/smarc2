using System;
using Newtonsoft.Json;
using SmarcGUI.MissionPlanning.Tasks;


namespace SmarcGUI.Connections
{
    /// <summary>
    /// {"agent-uuid": "7e1de90f-5e73-4460-999e-a38b9eab9479",
    ///  "task-uuid": "53f95311-85ee-4672-8af7-62cc38b370f4",
    ///  "feedback": "No feedback implemented yet.",
    ///  "status": "running"}
    /// </summary>

    [JsonObject(NamingStrategyType = typeof(Newtonsoft.Json.Serialization.KebabCaseNamingStrategy))]
    public class BaseFeedback
    {
        public string AgentUuid;
        public string TaskUuid;
        public string Feedback;
        public string Status;

        public BaseFeedback() { }

        public BaseFeedback(string jsonString)
        {
            JsonConvert.PopulateObject(jsonString, this);
        }

        public override string ToString()
        {
            return $"({Status}): {Feedback}";
        }
    }
}
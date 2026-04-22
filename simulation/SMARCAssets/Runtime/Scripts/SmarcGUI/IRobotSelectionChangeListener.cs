using System.Collections.Generic;

namespace SmarcGUI
{
    public interface IRobotSelectionChangeListener
    {
        public void OnRobotSelectionChange(List<RobotGUI> SelectedRobotGUIs);
    }
}
using UnityEditor;
using UnityEngine;

using Smarc.Rope;

namespace Editor.Scripts
{
    [CustomEditor(typeof(TwoSegmentWinch))]
    public class TwoSegmentWinchEditor : UnityEditor.Editor
    {
        TwoSegmentWinch container;

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            container = (TwoSegmentWinch)target;

            if (GUILayout.Button("ApplySettings"))
            {
                container.ApplySettings();
            }
        }
    }
}
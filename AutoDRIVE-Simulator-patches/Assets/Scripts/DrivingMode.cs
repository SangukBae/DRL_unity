using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DrivingMode : MonoBehaviour
{
    /*
    This script toggles the driving mode of the ego vehicle between Manual and
    Autonomous.
    */

    public VehicleController[] VehicleControllers;
    public AutomobileController[] AutomobileControllers;
    public Text Label;
    private bool Mode;

    private bool IsAutonomousActive()
    {
        for (int i = 0; i < VehicleControllers.Length; i++)
        {
            if (VehicleControllers[i] != null)
            {
                return VehicleControllers[i].DrivingMode == 1;
            }
        }

        for (int i = 0; i < AutomobileControllers.Length; i++)
        {
            if (AutomobileControllers[i] != null)
            {
                return AutomobileControllers[i].DrivingMode == 1;
            }
        }

        return false;
    }

    private void SetLabel()
    {
        if (Label != null)
        {
            Label.text = Mode ? "Manual" : "Autonomous";
        }
    }

    private void Start()
    {
        // Preserve the scene-configured driving mode when this UI is active at startup.
        Mode = !IsAutonomousActive();
        SetLabel();
    }

    public void ToggleDrivingMode()
    {
        Mode = !Mode;
        if(Mode)
        {
            for(int i=0; i<VehicleControllers.Length; i++)
            {
                VehicleControllers[i].DrivingMode = 0;
            }
            for(int i=0; i<AutomobileControllers.Length; i++)
            {
                AutomobileControllers[i].DrivingMode = 0;
            }
            SetLabel();
        }
        else
        {
            for(int i=0; i<VehicleControllers.Length; i++)
            {
                VehicleControllers[i].DrivingMode = 1;
            }
            for(int i=0; i<AutomobileControllers.Length; i++)
            {
                AutomobileControllers[i].DrivingMode = 1;
            }
            SetLabel();
        }
    }
}

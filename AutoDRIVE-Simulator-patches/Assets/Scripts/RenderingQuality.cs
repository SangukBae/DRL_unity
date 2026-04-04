using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RenderingQuality : MonoBehaviour
{
    /*
    This script toggles the rendering quality of the simulator between High Quality
    (i.e. HDRP and post-processing enabled), Medium Quality (i.e. only HDRP enabled)
    and Low Quality (i.e. HDRP and post-processing disabled). The rendering quality
    also affects the terrain details abd the shadows being generated in real-time.
    */

    //public GameObject PostProcessVolume;
    public GameObject HQ_HDRPSettings;
    public GameObject UQ_HDRPSettings;
    public GameObject ReflectionProbe;
    public Light SceneLight;
    public Terrain[] Terrains;
    public GameObject[] LightingObjects;
    public Text Label;

    private int Mode = 0;

    private void SetActiveIfAssigned(GameObject target, bool isActive)
    {
        if (target != null)
        {
            target.SetActive(isActive);
        }
    }

    private void SetLightingObjectsActive(bool isActive)
    {
        if (LightingObjects == null)
        {
            return;
        }

        for (int i = 0; i < LightingObjects.Length; i++)
        {
            if (LightingObjects[i] != null)
            {
                LightingObjects[i].SetActive(isActive);
            }
        }
    }

    private void UpdateTerrainQuality(float detailDensity, int pixelError)
    {
        if (Terrains == null)
        {
            return;
        }

        for (int i = 0; i < Terrains.Length; i++)
        {
            if (Terrains[i] != null)
            {
                Terrains[i].detailObjectDensity = detailDensity;
                Terrains[i].heightmapPixelError = pixelError;
                Terrains[i].Flush();
            }
        }
    }

    private void SetLabelText(string text)
    {
        if (Label != null)
        {
            Label.text = text;
        }
    }

    private void Awake()
    {
        //PostProcessVolume.SetActive(false);
        SetActiveIfAssigned(HQ_HDRPSettings, false);
        SetActiveIfAssigned(UQ_HDRPSettings, false);
        SetActiveIfAssigned(ReflectionProbe, false);
        if (SceneLight != null)
        {
            SceneLight.shadows = LightShadows.None;
        }
        SetLightingObjectsActive(false);
        UpdateTerrainQuality(0.0f, 15);
        Application.targetFrameRate = -1;
        SetLabelText("Low Quality");
    }

    public void ToggleRenderingQuality()
    {
        Mode += 1;
        if(Mode > 2)
        {
            Mode = 0;
        }
        if(Mode == 0)
        {
            //PostProcessVolume.SetActive(false);
            SetActiveIfAssigned(HQ_HDRPSettings, false);
            SetActiveIfAssigned(UQ_HDRPSettings, false);
            SetActiveIfAssigned(ReflectionProbe, false);
            if (SceneLight != null)
            {
                SceneLight.shadows = LightShadows.None;
            }
            SetLightingObjectsActive(false);
            UpdateTerrainQuality(0.0f, 15);
            Application.targetFrameRate = -1;
            SetLabelText("Low Quality");
        }
        else if(Mode == 1)
        {
            //PostProcessVolume.SetActive(false);
            SetActiveIfAssigned(HQ_HDRPSettings, true);
            SetActiveIfAssigned(UQ_HDRPSettings, false);
            SetActiveIfAssigned(ReflectionProbe, false);
            if (SceneLight != null)
            {
                SceneLight.shadows = LightShadows.Hard;
            }
            SetLightingObjectsActive(false);
            UpdateTerrainQuality(0.5f, 10);
            Application.targetFrameRate = -1;
            SetLabelText("High Quality");
        }
        else
        {
            //PostProcessVolume.SetActive(true);
            SetActiveIfAssigned(HQ_HDRPSettings, false);
            SetActiveIfAssigned(UQ_HDRPSettings, true);
            SetActiveIfAssigned(ReflectionProbe, true);
            if (SceneLight != null)
            {
                SceneLight.shadows = LightShadows.Soft;
            }
            SetLightingObjectsActive(true);
            UpdateTerrainQuality(1.0f, 5);
            Application.targetFrameRate = -1;
            SetLabelText("Ultra Quality");
        }
    }
}

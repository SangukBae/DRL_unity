using System;
using UnityEngine;
using UnityEngine.UI;

public class CameraSwitch : MonoBehaviour
{
    /*
    This script switches the cameras sequentially.
    Set DefaultCameraName to the exact name of the camera GameObject
    that should be active when the scene starts (e.g. "Free Camera").
    Leave empty to keep the original behaviour (first in array).
    */

    public GameObject[] Cameras;
    public Text Label;
    public string DefaultCameraName = "Free Camera";

    private int m_CurrentActiveCamera;


    private void Start()
    {
        // Activate the default camera; deactivate all others
        int defaultIndex = 0;
        if (!string.IsNullOrEmpty(DefaultCameraName))
        {
            GameObject defaultCamera = FindCameraByName(DefaultCameraName);
            if (defaultCamera != null)
            {
                int existingIndex = Array.IndexOf(Cameras, defaultCamera);
                if (existingIndex < 0)
                {
                    int emptyIndex = Array.FindIndex(Cameras, camera => camera == null);
                    if (emptyIndex < 0)
                    {
                        Array.Resize(ref Cameras, Cameras.Length + 1);
                        emptyIndex = Cameras.Length - 1;
                    }

                    Cameras[emptyIndex] = defaultCamera;
                    existingIndex = emptyIndex;
                }

                defaultIndex = existingIndex;
            }
        }

        for (int i = 0; i < Cameras.Length; i++)
        {
            if (Cameras[i] != null)
                Cameras[i].SetActive(i == defaultIndex);
        }

        m_CurrentActiveCamera = defaultIndex;
        Label.text = Cameras[m_CurrentActiveCamera].name;
    }

    private GameObject FindCameraByName(string cameraName)
    {
        for (int i = 0; i < Cameras.Length; i++)
        {
            if (Cameras[i] != null && Cameras[i].name == cameraName)
                return Cameras[i];
        }

        Camera[] sceneCameras = FindObjectsOfType<Camera>(true);
        for (int i = 0; i < sceneCameras.Length; i++)
        {
            if (sceneCameras[i] != null && sceneCameras[i].gameObject.name == cameraName)
                return sceneCameras[i].gameObject;
        }

        return null;
    }


    public void NextCamera()
    {
        int nextactiveobject = m_CurrentActiveCamera + 1 >= Cameras.Length ? 0 : m_CurrentActiveCamera + 1;

        for (int i = 0; i < Cameras.Length; i++)
        {
            Cameras[i].SetActive(i == nextactiveobject);
        }

        m_CurrentActiveCamera = nextactiveobject;
        Label.text = Cameras[m_CurrentActiveCamera].name;
    }
}

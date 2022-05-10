/****************************************************
 * File: BoneDebug.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Hybrid Blending Animations
   * Last update: 01/08/2021
*****************************************************/

using UnityEngine;
using System.Collections;

public class BoneDebug : MonoBehaviour
{
    #region Settings

    [Header("Bones Rendering")]
    public bool drawBones = false;
    public bool drawUnitLocalSystem = false;
    public float len = 0.1f;
    public float offset1 = 0.1f; 
    public float offset2 = 0.9f;

    [System.Serializable] public enum ColorBones
    {
        Blue,
        Green,
        Purple
    };
    public ColorBones colorBones;

    #endregion

    [Header("Skip")]
    public GameObject hipConnector;
    public GameObject spineConnector;
    public GameObject spine2Connector;
    public GameObject headConnector;
    public GameObject leftArmConnector;
    public GameObject leftForeArmConnector;
    public GameObject rightArmConnector;
    public GameObject rightForeArmConnector;

    public GameObject groundLeftFront;
    public GameObject groundLeftBack;
    public GameObject groundRightFront;
    public GameObject groundRightBack;

    void drawbone(Transform t)
    {
        //renderer.SetWidth(5.0, 2.0);
        foreach (Transform child in t)
        {
            // Skip this gameObjects
            if((child.gameObject == hipConnector) || (child.gameObject == groundLeftFront) || (child.gameObject == groundLeftBack) || (child.gameObject == groundRightFront) || (child.gameObject == groundRightBack) ||
               (child.gameObject == spineConnector) || (child.gameObject == spine2Connector) || (child.gameObject == headConnector) ||
               (child.gameObject == leftArmConnector) || (child.gameObject == leftForeArmConnector) ||
               (child.gameObject == rightArmConnector) || (child.gameObject == rightForeArmConnector))
            {
                continue;
            }
            else
            {
                Vector3 loxalX = new Vector3(len, 0, 0);
                Vector3 loxalY = new Vector3(0, len, 0);
                Vector3 loxalZ = new Vector3(0, 0, len);
                loxalX = child.rotation * loxalX;
                loxalY = child.rotation * loxalY;
                loxalZ = child.rotation * loxalZ;

                if (colorBones == ColorBones.Blue)
                    Gizmos.color = Color.blue;
                else if (colorBones == ColorBones.Green)
                    Gizmos.color = new Color(0.039f, 0.501f, 0f, 1f); // Instead of Color.green
                else if (colorBones == ColorBones.Purple)
                    Gizmos.color = Color.magenta;

                Gizmos.DrawLine(t.position * offset1 + child.position * offset2, t.position * offset2 + child.position * offset1);

                Gizmos.DrawSphere(t.position, 0.005f);

                // In case we want to draw the unit local vectors
                if (drawUnitLocalSystem)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(child.position, child.position + loxalX);
                    Gizmos.color = Color.green;
                    Gizmos.DrawLine(child.position, child.position + loxalY);
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(child.position, child.position + loxalZ);
                }

                drawbone(child);
            }
        }
    }

    void OnDrawGizmos()
    {
        if(drawBones)
            drawbone(transform);
    }
}
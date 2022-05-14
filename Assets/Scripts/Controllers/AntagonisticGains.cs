/****************************************************
 * File: AntagonisticGains.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class AntagonisticGains : MonoBehaviour
{

    #region Instance Fields

    [Header("Antagonistic Control - Gains INTERACTIVE")]
    public bool manualMode;
    public float expectedMaxMassLeft = 10f;
    public float expectedMaxMassRight = 10f;

    [Header("Antagonistic Control - Left")]
    [Range(0.5f, 8f)] public float stiffnessMultiplierLeft;
    public float realMassLeft;
    public float expectedMassLeft;

    [Header("Antagonistic Control - Right")]
    [Range(0.5f, 8f)] public float stiffnessMultiplierRight;
    public float realMassRight;
    public float expectedMassRight;

    [Header("Antagonistic Control - Settings")]
    public JointController rightHandController;
    public JointController rightForeArmController;
    public JointController rightArmController;
    public JointController leftHandController;
    public JointController leftForeArmController;
    public JointController leftArmController;

    public float handKLX;
    public float handKLY;
    public float handKLZ;

    public float foreArmKLX;
    public float foreArmKLY;
    public float foreArmKLZ;

    public float armKLX;
    public float armKLY;
    public float armKLZ;

    public SafetyRegionLeft safetyRegionLeft;
    public SafetyRegionRight safetyRegionRight;

    #endregion

    #region UI

    //public Slider stiffnessSlider;
    //public Text stiffnessValue;

    #endregion

    #region Unity Methods

    private void Start()
    {
        stiffnessMultiplierLeft = 3f;
        stiffnessMultiplierRight = 3f;

        // TODO: UI Initialize values slider
        //stiffnessSlider.minValue = 0.5f;
        //stiffnessSlider.maxValue = 5f;
    }

    // Update is called once per frame
    void Update()
    {
        // TODO UI
        //stiffnessMultiplier = stiffnessSlider.value;
        //stiffnessValue.text = stiffnessMultiplier.ToString();

        // Retrieve mass
        expectedMassLeft = safetyRegionLeft.targetObstacle.expectedMass;
        expectedMassRight = safetyRegionRight.targetObstacle.expectedMass;
        realMassLeft = safetyRegionLeft.targetObstacle.realMass;
        realMassRight = safetyRegionRight.targetObstacle.realMass;

        if (!manualMode)
        {
            stiffnessMultiplierLeft = Mathf.Lerp(0.5f, 8f, expectedMassLeft/expectedMaxMassLeft);
            stiffnessMultiplierRight = Mathf.Lerp(0.5f, 8f, expectedMassRight/expectedMaxMassRight);

            if (expectedMassLeft == 0f)
                stiffnessMultiplierLeft = 3f;

            if (expectedMassRight == 0f)
                stiffnessMultiplierRight = 3f;
        }

        SetMultipliedStiffness();
    }

    private void SetMultipliedStiffness()
    {
        rightHandController.pLX = handKLX * stiffnessMultiplierRight;
        rightHandController.pLY = handKLY * stiffnessMultiplierRight;
        rightHandController.pLZ = handKLZ * stiffnessMultiplierRight;

        rightForeArmController.pLX = foreArmKLX * stiffnessMultiplierRight;
        rightForeArmController.pLY = foreArmKLY * stiffnessMultiplierRight;
        rightForeArmController.pLZ = foreArmKLZ * stiffnessMultiplierRight;

        rightArmController.pLX = armKLX * stiffnessMultiplierRight;
        rightArmController.pLY = armKLY * stiffnessMultiplierRight;
        rightArmController.pLZ = armKLZ * stiffnessMultiplierRight;

        leftHandController.pLX = handKLX * stiffnessMultiplierLeft;
        leftHandController.pLY = handKLY * stiffnessMultiplierLeft;
        leftHandController.pLZ = handKLZ * stiffnessMultiplierLeft;

        leftForeArmController.pLX = foreArmKLX * stiffnessMultiplierLeft;
        leftForeArmController.pLY = foreArmKLY * stiffnessMultiplierLeft;
        leftForeArmController.pLZ = foreArmKLZ * stiffnessMultiplierLeft;

        leftArmController.pLX = armKLX * stiffnessMultiplierLeft;
        leftArmController.pLY = armKLX * stiffnessMultiplierLeft;
        leftArmController.pLZ = armKLX * stiffnessMultiplierLeft;
    }

    #endregion
}

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

    public JointController rightHandController;
    public JointController rightForeArmController;
    public JointController rightArmController;
    public JointController leftHandController;
    public JointController leftForeArmController;
    public JointController leftArmController;

    [Range(0.5f, 5f)] public float stiffnessMultiplier;

    public float handKLX;
    public float handKLY;
    public float handKLZ;

    public float foreArmKLX;
    public float foreArmKLY;
    public float foreArmKLZ;

    public float armKLX;
    public float armKLY;
    public float armKLZ;

    #endregion

    #region UI

    //public Slider stiffnessSlider;
    //public Text stiffnessValue;

    #endregion

    #region Unity Methods

    private void Start()
    {
        // TODO: Initialize values slider
        //stiffnessSlider.minValue = 0.5f;
        //stiffnessSlider.maxValue = 5f;
    }

    // Update is called once per frame
    void Update()
    {
        // TODO
        //stiffnessMultiplier = stiffnessSlider.value;
        //stiffnessValue.text = stiffnessMultiplier.ToString();

        rightHandController.pLX = handKLX * stiffnessMultiplier;
        rightHandController.pLY = handKLY * stiffnessMultiplier;
        rightHandController.pLZ = handKLZ * stiffnessMultiplier;

        rightForeArmController.pLX = foreArmKLX * stiffnessMultiplier;
        rightForeArmController.pLY = foreArmKLY * stiffnessMultiplier;
        rightForeArmController.pLZ = foreArmKLZ * stiffnessMultiplier;

        rightArmController.pLX = armKLX * stiffnessMultiplier;
        rightArmController.pLY = armKLY * stiffnessMultiplier;
        rightArmController.pLZ = armKLZ * stiffnessMultiplier;

        leftHandController.pLX = handKLX * stiffnessMultiplier;
        leftHandController.pLY = handKLY * stiffnessMultiplier;
        leftHandController.pLZ = handKLZ * stiffnessMultiplier;

        leftForeArmController.pLX = foreArmKLX * stiffnessMultiplier;
        leftForeArmController.pLY = foreArmKLY * stiffnessMultiplier;
        leftForeArmController.pLZ = foreArmKLZ * stiffnessMultiplier;

        leftArmController.pLX = armKLX * stiffnessMultiplier;
        leftArmController.pLY = armKLX * stiffnessMultiplier;
        leftArmController.pLZ = armKLX * stiffnessMultiplier;
    }

    #endregion
}

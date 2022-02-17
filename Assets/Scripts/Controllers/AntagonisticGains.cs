using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticGains : MonoBehaviour
{

    #region Instance Fields

    public JointController rightHandController;
    public JointController rightForeArmController;
    public JointController rightArmController;
    public JointController leftHandController;
    public JointController leftForeArmController;
    public JointController leftArmController;

    //public float rightHandKLX;
    //public float rightHandKLY;
    //public float rightHandKLZ;

    //public float rightForeArmKLX;
    //public float rightForeArmKLY;
    //public float rightForeArmKLZ;

    //public float rightArmKLX;
    //public float rightArmKLY;
    //public float rightArmKLZ;

    //public float leftHandKLX;
    //public float leftHandKLY;
    //public float leftHandKLZ;

    //public float leftForeArmKLX;
    //public float leftForeArmKLY;
    //public float leftForeArmKLZ;

    //public float leftArmKLX;
    //public float leftArmKLY;
    //public float leftArmKLZ;

    [Range(1f, 2.6f)]
    public float stiffnessMultiplier;

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

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //rightHandController.pLX = rightHandKLX;
        //rightHandController.pLY = rightHandKLY;
        //rightHandController.pLZ = rightHandKLZ;

        //rightForeArmController.pLX = rightForeArmKLX;
        //rightForeArmController.pLY = rightForeArmKLY;
        //rightForeArmController.pLZ = rightForeArmKLZ;

        //rightArmController.pLX = rightArmKLX;
        //rightArmController.pLY = rightArmKLY;
        //rightArmController.pLZ = rightArmKLZ;

        //leftHandController.pLX = leftHandKLX;
        //leftHandController.pLY = leftHandKLY;
        //leftHandController.pLZ = leftHandKLZ;

        //leftForeArmController.pLX = leftForeArmKLX;
        //leftForeArmController.pLY = leftForeArmKLY;
        //leftForeArmController.pLZ = leftForeArmKLZ;

        //leftArmController.pLX = leftArmKLX;
        //leftArmController.pLY = leftArmKLY;
        //leftArmController.pLZ = leftArmKLZ;

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
}

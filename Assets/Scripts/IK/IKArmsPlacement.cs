/****************************************************
 * File: IKArmsPlacement.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

using UnityEngine;
using System;
using System.Collections;

[RequireComponent(typeof(Animator))]

public class IKArmsPlacement : MonoBehaviour
{
    #region Read-only & Static Fields

    protected Animator animator;

    #endregion

    #region Instance Fields

    [Header("Arms Placement - Options")]
    public bool enableArmsIK = false;
    [Range(0f, 1f)] public float rightArmWeight;
    [Range(0f, 1f)] public float leftArmWeight;
    public Vector3 rotationOffsetRight;
    public Vector3 rotationOffsetLeft;

    [Header("Head Placement - Options")]
    [Range(0f, 1f)] public float headWeight;

    public TargetIK leftTarget;
    public TargetIK rightTarget;

    public bool activateIKLeft;
    public bool activateIKRight;

    // TEST
    public SafetyRegionLeft safetyRegionLeft;

    #endregion

    #region Unity Methods

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    private void Update()
    {
        activateIKLeft = leftTarget.activateIK;
        activateIKRight = rightTarget.activateIK;
    }

    void OnAnimatorIK()
    {
        if (animator)
        {
            // If the IK is active, set the position and rotation directly to the target 
            if (enableArmsIK)
            {
                if(safetyRegionLeft.obstacles.Count != 0)
                {
                    animator.SetLookAtWeight(headWeight);
                    animator.SetLookAtPosition(safetyRegionLeft.targetObstacle.location);
                }
                else
                {
                    animator.SetLookAtWeight(0f);
                }


                // Set the look target position, if one has been assigned, just the first it encounters
                if (rightTarget.target.position != null)
                {
                    //animator.SetLookAtWeight(headWeight);
                    //animator.SetLookAtPosition(rightTarget.target.position);
                }
                else if(leftTarget.target.position != null)
                {
                    //animator.SetLookAtWeight(headWeight);
                    //animator.SetLookAtPosition(safetyRegionLeft.targetObstacle.location);
                }

                // Set the right hand target position and rotation, if one has been assigned
                if (rightTarget.target != null)
                {
                    animator.SetIKPositionWeight(AvatarIKGoal.RightHand, rightArmWeight);
                    animator.SetIKRotationWeight(AvatarIKGoal.RightHand, rightArmWeight);
                    animator.SetIKPosition(AvatarIKGoal.RightHand, rightTarget.target.position); 
                    animator.SetIKRotation(AvatarIKGoal.RightHand, rightTarget.target.rotation * Quaternion.Euler(new Vector3(rotationOffsetRight.x, rotationOffsetRight.y, rotationOffsetRight.z))); // TEST
                }

                // Set the right hand target position and rotation, if one has been assigned
                if (leftTarget.target != null)
                {
                    animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, leftArmWeight);
                    animator.SetIKRotationWeight(AvatarIKGoal.LeftHand, leftArmWeight);
                    animator.SetIKPosition(AvatarIKGoal.LeftHand, leftTarget.target.position);
                    animator.SetIKRotation(AvatarIKGoal.LeftHand, leftTarget.target.rotation * Quaternion.Euler(new Vector3(rotationOffsetLeft.x, rotationOffsetLeft.y, rotationOffsetLeft.z))); // TEST
                }
            }
            // If the IK is not active, set the position and rotation of the hand and head back to the original position
            else
            {
                animator.SetIKPositionWeight(AvatarIKGoal.RightHand, 0);
                animator.SetIKRotationWeight(AvatarIKGoal.RightHand, 0);
                animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, 0);
                animator.SetIKRotationWeight(AvatarIKGoal.LeftHand, 0);
                animator.SetLookAtWeight(0);
            }
        }
    }

    #endregion
}

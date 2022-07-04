/****************************************************
 * File: IKArmsPlacement.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: Generating Upper-Body Motion for Real-Time Characters
   * Last update: 30/06/2022
*****************************************************/

// TODO: CHECK THIS SCRIPT WHEN WE HAVE THE WHOLE METHOD

using UnityEngine;
using System;
using System.Collections;

[RequireComponent(typeof(Animator))]

public class IKArmsPlacement : MonoBehaviour
{
    #region Read-only & Static Fields

    protected Animator _anim;

    #endregion

    #region Instance Fields

    [Header("Arms Placement - Basic")]
    public bool enableArmsIK = false;

    [Header("Arms Placement - Options")]
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
    public bool alwaysZero;

    #endregion

    #region Unity Methods

    void Start()
    {
        // Getting components from Inspector
        _anim = GetComponent<Animator>();
    }

    private void Update()
    {
        activateIKLeft = leftTarget.activateIK;
        activateIKRight = rightTarget.activateIK;
    }

    void OnAnimatorIK()
    {
        if (_anim)
        {
            // If the IK is active, set the position and rotation directly to the target 
            if (enableArmsIK)
            {
                if (safetyRegionLeft.obstacles.Count != 0)
                {
                    //animator.SetLookAtWeight(headWeight);
                    //StartCoroutine(Lerp());
                    _anim.SetLookAtWeight(headWeight);

                    _anim.SetLookAtPosition(safetyRegionLeft.targetObstacle.location);
                }
                else
                {
                    _anim.SetLookAtWeight(0f);
                }

                // Set the look target position, if one has been assigned, just the first it encounters
                //if (rightTarget.target.position != null)
                //{
                //    animator.SetLookAtWeight(headWeight);
                //    animator.SetLookAtPosition(rightTarget.target.position);
                //}
                //else if(leftTarget.target.position != null)
                //{
                //    animator.SetLookAtWeight(headWeight);
                //    animator.SetLookAtPosition(safetyRegionLeft.targetObstacle.location);
                //}

                // Set the right hand target position and rotation, if one has been assigned
                if (rightTarget.target != null)
                {
                    if(!alwaysZero)
                    {
                        _anim.SetIKPositionWeight(AvatarIKGoal.RightHand, rightArmWeight);
                        _anim.SetIKRotationWeight(AvatarIKGoal.RightHand, rightArmWeight);
                        _anim.SetIKPosition(AvatarIKGoal.RightHand, rightTarget.target.position);
                        _anim.SetIKRotation(AvatarIKGoal.RightHand, rightTarget.target.rotation * Quaternion.Euler(new Vector3(rotationOffsetRight.x, rotationOffsetRight.y, rotationOffsetRight.z))); // TEST
                    }
                    else
                    {
                        _anim.SetIKPositionWeight(AvatarIKGoal.RightHand, 0f);
                        _anim.SetIKRotationWeight(AvatarIKGoal.RightHand, 0f);
                        _anim.SetIKPosition(AvatarIKGoal.RightHand, rightTarget.target.position);
                        _anim.SetIKRotation(AvatarIKGoal.RightHand, rightTarget.target.rotation * Quaternion.Euler(new Vector3(rotationOffsetRight.x, rotationOffsetRight.y, rotationOffsetRight.z))); // TEST
                    }

                }

                // Set the right hand target position and rotation, if one has been assigned
                if (leftTarget.target != null)
                {
                    if(!alwaysZero)
                    {
                        _anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, leftArmWeight);
                        _anim.SetIKRotationWeight(AvatarIKGoal.LeftHand, leftArmWeight);
                        _anim.SetIKPosition(AvatarIKGoal.LeftHand, leftTarget.target.position);
                        _anim.SetIKRotation(AvatarIKGoal.LeftHand, leftTarget.target.rotation * Quaternion.Euler(new Vector3(rotationOffsetLeft.x, rotationOffsetLeft.y, rotationOffsetLeft.z))); // TEST
                    }
                    else
                    {
                        _anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, 0f);
                        _anim.SetIKRotationWeight(AvatarIKGoal.LeftHand, 0f);
                        _anim.SetIKPosition(AvatarIKGoal.LeftHand, leftTarget.target.position);
                        _anim.SetIKRotation(AvatarIKGoal.LeftHand, leftTarget.target.rotation * Quaternion.Euler(new Vector3(rotationOffsetLeft.x, rotationOffsetLeft.y, rotationOffsetLeft.z))); // TEST
                    }

                }
            }
            // If the IK is not active, set the position and rotation of the hand and head back to the original position
            else
            {
                _anim.SetIKPositionWeight(AvatarIKGoal.RightHand, 0);
                _anim.SetIKRotationWeight(AvatarIKGoal.RightHand, 0);
                _anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, 0);
                _anim.SetIKRotationWeight(AvatarIKGoal.LeftHand, 0);
                _anim.SetLookAtWeight(0);
            }
        }
    }

    #endregion
}

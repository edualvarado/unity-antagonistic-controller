/****************************************************
 * File: IKFeetPlacement.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Animation and beyond
   * Last update: 04/07/2022
*****************************************************/

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

public class IKFeetPlacement : MonoBehaviour
{
    #region Read-only & Static Fields

    private Animator _anim;
    private Quaternion leftFootIKRotation, rightFootIKRotation;
    private float lastPelvisPositionY, lastRightFootPositionY, lastLeftFootPositionY;
    private IEnumerator instLerpRight;
    private IEnumerator instLerpLeft;

    #endregion

    #region Instance Fields

    [Header("Feet Grounder - Basics")]
    public bool enableFeetIK = true;
    public bool enableAdaptiveWeight = false; // When using Motion Matching

    [Header("Feet Grounder - Options")]
    [SerializeField] private LayerMask environmentLayer;
    [Range(0, 20f)] [SerializeField] private float heightFromGroundRaycast = 0.2f;
    [Range(0, 20f)] [SerializeField] private float raycastDownDistance = 1.0f;
    [SerializeField] private float pelvisOffset = 0f;
    [Range(0, 1f)] [SerializeField] private float pelvisUpAndDownSpeed = 0.3f;
    [Range(0, 1f)] [SerializeField] private float feetToIKPositionSpeed = 0.2f;
    public float heightLeftFootOffset = 0f;
    public float heightRightFootOffset = 0f;

    [Header("Feet Grounder - Debug")]
    public Vector3 rightFootPosition;
    public Vector3 leftFootPosition;
    public Vector3 rightFootIKPosition;
    public Vector3 leftFootIKPosition;

    [Header("Individual Feet Grounder - Options")]
    public Transform groundCheckerLeftFootBack;
    public Transform groundCheckerLeftFootFront;
    public Transform groundCheckerRightFootBack;
    public Transform groundCheckerRightFootFront;
    public float feetToGroundDistance = 0.1f;

    [Header("Individual Feet Grounder - Debug")]
    public bool drawSensorRayGrounder = false;
    public bool isLeftFootGrounded = false;
    public bool isRightFootGrounded = false;

    [Header("Other Settings")]
    public string leftFootAnimVariableName = "LeftFootCurve";
    public string rightFootAnimVariableName = "RightFootCurve";
    public bool useProIKFeature = true;
    public bool showSolverDebug = true;

    [Header("Procedural Weight with LERP - Debug")]
    public float weightToChangeRightFoot = 0f;
    public float durationWeightRightFoot = 0.1f;
    public bool weightDecreasingRightFoot = false;
    public bool weightIncreasingRightFoot = false;
    public float weightToChangeLeftFoot = 0f;
    public float durationWeightLeftFoot = 0.1f;
    public bool weightDecreasingLeftFoot = false;
    public bool weightIncreasingLeftFoot = false;
    #endregion

    #region Instance Properties

    public Vector3 RightFootPosition
    {
        get { return rightFootPosition; }
        set { rightFootPosition = value; }
    }
    public Vector3 LeftFootPosition
    {
        get { return leftFootPosition; }
        set { leftFootPosition = value; }
    }
    public Vector3 RightFootIKPosition
    {
        get { return rightFootIKPosition; }
        set { rightFootIKPosition = value; }
    }
    public Vector3 LeftFootIKPosition
    {
        get { return leftFootIKPosition; }
        set { leftFootIKPosition = value; }
    }

    #endregion

    #region Unity Methods
    
    void Start()
    {
        // Getting components from Inspector
        _anim = GetComponent<Animator>();

        // Set Environment Layer
        environmentLayer = LayerMask.GetMask("Ground");
    }

    /// <summary>
    /// Update the AdjustFeetTarget method and also find the position of each foot inside our Solver Position.
    /// </summary>
    private void FixedUpdate()
    {
        if (enableFeetIK == false) { return; }
        if (_anim == null) { return; }

        AdjustFeetTarget(ref rightFootPosition, HumanBodyBones.RightFoot);
        AdjustFeetTarget(ref leftFootPosition, HumanBodyBones.LeftFoot);

        // Find a raycast to the ground to find positions
        FeetPositionSolver(rightFootPosition, ref rightFootIKPosition, ref rightFootIKRotation, 0); // Handle the solver for right foot. 0 for left
        FeetPositionSolver(leftFootPosition, ref leftFootIKPosition, ref leftFootIKRotation, 1); // Handle the solver for left foot. 1 for right

        // Check if each foot is grounded
        CheckFeetAreGrounded();

        // IMPORTANT!
        // The pivot is the most stable point between the left and right foot of the avatar. For a value of 0, the left foot is the most stable point. For a value of 1, the right foot is the most stable point
        //Debug.Log("Pivot Weight: " + anim.pivotWeight);
        //Debug.Log("Pivot Position: " + this.transform.InverseTransformPoint(anim.pivotPosition));
    }

    /// <summary>
    /// Called when IK Pass is activated.
    /// </summary>
    /// <param name="layerIndex"></param>
    private void OnAnimatorIK(int layerIndex)
    {
        if (_anim == null) { return; }

        MovePelvisHeight();

        // RIGHT FOOT

        // RightFoot IK Position  - Max. IK position
        _anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);

        // RightFoot IK Rotation  - for PRO feature
        if (useProIKFeature)
        {
            if (!enableAdaptiveWeight)
            {
                _anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, _anim.GetFloat(rightFootAnimVariableName));
            }
            else
            {
                // When back foot touches, then we procedurally increase weight to 1
                // When back is not touching anymore, we start decreasing to 0

                if (Physics.CheckSphere(groundCheckerRightFootFront.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore) && !weightIncreasingRightFoot)
                {
                    if (weightIncreasingRightFoot || weightDecreasingRightFoot)
                    {
                        Debug.Log("STOPPING COROUTINE");
                        StopCoroutine(instLerpRight);
                    }

                    Debug.Log("Right Front is touching now - START INCREASING");
                    instLerpRight = LerpFunctionRight(1, durationWeightRightFoot);
                    StartCoroutine(instLerpRight);

                    weightDecreasingRightFoot = false;
                    weightIncreasingRightFoot = true;

                }
                else if (!Physics.CheckSphere(groundCheckerRightFootFront.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore) && !weightDecreasingRightFoot)
                {
                    if (weightIncreasingRightFoot || weightDecreasingRightFoot)
                    {
                        Debug.Log("STOPPING COROUTINE");
                        StopCoroutine(instLerpRight);
                    }

                    Debug.Log("Right Front NOT touching now - START DECREASING");
                    instLerpRight = LerpFunctionRight(0, durationWeightRightFoot);
                    StartCoroutine(instLerpRight);

                    weightDecreasingRightFoot = true;
                    weightIncreasingRightFoot = false;
                }

                Debug.Log("VALUE TO CHANGE RIGHT: " + weightToChangeRightFoot);
                _anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, weightToChangeRightFoot);
            }
        }

        // Move RightFoot to the target IK position and rotation
        MoveFeetToIKPoint(AvatarIKGoal.RightFoot, rightFootIKPosition, rightFootIKRotation, ref lastRightFootPositionY);

        // LEFT FOOT

        // LeftFoot IK Position  - Max. IK position
        _anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);

        // LeftFoot IK Rotation  - for PRO feature
        if (useProIKFeature)
        {
            if (!enableAdaptiveWeight)
            {
                _anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, _anim.GetFloat(leftFootAnimVariableName));
            }
            else
            {
                // When back foot touches, then we procedurally increase weight to 1
                // When back is not touching anymore, we start decreasing to 0

                if (Physics.CheckSphere(groundCheckerLeftFootFront.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore) && !weightIncreasingLeftFoot)
                {
                    if (weightIncreasingLeftFoot || weightDecreasingLeftFoot)
                    {
                        Debug.Log("STOPPING COROUTINE");
                        StopCoroutine(instLerpLeft);
                    }

                    Debug.Log("Left Front is touching now - START INCREASING");
                    instLerpLeft = LerpFunctionLeft(1, durationWeightLeftFoot);
                    StartCoroutine(instLerpLeft);

                    weightDecreasingLeftFoot = false;
                    weightIncreasingLeftFoot = true;

                }
                else if (!Physics.CheckSphere(groundCheckerLeftFootFront.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore) && !weightDecreasingLeftFoot)
                {
                    if (weightIncreasingLeftFoot || weightDecreasingLeftFoot)
                    {
                        Debug.Log("STOPPING COROUTINE");
                        StopCoroutine(instLerpLeft);
                    }

                    Debug.Log("Left Front NOT touching now - START DECREASING");
                    instLerpLeft = LerpFunctionLeft(0, durationWeightLeftFoot);
                    StartCoroutine(instLerpLeft);

                    weightDecreasingLeftFoot = true;
                    weightIncreasingLeftFoot = false;
                }

                Debug.Log("VALUE TO CHANGE LEFT: " + weightToChangeLeftFoot);
                _anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, weightToChangeLeftFoot);
            }
        }

        // Move LeftFoot to the target IK position and rotation
        MoveFeetToIKPoint(AvatarIKGoal.LeftFoot, leftFootIKPosition, leftFootIKRotation, ref lastLeftFootPositionY);
    }

    #endregion

    #region FeetGrounding

    private void CheckFeetAreGrounded()
    {

        if (Physics.CheckSphere(groundCheckerLeftFootFront.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore) || Physics.CheckSphere(groundCheckerLeftFootBack.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore))
        {
            isLeftFootGrounded = true;
        }
        else
        {
            isLeftFootGrounded = false;
        }

        if (Physics.CheckSphere(groundCheckerRightFootFront.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore) || Physics.CheckSphere(groundCheckerRightFootBack.position, feetToGroundDistance, environmentLayer, QueryTriggerInteraction.Ignore))
        {
            isRightFootGrounded = true;
        }
        else
        {
            isRightFootGrounded = false;
        }

        if (drawSensorRayGrounder)
        {
            Debug.DrawRay(groundCheckerLeftFootFront.position, -Vector3.up * feetToGroundDistance, Color.cyan);
            Debug.DrawRay(groundCheckerLeftFootBack.position, -Vector3.up * feetToGroundDistance, Color.cyan);
            Debug.DrawRay(groundCheckerRightFootFront.position, -Vector3.up * feetToGroundDistance, Color.cyan);
            Debug.DrawRay(groundCheckerRightFootBack.position, -Vector3.up * feetToGroundDistance, Color.cyan);
        }
    }

    #endregion

    #region FeetGroundingMethods

    /// <summary>
    /// Move feet to the IK target.
    /// </summary>
    /// <param name="foot"></param>
    /// <param name="positionIKHolder"></param>
    /// <param name="rotationIKHolder"></param>
    /// <param name="lastFootPositionY"></param>
    void MoveFeetToIKPoint(AvatarIKGoal foot, Vector3 positionIKHolder, Quaternion rotationIKHolder, ref float lastFootPositionY)
    {
        //  Get the current position of the foot, which we are going to move
        Vector3 targetIKPosition = _anim.GetIKPosition(foot);

        // If there is a IK target in a different position (not 0 locally) than the position where we have our foot currently
        if (positionIKHolder != Vector3.zero)
        {
            // Convert the world coordinates for current/target foot positions to local coordinates with respect to the character
            targetIKPosition = transform.InverseTransformPoint(targetIKPosition);
            positionIKHolder = transform.InverseTransformPoint(positionIKHolder);

            // Calculate the translation in Y necessary to move the last foot position to the target position, by a particular speed
            float yVariable = Mathf.Lerp(lastFootPositionY, positionIKHolder.y, feetToIKPositionSpeed);

            // Add this desired translation in Y to our current feet position
            targetIKPosition.y += yVariable;

            // We update the last foot position in Y
            lastFootPositionY = yVariable;

            // Convert the current foot position to world coordinates
            targetIKPosition = transform.TransformPoint(targetIKPosition);

            // Set the new goal rotation (world coordinates) for the foot
            _anim.SetIKRotation(foot, rotationIKHolder);
        }

        // Set the new goal position (world coordinates) for the foot
        _anim.SetIKPosition(foot, targetIKPosition);
    }

    /// <summary>
    /// Adapt height of pelvis - TODO: REVIEW
    /// </summary>
    private void MovePelvisHeight()
    {
        if (rightFootIKPosition == Vector3.zero || leftFootIKPosition == Vector3.zero || lastPelvisPositionY == 0)
        {
            lastPelvisPositionY = _anim.bodyPosition.y;
            return;
        }

        float leftOffsetPosition = leftFootIKPosition.y - transform.position.y;
        float rightOffsetPosition = rightFootIKPosition.y - transform.position.y;

        float totalOffset = (leftOffsetPosition < rightOffsetPosition) ? leftOffsetPosition : rightOffsetPosition;

        // Hold new pelvis position where we want to move to
        // Move from last to new position with certain speed
        Vector3 newPelvisPosition = _anim.bodyPosition + Vector3.up * totalOffset;
        newPelvisPosition.y = Mathf.Lerp(lastPelvisPositionY, newPelvisPosition.y, pelvisUpAndDownSpeed);

        // Update current body position
        _anim.bodyPosition = newPelvisPosition;

        // Now the last known pelvis position in Y is the current body position in Y
        lastPelvisPositionY = _anim.bodyPosition.y;
    }

    /// <summary>
    /// Locate the feet position via a raycast and then solving.
    /// </summary>
    /// <param name="fromSkyPosition"></param>
    /// <param name="feetIKPositions"></param>
    /// <param name="feetIKRotations"></param>
    private void FeetPositionSolver(Vector3 fromSkyPosition, ref Vector3 feetIKPositions, ref Quaternion feetIKRotations, int foot)
    {
        // To store all the info regarding the hit of the ray
        RaycastHit feetoutHit;

        // To visualize the ray
        if (showSolverDebug)
        {
            Debug.DrawLine(fromSkyPosition, fromSkyPosition + Vector3.down * (raycastDownDistance + heightFromGroundRaycast), Color.yellow);
        }

        // If the ray, starting at the sky position, goes down certain distance and hits an environment layer
        if (Physics.Raycast(fromSkyPosition, Vector3.down, out feetoutHit, raycastDownDistance + heightFromGroundRaycast, environmentLayer))
        {
            // Position the new IK feet positions parallel to the sky position, and put them where the ray intersects with the environment layer
            feetIKPositions = fromSkyPosition;

            if (foot == 1)
                feetIKPositions.y = feetoutHit.point.y + pelvisOffset + heightLeftFootOffset;
            else if (foot == 0)
                feetIKPositions.y = feetoutHit.point.y + pelvisOffset + heightRightFootOffset;

            // Creates a rotation from the (0,1,0) to the normal of where the feet is placed it in the terrain
            feetIKRotations = Quaternion.FromToRotation(Vector3.up, feetoutHit.normal) * transform.rotation;

            return;
        }

        feetIKPositions = Vector3.zero; // If we reach this, it didn't work
    }

    /// <summary>
    /// Adjust the IK target for the feet.
    /// </summary>
    /// <param name="feetPositions"></param>
    /// <param name="foot"></param>
    private void AdjustFeetTarget(ref Vector3 feetPositions, HumanBodyBones foot)
    {
        // Takes the Vector3 transform of that human bone id
        feetPositions = _anim.GetBoneTransform(foot).position;
        feetPositions.y = transform.position.y + heightFromGroundRaycast;
    }

    #endregion

    #region Coroutine Foot Procedural Weight

    IEnumerator LerpFunctionRight(float endValue, float duration)
    {
        float time = 0;
        float startValue = weightToChangeRightFoot;
        while (time < duration)
        {
            weightToChangeRightFoot = Mathf.Lerp(startValue, endValue, time / duration);
            time += Time.deltaTime;
            yield return null;
        }
        weightToChangeRightFoot = endValue;
    }

    IEnumerator LerpFunctionLeft(float endValue, float duration)
    {
        float time = 0;
        float startValue = weightToChangeLeftFoot;
        while (time < duration)
        {
            weightToChangeLeftFoot = Mathf.Lerp(startValue, endValue, time / duration);
            time += Time.deltaTime;
            yield return null;
        }
        weightToChangeLeftFoot = endValue;
    }

    #endregion
}

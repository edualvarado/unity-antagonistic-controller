/****************************************************
 * File: SafetyRegionLeft.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 18/02/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegionLeft : SafetyRegion
{
    #region Instance Fields

    [Header("Left Hand - IK")]
    public ArmsFastIK leftHandIK;
    public bool drawRays;

    [Header("Left Hand - Hit")]
    public Vector3 hitOffsetLeft;
    public Vector3 hitLeftFixed;
    public Vector3 hitLeft;
    public Vector3 hitNormalLeft;
    public Vector3 raycastOriginLeft;

    [Header("Left Hand - Debug")]
    public bool hasLeftStartedMovingIn;
    public bool hasLeftTargetReached;
    public bool hasLeftStartedMovingOut;
    public bool isLeftInRange = false;

    #endregion

    #region Read-only & Static Fields

    private SphereCollider sphereColliderLeft;
    private Transform leftTargetTransform;
    private BoxCollider leftHandKinematic;
    public bool isKinematicLeftHandTouching;

    #endregion

    void Start()
    {
        sphereColliderLeft = GetComponent<SphereCollider>();
        leftTargetTransform = GameObject.Find("Target Left Hand").GetComponent<Transform>();
        leftHandKinematic = leftHandIK.gameObject.GetComponent<BoxCollider>();
    }

    void Update()
    {
        sphereColliderLeft.radius = radiusRegion;
        sphereColliderLeft.center = transform.InverseTransformPoint(originRegion.position) + originOffset;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle ENTERS LEFT");

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginLeft = originRegion.position;

            // Create an offset, in case it is necessary
            Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);

            // Fix the first contact position
            hitLeftFixed = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation) + offset;

            // Start moving to the target
            hasLeftStartedMovingIn = true;
            hasLeftStartedMovingOut = false;
            hasLeftTargetReached = false;
            Debug.Log("[INFO] hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);
            // hasLeftStartedMovingIn: TRUE, hasLeftStartedMovingOut: FALSE, hasLeftTargeReached: FALSE
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle STAYS LEFT");

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginLeft = originRegion.position;

            // Keep track of the contact position, without updating the fixed one
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);

            // 1. We update if we did not reach yet the position
            // 2. Or if we stop being in contact when the distance to the hit is larger than the arm itself
            if (!hasLeftTargetReached)
            {
                hitLeftFixed = hitLeft;
            }
            else
            {
                hitLeftFixed = hitLeft; // TEST

                // TODO: One for static objects, and other for dynamic?

                //if(isKinematicLeftHandTouching)
                //{
                //    hitLeftFixed = hitLeft;
                //}

                if (!isLeftInRange)
                {
                    Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);
                    leftHandIK.SetTargetUpdate(hitLeftFixed, offset, 0.5f); // TODO: Time that takes to make the small jump
                }
            }

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray)
            if (Physics.Raycast(raycastOriginLeft, (hitLeftFixed - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                hitNormalLeft = hit.normal;

                if (drawRays)
                {
                    Debug.DrawRay(raycastOriginLeft, (hitLeftFixed - originRegion.position), Color.blue);
                    Debug.DrawRay(hitLeftFixed, hitNormalLeft * 0.2f, Color.cyan);
                }

                // Set target where it his, based on if reacting or just placing the hand
                // hasLeftStartedMovingIn will be TRUE until we reach the object
                leftHandIK.SetTargetStay(reactionTime, hasLeftStartedMovingIn);
            }

            // Activate - TODO: Have weights would be great to decide the amount of IK
            leftHandIK.activateIK = true;

            // Once we have finished, we update the variables
            hasLeftStartedMovingOut = false;
            if (hasLeftTargetReached)
                hasLeftStartedMovingIn = false;            
            Debug.Log("[INFO] hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);
            // hasLeftStartedMovingIn: FALSE, hasLeftStartedMovingOut: FALSE, hasLeftTargeReached: FALSE -> TRUE (when the coroutine finishes)
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle EXITS LEFT");

            // Starts moving out
            hasLeftStartedMovingIn = false;
            hasLeftStartedMovingOut = true;
            hasLeftTargetReached = false;
            Debug.Log("[INFO] hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);
            // hasLeftStartedMovingIn: FALSE, hasLeftStartedMovingOut: TRUE, hasLeftTargeReached: FALSE

            // Set target back to original position - TODO: FIX the original position
            leftHandIK.SetTargetBack(reactionTime, hasLeftStartedMovingOut);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(hitLeftFixed, 0.05f);
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(hitLeft, 0.05f);
    }
}

/****************************************************
 * File: SafetyRegionRight.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 18/02/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegionRight : SafetyRegion
{
    #region Instance Fields

    [Header("Right Hand - IK")]
    public ArmsFastIK rightHandIK;
    public bool drawRays;

    [Header("Right Hand - Hit")]
    public Vector3 hitOffsetRight;
    public Vector3 hitRightFixed;
    public Vector3 hitRight;
    public Vector3 hitNormalRight;
    public Vector3 raycastOriginRight;

    [Header("Right Hand - Debug")]
    public bool hasRightStartedMovingIn;
    public bool hasRightTargetReached;
    public bool hasRightStartedMovingOut;
    public bool isRightInRange = false;

    #endregion

    #region Read-only & Static Fields

    private SphereCollider sphereColliderRight;
    private Transform rightTargetTransform;
    private BoxCollider rightHandKinematic;
    public bool isKinematicRightHandTouching;

    #endregion

    void Start()
    {
        sphereColliderRight = GetComponent<SphereCollider>();
        rightTargetTransform = GameObject.Find("Target Right Hand").GetComponent<Transform>();
        rightHandKinematic = rightHandIK.gameObject.GetComponent<BoxCollider>();
    }

    void Update()
    {
        sphereColliderRight.radius = radiusRegion;
        sphereColliderRight.center = transform.InverseTransformPoint(originRegion.position) + originOffset;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle ENTERS RIGHT");

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginRight = originRegion.position;

            // Create an offset, in case it is necessary
            Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);

            // Fix the first contact position
            hitRightFixed = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation) + offset;

            // Start moving to the target
            hasRightStartedMovingIn = true;
            hasRightStartedMovingOut = false;
            hasRightTargetReached = false;
            Debug.Log("[INFO] hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);
            // hasRightStartedMovingIn: TRUE, hasRightStartedMovingOut: FALSE, hasRightTargeReached: FALSE
         }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle STAYS RIGHT");

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginRight = originRegion.position;

            // Keep track of the contact position, without updating the fixed one
            hitRight = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation);

            // 1. We update if we did not reach yet the position
            // 2. Or if we stop being in contact when the distance to the hit is larger than the arm itself
            if (!hasRightTargetReached)
            {
                hitRightFixed = hitRight;
            }
            else
            {
                //hitRightFixed = hitRight; // TEST

                // TODO: One for static objects, and other for dynamic?

                //if(isKinematicLeftHandTouching)
                //{
                //    hitLeftFixed = hitLeft;
                //}

                // TODO: The right hand does not go as expected
                if (!isRightInRange)
                {
                    Debug.Log("Updating!");
                    Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);
                    rightHandIK.SetTargetUpdate(hitRightFixed, offset, 0.5f); // TODO: Time that takes to make the small jump
                }
            }

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray)
            if (Physics.Raycast(raycastOriginRight, (hitRightFixed - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                hitNormalRight = hit.normal;

                if (drawRays)
                {
                    Debug.DrawRay(raycastOriginRight, (hitRightFixed - originRegion.position), Color.blue);
                    Debug.DrawRay(hitRightFixed, hitNormalRight * 0.2f, Color.cyan);
                }

                // Set target where it his, based on if reacting or just placing the hand
                // hasRightStartedMovingIn will be TRUE until we reach the object
                rightHandIK.SetTargetStay(reactionTime, hasRightStartedMovingIn);
            }

            // Activate - TODO: Have weights would be great to decide the amount of IK
            rightHandIK.activateIK = true;

            // Once we have finished, we update the variables
            hasRightStartedMovingOut = false;
            if (hasRightTargetReached)
                hasRightStartedMovingIn = false;
            Debug.Log("[INFO] hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);
            // hasRightStartedMovingIn: FALSE, hasRightStartedMovingOut: FALSE, hasRightTargeReached: FALSE -> TRUE (when the coroutine finishes)
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle EXITS RIGHT");

            // Starts moving out
            hasRightStartedMovingIn = false;
            hasRightStartedMovingOut = true;
            hasRightTargetReached = false;
            Debug.Log("[INFO] hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);
            // hasRightStartedMovingIn: FALSE, hasRightStartedMovingOut: TRUE, hasRightTargeReached: FALSE

            // Set target back to original position - TODO: FIX the original position
            rightHandIK.SetTargetBack(reactionTime, hasRightStartedMovingOut);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(hitRightFixed, 0.05f);
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(hitRight, 0.05f);
    }
}

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

    [Header("Left Hand - Hit")]
    public Vector3 hitOffsetLeft;
    public Vector3 hitLeftFixed;
    public float distanceToObstacleFromHitLeftFixed;
    public Vector3 hitLeft;
    public float distanceToObstacleFromHitLeft;
    public Vector3 hitNormalLeft;
    public Vector3 raycastOriginLeft;

    [Header("Left Hand - Debug")]
    public bool hasLeftStartedMovingIn;
    public bool hasLeftContact;
    public bool hasLeftStartedMovingOut;
    public bool firstContact = false;

    #endregion

    #region Read-only & Static Fields

    private SphereCollider sphereColliderLeft;
    private Transform leftTarget;
    private Transform leftHandTransform;

    #endregion

    // Start is called before the first frame update
    void Start()
    {
        sphereColliderLeft = GetComponent<SphereCollider>();
        leftTarget = GameObject.Find("Target Left Hand").GetComponent<Transform>();
        leftHandTransform = leftHandIK.gameObject.GetComponent<Transform>();
    }

    // Update is called once per frame
    void Update()
    {
        // Set Safety Region
        sphereColliderLeft.radius = radiusRegion;
        sphereColliderLeft.center = transform.InverseTransformPoint(originRegion.position) + originOffset;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle ENTERS LEFT");
            firstContact = true;

            // We protect the origin, and get the closest point in the external object to the previous body part to protect (shoulder in this case)
            raycastOriginLeft = originRegion.position;
            hitLeftFixed = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation) + (leftTarget.up * hitOffsetLeft.y) + (leftTarget.right * hitOffsetLeft.x) + (leftTarget.forward * hitOffsetLeft.z);

            // Start moving to the target
            hasLeftStartedMovingIn = true;
            hasLeftStartedMovingOut = false;
            hasLeftContact = false;
            Debug.Log("hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftContact: " + hasLeftContact);
            // hasLeftStartedMovingIn: TRUE, hasLeftStartedMovingOut: FALSE, hasLeftContact: FALSE
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle STAYS LEFT");

            // We protect the origin, and get the closest point in the external object to the previous body part to protect (shoulder in this case)
            raycastOriginLeft = originRegion.position;
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);

            // If we get to away from the first hit position, we update to the closest one.
            distanceToObstacleFromHitLeftFixed = Vector3.Distance(hitLeftFixed, hitLeft);
            Debug.DrawLine(hitLeft, hitLeftFixed, Color.green);

            // New - I dont like it
            float distanceToHand = Vector3.Distance(hitLeft, leftHandTransform.position);
            Debug.Log("distanceToHand: " + distanceToHand);

            // Until the moment we arrived, we keep updating the position
            if (!hasLeftContact)
            {
                Debug.Log("UPDATING SINCE WE DID NOT ARRIVED YET");
                hitLeftFixed = hitLeft;
            }

            if (firstContact)
            {
                if (hasLeftContact && (distanceToObstacleFromHitLeftFixed > 0.5f)) // TODO: Put LENGTH of ARM?
                {
                    Debug.Log("UPDATING SINCE DISTANCE IS LARGER");
                    hitLeftFixed = hitLeft + (leftTarget.up * hitOffsetLeft.y) + (leftTarget.right * hitOffsetLeft.x) + (leftTarget.forward * hitOffsetLeft.z);
                    firstContact = false;
                }
            }
            else
            {
                if (hasLeftContact && ((distanceToObstacleFromHitLeftFixed > 0.5f))) // TODO: Put LENGTH of ARM?
                {
                    Debug.Log("UPDATING SINCE DISTANCE IS LARGER");
                    hitLeftFixed = hitLeft + (leftTarget.up * hitOffsetLeft.y) + (leftTarget.right * hitOffsetLeft.x) + (leftTarget.forward * hitOffsetLeft.z);
                }
            }

            Debug.DrawRay(raycastOriginLeft, (hitLeftFixed - originRegion.position), Color.blue);

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray). It it is an obstacle, enter.
            if (Physics.Raycast(raycastOriginLeft, (hitLeftFixed - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                // hit.point is equal than hitLeft. We just use hit to calculate the normal in that point, thanks to the ray.
                hitNormalLeft = hit.normal;

                Debug.DrawRay(hitLeftFixed, hitNormalLeft * 0.2f, Color.cyan);

                // Set target where it his, based on if reacting or just placing the hand.
                leftHandIK.SetTargetStay(reactionTime, hasLeftStartedMovingIn);
            }

            // Activate -> TODO: Have weights would be great to decide the amount of IK
            leftHandIK.activateIK = true;

            // In process of reaching
            hasLeftStartedMovingOut = false;
            Debug.Log("hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftContact: " + hasLeftContact);
            // hasLeftStartedMovingIn: FALSE, hasLeftStartedMovingOut: FALSE, hasLeftContact: FALSE -> TRUE
        }

    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Obstacle EXITS LEFT");

            // Starts moving out
            hasLeftStartedMovingIn = false;
            hasLeftStartedMovingOut = true;
            hasLeftContact = false;
            Debug.Log("hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftContact: " + hasLeftContact);
            // hasLeftStartedMovingIn: FALSE, hasLeftStartedMovingOut: TRUE, hasLeftContact: FALSE

            // Set target back
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

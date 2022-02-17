using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegionLeft : SafetyRegion
{

    #region Instance Fields

    [Header("Left Hand - Debug")]
    public ArmsFastIK leftHand;
    public Vector3 hitLeft;
    public float distanceToObstacleFromLeft;
    public Vector3 hitLeftStay;
    public float distanceToObstacleFromLeftStay;

    [Header("Left Hand Placement - Debug")]
    public Vector3 hitNormalLeft;
    public Vector3 hitOffsetLeft;
    public Vector3 raycastOriginLeft;

    [Header("Left Hand Flags - Debug")]
    public bool hasLeftStartedMovingIn;
    public bool hasLeftContact;
    public bool hasLeftStartedMovingOut;

    #endregion

    #region Read-only & Static Fields

    private SphereCollider sphereColliderLeft;

    #endregion

    // Start is called before the first frame update
    void Start()
    {
        sphereColliderLeft = GetComponent<SphereCollider>();
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
            Debug.Log("[INFO] Obstacle ENTERS LEFT");

            // We protect the origin, and get the closest point in the external object to the previous body part to protect (shoulder in this case)
            raycastOriginLeft = originRegion.position;
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
            distanceToObstacleFromLeft = Vector3.Distance(hitLeft, originRegion.position);

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
            hitLeftStay = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
            distanceToObstacleFromLeftStay = Vector3.Distance(hitLeftStay, originRegion.position);

            // If we get to away from the first hit position, we update to the closest one.
            distanceToObstacleFromLeft = Vector3.Distance(hitLeft, originRegion.position);
            if (distanceToObstacleFromLeft > 0.4f) // TODO: Put LENGTH of ARM
            {
                hitLeft = hitLeftStay;
                distanceToObstacleFromLeft = distanceToObstacleFromLeftStay;
            }

            // Until the moment we arrived, we keep updating the position
            if(!hasLeftContact)
            {
                hitLeft = hitLeftStay;
                distanceToObstacleFromLeft = distanceToObstacleFromLeftStay;
            }

            Debug.DrawRay(raycastOriginLeft, (hitLeft - originRegion.position), Color.blue);

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray). It it is an obstacle, enter.
            if (Physics.Raycast(raycastOriginLeft, (hitLeft - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                // hit.point is equal than hitLeft. We just use hit to calculate the normal in that point, thanks to the ray.
                hitNormalLeft = hit.normal;

                // TODO: Improve offset
                hitLeft = hitLeft + hitOffsetLeft.x * hitNormalLeft;
                Debug.DrawRay(hitLeft, hitNormalLeft * 0.2f, Color.cyan);

                // Set target where it his, based on if reacting or just placing the hand.
                leftHand.SetTargetStay(reactionTime, hasLeftStartedMovingIn);
            }

            // Activate -> TODO: Have weights would be great to decide the amount of IK
            leftHand.activateIK = true;

            // In process of reaching
            //hasLeftStartedMovingIn = false; // TODO: THIS WORKS
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
            leftHand.SetTargetBack(reactionTime, hasLeftStartedMovingOut);

            // End -> IF THIS THEN DOESNT WORK
            //hasLeftStartedMovingIn = false;
            //hasLeftStartedMovingOut = false;
            //hasLeftContact = false;
            Debug.Log("hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftContact: " + hasLeftContact);
            // hasLeftStartedMovingIn: FALSE, hasLeftStartedMovingOut: FALSE, hasLeftContact: FALSE

        }
    }
}

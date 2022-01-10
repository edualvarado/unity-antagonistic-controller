using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegionLeft : SafetyRegion
{
    [Header("Left Hand - Debug")]
    public ArmsFastIK leftHand;
    public float distanceToObstacleFromLeft;
    public Vector3 hitLeft;
    public Vector3 hitNormalLeft;
    public Vector3 hitOffsetLeft;
    public bool isLeftHandMovingInitially;
    public bool isLeftHandReturningInitially;

    private SphereCollider sphereColliderLeft;

    // Start is called before the first frame update
    void Start()
    {
        sphereColliderLeft = GetComponent<SphereCollider>();
    }

    // Update is called once per frame
    void Update()
    {
        sphereColliderLeft.radius = radiusRegion;
        sphereColliderLeft.center = transform.InverseTransformPoint(originRegion.position) + originOffset;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Entering obstacle LEFT");

            // We protect our shoulder in this version.
            Vector3 raycastOriginLeft = originRegion.position;

            // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);

            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromLeft = Vector3.Distance(hitLeft, originRegion.position);

            // Start moving
            isLeftHandMovingInitially = true;
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Staying obstacle LEFT");

            // We protect our shoulder in this version.
            Vector3 raycastOriginLeft = originRegion.position;

            // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);

            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromLeft = Vector3.Distance(hitLeft, originRegion.position);

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
                leftHand.SetTargetStay(reactionTime, isLeftHandMovingInitially);
            }

            // Activate -> TODO: Have weights would be great to decide the amount of IK
            leftHand.activateIK = true;

            // Stop reacting phase
            isLeftHandMovingInitially = false;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Exiting obstacle LEFT");

            // For the moment is not used
            isLeftHandReturningInitially = true;

            // Set target back
            leftHand.SetTargetBack(reactionTime, isLeftHandReturningInitially);
        }
    }
}

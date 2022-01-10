using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegionRight : SafetyRegion
{
    [Header("Right Hand - Debug")]
    public ArmsFastIK rightHand;
    public float distanceToObstacleFromRight;
    public Vector3 hitRight;
    public Vector3 hitNormalRight;
    public Vector3 hitOffsetRight;
    public bool isRightHandMovingInitially;
    public bool isRightHandReturningInitially;

    private SphereCollider sphereColliderRight;

    // Start is called before the first frame update
    void Start()
    {
        sphereColliderRight = GetComponent<SphereCollider>();
    }

    // Update is called once per frame
    void Update()
    {
        sphereColliderRight.radius = radiusRegion;
        sphereColliderRight.center = transform.InverseTransformPoint(originRegion.position) + originOffset;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Entering obstacle RIGHT");

            // We protect our shoulder in this version.
            Vector3 raycastOriginRight = originRegion.position;

            // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
            hitRight = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation);

            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromRight = Vector3.Distance(hitRight, originRegion.position);
                      
            // Start moving
            isRightHandMovingInitially = true;
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Staying obstacle RIGHT");

            // We protect our shoulder in this version.
            Vector3 raycastOriginRight = originRegion.position;

            // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
            hitRight = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation);

            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromRight = Vector3.Distance(hitRight, originRegion.position);

            Debug.DrawRay(raycastOriginRight, (hitRight - originRegion.position), Color.blue);

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray). It it is an obstacle, enter.
            if (Physics.Raycast(raycastOriginRight, (hitRight - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                // hit.point is equal than hitLeft. We just use hit to calculate the normal in that point, thanks to the ray.
                hitNormalRight = hit.normal;

                // TODO: Improve offset
                hitRight = hitRight + hitOffsetRight.x * hitNormalRight;
                Debug.DrawRay(hitRight, hitNormalRight * 0.2f, Color.cyan);

                // Set target where it his, based on if reacting or just placing the hand.
                rightHand.SetTargetStay(reactionTime, isRightHandMovingInitially);
            }

            // Activate -> TODO: Have weights would be great to decide the amount of IK
            rightHand.activateIK = true;

            // Stop reacting
            isRightHandMovingInitially = false;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("[INFO] Exiting obstacle RIGHT");

            // For the moment is not used
            isRightHandReturningInitially = true;

            // Set target back
            rightHand.SetTargetBack(reactionTime, isRightHandReturningInitially);
        }
    }
}

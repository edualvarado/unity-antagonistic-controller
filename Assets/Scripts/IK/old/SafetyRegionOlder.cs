using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegionOlder : MonoBehaviour
{
    public bool activateIK = false;

    [Header("Kinematic Model")]
    public Transform kinBody;

    [Header("Left Hand")]
    public ArmsFastIK leftHand;
    public Transform leftShoulder;
    public float distanceToObstacleFromLeft;
    public Vector3 hitLeft;
    public Vector3 hitNormalLeft;
    public Vector3 offsetLeft;

    [Header("Right Hand")]
    public ArmsFastIK rightHand;
    public Transform rightShoulder;
    public float distanceToObstacleFromRight;
    public Vector3 hitRight;
    public Vector3 hitNormalRight;
    public Vector3 offsetRight;

    [Header("Debug")]
    public bool isLeftHandMovingInitially;
    public bool isLeftHandReturningInitially;
    public bool isRightHandMovingInitially;
    public bool isRightHandReturningInitially;

    [Header("Settings")]
    public float reactionTime;

    public Vector3 leftOriginalPos;
    public Vector3 leftOriginalPosLocal;
    public Quaternion leftOriginalRot;
    public Vector3 rightOriginalPos;
    public Vector3 rightOriginalPosLocal;
    public Quaternion rightOriginalRot;

    private CapsuleCollider colliderSafetyRegion;

    public SphereCollider leftSafetyRegion;
    public SphereCollider rightSafetyRegion;

    // Start is called before the first frame update
    void Start()
    {
        //colliderSafetyRegion = GetComponent<CapsuleCollider>();
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("ASD");
        if (other.CompareTag("Obstacle"))
        {
            // We protect our shoulder in this version.
            Vector3 raycastOriginLeft = leftShoulder.position;
            Vector3 raycastOriginRight = rightShoulder.position;

            // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
            hitRight = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation);

            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromLeft = Vector3.Distance(hitLeft, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(hitRight, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                Debug.Log("[INFO] Entering obstacle LEFT");

                // Start moving
                isLeftHandMovingInitially = true;
            }
            else if (distanceToObstacleFromLeft > distanceToObstacleFromRight)
            {
                Debug.Log("[INFO] Entering obstacle RIGHT");

                // Start moving
                isRightHandMovingInitially = true;
            }
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            // We protect our shoulder in this version.
            Vector3 raycastOriginLeft = leftShoulder.position;
            Vector3 raycastOriginRight = rightShoulder.position;

            // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
            hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
            hitRight = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation);

            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromLeft = Vector3.Distance(hitLeft, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(hitRight, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                Debug.Log("[INFO] Staying obstacle LEFT");
                Debug.DrawRay(raycastOriginLeft, (hitLeft - leftShoulder.position), Color.blue);
                Debug.DrawRay(raycastOriginRight, (hitRight - leftShoulder.position), Color.red);

                // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray). It it is an obstacle, enter.
                if (Physics.Raycast(raycastOriginLeft, (hitLeft - leftShoulder.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
                {
                    // hit.point is equal than hitLeft. We just use hit to calculate the normal in that point, thanks to the ray.
                    hitNormalLeft = hit.normal;

                    // TODO: Improve offset
                    hitLeft = hitLeft + offsetLeft.x * hitNormalLeft;
                    Debug.DrawRay(hitLeft, hitNormalLeft * 0.2f, Color.cyan);

                    // Set target where it his, based on if reacting or just placing the hand.
                    leftHand.SetTargetStay(reactionTime, isLeftHandMovingInitially);
                }

                // TODO: Provisional, only one hand at a time.
                leftHand.activateIK = true;
                rightHand.activateIK = false;

                // Stop reacting
                isLeftHandMovingInitially = false;
            }
            else if(distanceToObstacleFromLeft > distanceToObstacleFromRight)
            {
                Debug.Log("[INFO] Staying obstacle RIGHT");
                Debug.DrawRay(raycastOriginLeft, (hitLeft - leftShoulder.position), Color.red);
                Debug.DrawRay(raycastOriginRight, (hitRight - leftShoulder.position), Color.blue);

                // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray). It it is an obstacle, enter.
                if (Physics.Raycast(raycastOriginRight, (hitRight - rightShoulder.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
                {
                    // hit.point is equal than hitLeft. We just use hit to calculate the normal in that point, thanks to the ray.
                    hitNormalRight = hit.normal;

                    // TODO: Improve offset
                    hitRight = hitRight + offsetRight.x * hitNormalRight;
                    Debug.DrawRay(hitRight, hitNormalRight * 0.2f, Color.cyan);

                    // Set target where it his, based on if reacting or just placing the hand.
                    rightHand.SetTargetStay(reactionTime, isRightHandMovingInitially);
                }

                // TODO: Provisional, only one hand at a time.
                leftHand.activateIK = false;
                rightHand.activateIK = true;

                // Stop reacting
                isRightHandMovingInitially = false;
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            // For the moment is not used
            isLeftHandReturningInitially = true;
            isRightHandReturningInitially = true;

            leftHand.SetTargetBack(reactionTime, isLeftHandReturningInitially);
            rightHand.SetTargetBack(reactionTime, isRightHandReturningInitially);
        }
    }
}

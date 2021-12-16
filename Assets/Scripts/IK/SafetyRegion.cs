using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegion : MonoBehaviour
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
    public float leftThreshold = 0.5f;

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

    // Start is called before the first frame update
    void Start()
    {
        colliderSafetyRegion = GetComponent<CapsuleCollider>();
    }

    // Update is called once per frame
    void Update()
    {
        // Keep track of the original position before moving the hand.
        Debug.DrawRay(kinBody.position, kinBody.rotation * leftOriginalPosLocal, Color.blue);

        // Change:
        //rightOriginalPosLocal = transform.InverseTransformPoint(kinBody.position + (kinBody.rotation * rightOriginalPos));
        //Debug.DrawRay(kinBody.position, kinBody.rotation * rightOriginalPosLocal, Color.blue);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromLeft = Vector3.Distance(other.transform.position, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(other.transform.position, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                // Position and rotation of constant target
                leftOriginalPos = leftHand.TargetConstant.position;
                leftOriginalPosLocal = transform.InverseTransformPoint(leftOriginalPos);
                leftOriginalRot = leftHand.TargetConstant.rotation;

                // Start moving
                isLeftHandMovingInitially = true;
            }
            else if (distanceToObstacleFromLeft > distanceToObstacleFromRight)
            {
                // FOR NOW NO RIGHT

                //rightOriginalPos = transform.InverseTransformPoint(rightHand.GetComponent<Transform>().position);
                //rightOriginalRot = rightHand.Target.rotation;

                //isRightHandMovingInitially = true;
            }
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            // Remember that you are calculating the distance to the center (in a wall, it would be far away)
            distanceToObstacleFromLeft = Vector3.Distance(other.transform.position, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(other.transform.position, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                // We protect our shoulder in this version.
                Vector3 raycastOriginLeft = leftShoulder.position;

                // Get the closest point in the external object to the previous body part to protect (shoulder in this case)
                hitLeft = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
                Debug.DrawRay(raycastOriginLeft, (hitLeft - leftShoulder.position), Color.red);

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
            else
            {
                /*
                // We protect our shoulder in this version.
                Vector3 raycastOriginRight = rightShoulder.position;

                // TODO: Use InverseTransform?
                Debug.DrawRay(raycastOriginRight, (other.transform.position - rightShoulder.position), Color.red);

                if (Physics.Raycast(raycastOriginRight, (other.transform.position - rightShoulder.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
                {
                    // Detect impact point.
                    hitRight = hit.point;
                    hitNormalRight = hit.normal;

                    // TODO: Improve offset
                    hitRight = hitRight + offsetRight.x * hitNormalRight;

                    Debug.DrawRay(hitRight, hitNormalRight * 0.2f, Color.cyan);

                    // Set target where it his, based on if reacting or just placing the hand.
                    rightHand.SetTargetStay(hitRight, hitNormalRight, reactionTime, isRightHandMovingInitially);
                }

                // TODO: Provisional, only one hand at a time.
                rightHand.activateIK = true;
                leftHand.activateIK = false;

                // Stop reacting
                isRightHandMovingInitially = false;
                */
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

            distanceToObstacleFromLeft = Vector3.Distance(other.transform.position, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(other.transform.position, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                leftHand.SetTargetBack(reactionTime, isLeftHandReturningInitially);
            }
            else
            {
                /*
                rightHand.SetTargeBack(rightOriginalPosLocal, kinBody, rightOriginalRot, reactionTime, isRightHandReturningInitially);
                */
            }

        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(kinBody.position + (kinBody.rotation * leftOriginalPos), 0.1f);
    }
}

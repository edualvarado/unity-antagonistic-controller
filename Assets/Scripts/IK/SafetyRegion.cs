using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegion : MonoBehaviour
{
    public bool activateIK = false;

    public ArmsFastIK leftHand;
    public float distanceToObstacleFromLeft;
    public Vector3 hitLeft;
    public Vector3 hitNormalLeft;
    public Vector3 offsetLeft;
    public float leftThreshold = 0.5f;

    public ArmsFastIK rightHand;
    public float distanceToObstacleFromRight;
    public Vector3 hitRight;
    public Vector3 hitNormalRight;
    public Vector3 offsetRight;

    public Transform leftShoulder;
    public Transform rightShoulder;

    public bool isLeftHandMovingInitially;
    public bool isRightHandMovingInitially;

    public float reactionTime;

    public bool isLeftHandReturningInitially;
    public Vector3 leftOriginalPos;
    public Quaternion leftOriginalRot;

    public bool isRightHandReturningInitially;
    public Vector3 rightOriginalPos;
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
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            distanceToObstacleFromLeft = Vector3.Distance(other.transform.position, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(other.transform.position, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                // TODO: WRONG - we need to take it with respect to the body or safety region
                leftOriginalPos = leftHand.GetComponent<Transform>().position;
                leftOriginalRot = leftHand.GetComponent<Transform>().rotation;

                isLeftHandMovingInitially = true;
            }
            else if (distanceToObstacleFromLeft > distanceToObstacleFromRight)
            {
                rightOriginalPos = rightHand.GetComponent<Transform>().position;
                rightOriginalRot = rightHand.GetComponent<Transform>().rotation;

                isRightHandMovingInitially = true;
            }
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            distanceToObstacleFromLeft = Vector3.Distance(other.transform.position, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(other.transform.position, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                // We protect our shoulder in this version.
                Vector3 raycastOriginLeft = leftShoulder.position;

                // TODO: Use InverseTransform?
                Debug.DrawRay(raycastOriginLeft, (other.transform.position - leftShoulder.position), Color.red);

                if (Physics.Raycast(raycastOriginLeft, (other.transform.position - leftShoulder.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
                {
                    // Detect impact point.
                    hitLeft = hit.point;
                    hitNormalLeft = hit.normal;

                    // TODO: Improve offset
                    hitLeft = hitLeft + offsetLeft.x * hitNormalLeft;

                    Debug.DrawRay(hitLeft, hitNormalLeft * 0.2f, Color.cyan);

                    // Set target where it his, based on if reacting or just placing the hand.
                    leftHand.SetTarget(hitLeft, hitNormalLeft, reactionTime, isLeftHandMovingInitially, 0);
                }

                // TODO: Provisional, only one hand at a time.
                leftHand.activateIK = true;
                rightHand.activateIK = false;

                // Stop reacting
                isLeftHandMovingInitially = false;
            }
            else
            {
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
                    rightHand.SetTarget(hitRight, hitNormalRight, reactionTime, isRightHandMovingInitially, 1);
                }

                // TODO: Provisional, only one hand at a time.
                rightHand.activateIK = true;
                leftHand.activateIK = false;

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

            distanceToObstacleFromLeft = Vector3.Distance(other.transform.position, leftShoulder.position);
            distanceToObstacleFromRight = Vector3.Distance(other.transform.position, rightShoulder.position);

            if (distanceToObstacleFromLeft < distanceToObstacleFromRight)
            {
                // TODO: WRONG: SET ORIGINAL SETPOINT
                leftHand.SetTargetOrigin(leftOriginalPos, leftOriginalRot, reactionTime, isLeftHandReturningInitially);
            }
            else
            {
                rightHand.SetTargetOrigin(rightOriginalPos, rightOriginalRot, reactionTime, isRightHandReturningInitially);
            }

        }
    }
}

/****************************************************
 * File: SafetyRegionRight.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class SafetyRegionRight : SafetyRegion
{
    #region Read-only & Static Fields

    private SphereCollider sphereColliderRight;
    private Transform rightTargetTransform;

    #endregion

    #region Instance Fields

    [Header("Right Hand - Obstacles")]
    [SerializeField]
    public List<Obstacle> obstacles = new List<Obstacle>();
    public Obstacle targetObstacle;
    public Collider nowObstacle;
    public Collider prevObstacle;
    public bool colliderChanged = false;

    [Header("Right Hand - IK")]
    public TargetIK rightTarget;
    public bool fixHandToDynamicObject = false;
    public bool drawIK;

    [Header("Right Hand - Hit")]
    public Vector3 hitOffsetRight;
    public Vector3 hitRightFixed;
    public Vector3 localHitRightFixed;
    public Vector3 hitRight;
    public Vector3 hitNormalRight;
    public Vector3 raycastOriginRight;

    [Header("Right Hand - Debug")]
    public bool hasRightStartedMovingIn;
    public bool hasRightTargetReached;
    public bool hasRightStartedMovingOut;
    public bool isRightInRange = false;

    #endregion

    #region Unity Methods

    void Start()
    {
        sphereColliderRight = GetComponent<SphereCollider>();
        rightTargetTransform = GameObject.Find("Target Right Hand").GetComponent<Transform>();
    }

    void Update()
    {
        sphereColliderRight.radius = radiusRegion;
        sphereColliderRight.center = transform.InverseTransformPoint(originRegion.position) + originOffset;
    }

    #endregion

    // When an object enters in the Safety Region
    private void OnTriggerEnter(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[INFO] New obstacle ENTERS RIGHT: " + other.name);

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginRight = originRegion.position;

            // Create an offset, in case it is necessary
            Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);

            // For the first obstacle, the list is empty at this point
            if (obstacles.Count == 0)
            {
                // Start moving to the target
                hasRightStartedMovingIn = true;
                hasRightStartedMovingOut = false;
                hasRightTargetReached = false;
                Debug.Log("[TEST] 1) hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);
            }

            // Add new obstacle to the dynamic list
            Vector3 closestPoint = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation) + offset;

            // TODO: Retrieve mass
            if(other.gameObject.GetComponent<Rigidbody>())
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginRight), other.gameObject.GetComponent<ObstacleMass>().realMass, other.gameObject.GetComponent<ObstacleMass>().expectedMass));
            else
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginRight), other.GetComponentInParent<ObstacleMass>().realMass, other.GetComponentInParent<ObstacleMass>().expectedMass));
        }
    }

    // When an object stays in the Safety Region
    private void OnTriggerStay(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[INFO] Obstacle STAYS RIGHT: " + other.name);

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginRight = originRegion.position;

            // Take the instance of the obstacle that corresponds, and update at each time the closest point and distance to it
            Obstacle currentObstacle = obstacles.Find(x => x.obstacle == other);
            Vector3 closestPoint = Physics.ClosestPoint(raycastOriginRight, other, other.transform.position, other.transform.rotation);
            currentObstacle.location = closestPoint;
            currentObstacle.distance = Vector3.Distance(closestPoint, raycastOriginRight);

            // In case of multiple objects inside the region, we take the closest one -> TODO Can be improved
            var min = 100f;
            for (int i = 0; i < obstacles.Count; i++)
            {
                if (min > obstacles[i].distance)
                {
                    // We update the closest obstacle
                    targetObstacle = obstacles[i]; // To provide to the Antagonistic Gains
                    nowObstacle = obstacles[i].obstacle;
                    hitRight = obstacles[i].location;
                    min = obstacles[i].distance;   
                }
            }

            // If we update the obstacle for a new one, we make colliderChanged TRUE
            if(prevObstacle != nowObstacle)
            {
                colliderChanged = true;
                Debug.Log("[TEST] colliderChanged RIGHT: " + colliderChanged);
                prevObstacle = nowObstacle;
            }

            // Here, we already have the hitRight that corresponds to the closest point to the origin
            // Now, we need to take hitRightFixed and updated with the constantly tracked value of hitRight when is convenient

            // =====

            // When do we update?
            // 1. We update constantly to the hit if we did not reach yet the position or if we stop being in contact when the distance to the hit is larger than the arm itself
            // 2. Otherwise, hasRightStartedMovingIn is false
            if (hasRightStartedMovingIn) 
            {
                // Create an offset, in case it is necessary
                Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);

                Debug.Log("[TEST] Fixing to pose because hasRightStartedMovingIn: " + hasRightStartedMovingIn);
                hitRightFixed = hitRight + offset;
                localHitRightFixed = (hitRightFixed - other.transform.position);

            }
            else
            {
                // Two types of objects: Dynamic and Static
                // 1. Dynamic: If the object can move, we update always to the closest position for convenience
                // 2. If the object is rigid (like a wall), to the updates when we are far from the fixed position
                if (other.CompareTag("Dynamic Obstacle"))
                {
                    // Create an offset, in case it is necessary
                    Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);

                    // Two modes: First one is experimental
                    if (fixHandToDynamicObject)
                    {
                        hitRightFixed = other.transform.position + localHitRightFixed + offset;
                    }
                    else
                    {
                        // During the contact, we update constantly to the hit
                        // If the system detects a change of obstacle, we update the bools, that will induce again the smooth transition
                        if(!colliderChanged && !hasRightStartedMovingIn)
                        {
                            Debug.Log("[TEST] Fixing to pose because colliderChanged: " + colliderChanged + " and hasRightStartedMovingIn: " + hasRightStartedMovingIn);
                            hitRightFixed = hitRight + offset;
                        }
                        else if (colliderChanged)
                        {
                            // Start moving to the target
                            hasRightStartedMovingIn = true;
                            hasRightStartedMovingOut = false;
                            hasRightTargetReached = false;
                            Debug.Log("[TEST] 2) hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);
                        }
                    }
                }
                else if (other.CompareTag("Static Obstacle"))
                {
                    if (!isRightInRange)
                    {
                        Debug.Log("WALL UPDATE RIGHT");
                        Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);
                        rightTarget.SetTargetUpdate(hitRightFixed, offset, 0.5f); // TODO: Time that takes to make the small jump
                    }
                }
            }

            // =====

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray)
            if (Physics.Raycast(raycastOriginRight, (hitRightFixed - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                hitNormalRight = hit.normal;

                if (drawIK)
                {
                    Debug.DrawRay(raycastOriginRight, (hitRightFixed - originRegion.position), Color.blue);
                    Debug.DrawRay(hitRightFixed, hitNormalRight * 0.2f, Color.red);
                }

                // Method in charge of moving behaviour, which changes if we are on the move or we arrived
                // hasRightStartedMovingIn will be TRUE until we reach the object
                rightTarget.SetTargetStay(reactionTime, hasRightStartedMovingIn);
            }

            // Allow IK from this point
            rightTarget.activateIK = true;

            // Once we have finished, we update the variables
            hasRightStartedMovingOut = false;
            if (hasRightTargetReached) 
                hasRightStartedMovingIn = false;

            Debug.Log("[TEST] 3) hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);

            colliderChanged = false;
            Debug.Log("[TEST] colliderChanged: " + colliderChanged);
        }
    }

    // When an object exits in the Safety Region
    private void OnTriggerExit(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[TEST] Obstacle EXITS RIGHT: " + other.name);

            // Remove the obstacle instance from the dynamic list
            obstacles.RemoveAll(x => x.obstacle == other);

            // Set target back to original position
            // This should only happen if NO objects are inside the region
            if(obstacles.Count == 0)
            {
                // Starts moving out
                hasRightStartedMovingIn = false;
                hasRightStartedMovingOut = true;
                hasRightTargetReached = false;
                Debug.Log("[TEST] 4) hasRightStartedMovingIn: " + hasRightStartedMovingIn + " | hasRightStartedMovingOut: " + hasRightStartedMovingOut + " | hasRightTargeReached: " + hasRightTargetReached);

                // Method in charge of moving back the hand
                rightTarget.SetTargetBack(reactionTime, hasRightStartedMovingOut);
            }
        }
    }

    private void OnDrawGizmos()
    {
        //Gizmos.color = Color.red;
        //Gizmos.DrawSphere(hitRightFixed, 0.05f);
        //Gizmos.color = Color.green;
        //Gizmos.DrawSphere(hitRight, 0.05f);
    }
}

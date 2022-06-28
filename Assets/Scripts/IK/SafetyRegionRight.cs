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

    // Continue of "Safety Region - Settings INTERACTIVE"
    public bool manualMode;
    public float realVelocityRight;
    public float expectedVelocityRight;
    public float expectedMaxVelocityRight = 10f;

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

    // Extension Colliders - New
    [Header("Other Safety Regions Right - All")]
    public float sphereAllCollidersRight;

    [Header("Other Safety Regions Right - 2")]
    public OnTriggerEventRight2 onTriggerEventRight2;
    public SphereCollider sphereColliderRight2;
    public Vector3 sphereColliderRight2Center;
    public float sphereColliderRight2Multiplier;

    [Header("Other Safety Regions Right - 3")]
    public OnTriggerEventRight3 onTriggerEventRight3;
    public SphereCollider sphereColliderRight3;
    public Vector3 sphereColliderRight3Center;
    //public float sphereColliderRight3Multiplier;

    [Header("Hand Spring")]
    public SpringJoint rightHandSpring;
    public bool modifySpring;

    [Header("New Reaction Time")]
    public float newReactionTime;
    public float securityWeight;
    public float differenceDistance;
    public float preReactionTime;

    [Header("Hand RB")]
    public Rigidbody handRB;

    #endregion

    #region Unity Methods

    void Start()
    {
        sphereColliderRight = GetComponent<SphereCollider>();
        rightTargetTransform = GameObject.Find("Target Right Hand").GetComponent<Transform>();

        // Initialize by default
        //reactionTime = 0.5f;
    }

    void Update()
    {
        sphereColliderRight.radius = radiusRegion;
        sphereColliderRight.center = transform.InverseTransformPoint(originRegion.position) + originOffset;

        // Extension Colliders - New
        sphereColliderRight2.center = sphereColliderRight.center + sphereColliderRight2Center + new Vector3(0f,0f, radiusRegion);
        sphereColliderRight2.radius = sphereColliderRight2Center.z + radiusRegion;

        sphereColliderRight3.center = sphereColliderRight2.center + sphereColliderRight3Center + new Vector3(0f, 0f, sphereColliderRight2.radius);
        sphereColliderRight3.radius = sphereColliderRight3Center.z + sphereColliderRight2.radius;

        sphereColliderRight.radius += sphereAllCollidersRight;
        sphereColliderRight2.radius += sphereAllCollidersRight;
        sphereColliderRight3.radius += sphereAllCollidersRight;

        // Retrieve velocity
        expectedVelocityRight = targetObstacle.expectedVelocity;
        realVelocityRight = targetObstacle.realVelocity;

        if (!manualMode)
        {
            reactionTime = Mathf.Lerp(2f, 0.3f, expectedVelocityRight / expectedMaxVelocityRight);

            differenceDistance = Vector3.Distance(hitRightFixed, raycastOriginRight);
            preReactionTime = (differenceDistance / expectedVelocityRight);
            newReactionTime = Mathf.Clamp(securityWeight * preReactionTime, 0.3f, 2f);
        }
    }

    #endregion

    // When an object enters in the Safety Region
    private void OnTriggerEnter(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            // TEST - Setting the spring based on the reaction time
            if(modifySpring)
                StartCoroutine(IncreaseSpring());

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
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginRight), other.gameObject.GetComponent<ObstacleDynamics>().realMass, other.gameObject.GetComponent<ObstacleDynamics>().expectedMass, other.gameObject.GetComponent<ObstacleDynamics>().realVelocity, other.gameObject.GetComponent<ObstacleDynamics>().expectedVelocity));
            else
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginRight), other.GetComponentInParent<ObstacleDynamics>().realMass, other.GetComponentInParent<ObstacleDynamics>().expectedMass, other.GetComponentInParent<ObstacleDynamics>().realVelocity, other.GetComponentInParent<ObstacleDynamics>().expectedVelocity));
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
                        //handRB.freezeRotation = false;

                        Vector3 offset = (rightTargetTransform.up * hitOffsetRight.y) + (rightTargetTransform.right * hitOffsetRight.x) + (rightTargetTransform.forward * hitOffsetRight.z);
                        rightTarget.SetTargetUpdate(hitRightFixed, offset, 0.5f); // TODO: Time that takes to make the small jump
                    }
                    else
                    {
                        Debug.Log("NOW FIX HAND");
                        //handRB.freezeRotation = true;
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
        // TEST - Setting the spring based on the reaction time
        if (modifySpring)
            StartCoroutine(DecreaseSpring());

        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[TEST] Obstacle EXITS RIGHT: " + other.name);

            // Remove the obstacle instance from the dynamic list
            //obstacles.RemoveAll(x => x.obstacle == other);

            var itemsToRemove = obstacles.Where(x => x.obstacle == other); 
            foreach (var i in itemsToRemove)
            {
                obstacles.Remove(i);
                break;
            }



            // Set target back to original position
            // This should only happen if NO objects are inside the region
            if (obstacles.Count == 0)
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

    // Extension Colliders - New

    void OnEnable()
    {
        // Safety 2
        onTriggerEventRight2.onTriggerEnter.AddListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventRight2.onTriggerStay.AddListener(OnTheOtherTriggerStayMethod);
        onTriggerEventRight2.onTriggerExit.AddListener(OnTheOtherTriggerExitMethod);

        // Safety 3
        onTriggerEventRight3.onTriggerEnter.AddListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventRight3.onTriggerStay.AddListener(OnTheOtherTriggerStayMethod);
        onTriggerEventRight3.onTriggerExit.AddListener(OnTheOtherTriggerExitMethod);
    }

    void OnDisable()
    {
        // Safety 2
        onTriggerEventRight2.onTriggerEnter.RemoveListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventRight2.onTriggerStay.RemoveListener(OnTheOtherTriggerStayMethod);
        onTriggerEventRight2.onTriggerExit.RemoveListener(OnTheOtherTriggerExitMethod);

        // Safety 3
        onTriggerEventRight3.onTriggerEnter.RemoveListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventRight3.onTriggerStay.RemoveListener(OnTheOtherTriggerStayMethod);
        onTriggerEventRight3.onTriggerExit.RemoveListener(OnTheOtherTriggerExitMethod);
    }

    void OnTheOtherTriggerEnterMethod(Collider other)
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
            if (other.gameObject.GetComponent<Rigidbody>())
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginRight), other.gameObject.GetComponent<ObstacleDynamics>().realMass, other.gameObject.GetComponent<ObstacleDynamics>().expectedMass, other.gameObject.GetComponent<ObstacleDynamics>().realVelocity, other.gameObject.GetComponent<ObstacleDynamics>().expectedVelocity));
            else
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginRight), other.GetComponentInParent<ObstacleDynamics>().realMass, other.GetComponentInParent<ObstacleDynamics>().expectedMass, other.GetComponentInParent<ObstacleDynamics>().realVelocity, other.GetComponentInParent<ObstacleDynamics>().expectedVelocity));

        }
    }

    void OnTheOtherTriggerStayMethod(Collider other)
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
            if (prevObstacle != nowObstacle)
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
                        if (!colliderChanged && !hasRightStartedMovingIn)
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
                //rightTarget.SetTargetStay(reactionTime, hasRightStartedMovingIn); // COMMENTED FOR TEST
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

    void OnTheOtherTriggerExitMethod(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[TEST] Obstacle EXITS RIGHT: " + other.name);

            // Remove the obstacle instance from the dynamic list
            //obstacles.RemoveAll(x => x.obstacle == other);
            var itemsToRemove = obstacles.Where(x => x.obstacle == other);
            foreach (var i in itemsToRemove)
            {
                obstacles.Remove(i);
                break;
            }

            // Set target back to original position
            // This should only happen if NO objects are inside the region
            if (obstacles.Count == 0)
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

    // Hand Springs

    IEnumerator IncreaseSpring()
    {
        float timeElapsed = 0;
        while (timeElapsed < reactionTime)
        {
            rightHandSpring.spring = Mathf.Lerp(0f, 10000f, timeElapsed / reactionTime);
            timeElapsed += Time.deltaTime;
            yield return null;
        }
        rightHandSpring.spring = 10000f;
    }

    IEnumerator DecreaseSpring()
    {
        float timeElapsed = 0;
        while (timeElapsed < reactionTime)
        {
            rightHandSpring.spring = Mathf.Lerp(10000f, 0f, timeElapsed / reactionTime);
            timeElapsed += Time.deltaTime;
            yield return null;
        }
        rightHandSpring.spring = 0f;
    }

    private void OnDrawGizmos()
    {
        //Gizmos.color = Color.red;
        //Gizmos.DrawSphere(hitRightFixed, 0.05f);
        //Gizmos.color = Color.green;
        //Gizmos.DrawSphere(hitRight, 0.05f);
    }
}

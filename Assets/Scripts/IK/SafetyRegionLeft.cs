/****************************************************
 * File: SafetyRegionLeft.cs
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

public class SafetyRegionLeft : SafetyRegion
{
    #region Read-only & Static Fields

    private SphereCollider sphereColliderLeft;
    private Transform leftTargetTransform;

    #endregion

    #region Instance Fields

    // Continue of "Safety Region - Settings INTERACTIVE"
    public bool manualMode;
    public float realVelocityLeft = 0f;
    public float expectedVelocityLeft = 0f;
    public float expectedMaxVelocityLeft = 10f;
    public float realVelocityLeftOld = 0f; // TEST
    public float expectedVelocityLeftOld = 0f; // TEST

    [Header("Left Hand - Obstacles")]
    [SerializeField]
    public List<Obstacle> obstacles = new List<Obstacle>();
    public Obstacle targetObstacle;
    public Collider nowObstacle;
    public Collider prevObstacle;
    public bool colliderChanged = false;

    [Header("Left Hand - IK")]
    public TargetIK leftTarget;
    public bool fixHandToDynamicObject = false;
    public bool drawIK;

    [Header("Left Hand - Hit")]
    public Vector3 hitOffsetLeft;
    public Vector3 hitLeftFixed;
    public Vector3 localHitLeftFixed;
    public Vector3 hitLeft;
    public Vector3 hitNormalLeft;
    public Vector3 raycastOriginLeft;

    [Header("Left Hand - Debug")]
    public bool hasLeftStartedMovingIn;
    public bool hasLeftTargetReached;
    public bool hasLeftStartedMovingOut;
    public bool isLeftInRange = false;

    // Extension Colliders - New
    [Header("Other Safety Regions Left - All")]
    public float sphereAllCollidersLeft;

    [Header("Other Safety Regions Left - 2")]
    public OnTriggerEventLeft2 onTriggerEventLeft2;
    public SphereCollider sphereColliderLeft2;
    public Vector3 sphereColliderLeft2Center;
    public float sphereColliderLeft2Multiplier;

    [Header("Other Safety Regions Left - 3")]
    public OnTriggerEventLeft3 onTriggerEventLeft3;
    public SphereCollider sphereColliderLeft3;
    public Vector3 sphereColliderLeft3Center;
    //public float sphereColliderLeft3Multiplier;

    [Header("Hand Spring")]
    public SpringJoint leftHandSpring;
    public bool modifySpring;


    [Header("Hand RB")]
    public Rigidbody handRB;

    #endregion

    #region Unity Methods

    void Start()
    {
        sphereColliderLeft = GetComponent<SphereCollider>();
        leftTargetTransform = GameObject.Find("Target Left Hand").GetComponent<Transform>();
    }

    void Update()
    {
        sphereColliderLeft.radius = radiusRegion;
        sphereColliderLeft.center = transform.InverseTransformPoint(originRegion.position) + originOffset;

        // Extension Colliders - New
        sphereColliderLeft2.center = sphereColliderLeft.center + sphereColliderLeft2Center + new Vector3(0f, 0f, radiusRegion);
        sphereColliderLeft2.radius = sphereColliderLeft2Center.z + radiusRegion;

        sphereColliderLeft3.center = sphereColliderLeft2.center + sphereColliderLeft3Center + new Vector3(0f, 0f, sphereColliderLeft2.radius);
        sphereColliderLeft3.radius = sphereColliderLeft3Center.z + sphereColliderLeft2.radius;

        sphereColliderLeft.radius += sphereAllCollidersLeft;
        sphereColliderLeft2.radius += sphereAllCollidersLeft;
        sphereColliderLeft3.radius += sphereAllCollidersLeft;

        // Retrieve velocity (max) - TEST

        //expectedVelocityLeftOld = targetObstacle.expectedVelocity;
        //if (expectedVelocityLeftOld >= expectedVelocityLeft)
        //{
        //    expectedVelocityLeft = targetObstacle.expectedVelocity;
        //}
        //realVelocityLeftOld = targetObstacle.realVelocity;
        //if (realVelocityLeftOld >= realVelocityLeft)
        //{
        //    realVelocityLeft = targetObstacle.realVelocity;
        //}

        // Retrieve velocity
        expectedVelocityLeft = targetObstacle.expectedVelocity;
        realVelocityLeft = targetObstacle.realVelocity;


        if (!manualMode)
        {
            reactionTime = Mathf.Lerp(2f, 0.3f, expectedVelocityLeft / expectedMaxVelocityLeft);



        }
    }

    #endregion

    // When an object enters in the Safety Region
    private void OnTriggerEnter(Collider other)
    {
        // TEST - Setting the spring based on the reaction time
        if (modifySpring)
            StartCoroutine(IncreaseSpring());

        if (other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle"))
        {
            //Debug.Log("[INFO] New obstacle ENTERS LEFT: " + other.name);

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginLeft = originRegion.position;

            // Create an offset, in case it is necessary
            Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);

            // For the first obstacle, the list is empty at this point
            if (obstacles.Count == 0)
            {
                // Start moving to the target
                hasLeftStartedMovingIn = true;
                hasLeftStartedMovingOut = false;
                hasLeftTargetReached = false;
                Debug.Log("[TEST] 1) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);
            }

            // Add new obstacle to the dynamic list
            Vector3 closestPoint = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation) + offset;

            // TODO: Retrieve mass
            if(other.gameObject.GetComponent<Rigidbody>())
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginLeft), other.gameObject.GetComponent<ObstacleDynamics>().realMass, other.gameObject.GetComponent<ObstacleDynamics>().expectedMass, other.gameObject.GetComponent<ObstacleDynamics>().realVelocity, other.gameObject.GetComponent<ObstacleDynamics>().expectedVelocity));
            else
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginLeft), other.GetComponentInParent<ObstacleDynamics>().realMass, other.GetComponentInParent<ObstacleDynamics>().expectedMass, other.GetComponentInParent<ObstacleDynamics>().realVelocity, other.GetComponentInParent<ObstacleDynamics>().expectedVelocity));
        }
    }

    // When an object stays in the Safety Region
    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle"))
        {
            //Debug.Log("[INFO] Obstacle STAYS LEFT: " + other.name);

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginLeft = originRegion.position;

            // Take the instance of the obstacle that corresponds, and update at each time the closest point and distance to it
            Obstacle currentObstacle = obstacles.Find(x => x.obstacle == other);
            Vector3 closestPoint = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
            currentObstacle.location = closestPoint;
            currentObstacle.distance = Vector3.Distance(closestPoint, raycastOriginLeft);
            
            // In case of multiple objects inside the region, we take the closest one -> TODO Can be improved
            var min = 100f;
            for (int i = 0; i < obstacles.Count; i++)
            {
                if (min > obstacles[i].distance)
                {
                    // We update the closest obstacle
                    targetObstacle = obstacles[i]; // To provide to the Antagonistic Gains
                    nowObstacle = obstacles[i].obstacle;
                    hitLeft = obstacles[i].location;
                    min = obstacles[i].distance;
                }
            }

            // If we update the obstacle for a new one, we make colliderChanged TRUE
            if (prevObstacle != nowObstacle)
            {
                colliderChanged = true;
                Debug.Log("[TEST] colliderChanged LEFT: " + colliderChanged);
                prevObstacle = nowObstacle;
            }

            // Here, we already have the hitLeft that corresponds to the closest point to the origin
            // Now, we need to take hitLeftFixed and updated with the constantly tracked value of hitLeft when is convenient

            // =====

            // When do we update?
            // 1. We update constantly to the hit if we did not reach yet the position or if we stop being in contact when the distance to the hit is larger than the arm itself
            // 2. Otherwise, hasLeftStartedMovingIn is false
            if (hasLeftStartedMovingIn)
            {
                Debug.Log("[TEST] Fixing to pose because hasLeftStartedMovingIn: " + hasLeftStartedMovingIn);
                hitLeftFixed = hitLeft;
                localHitLeftFixed = (hitLeftFixed - other.transform.position);

            }
            else
            {
                // Two types of objects: Dynamic and Static
                // 1. Dynamic: If the object can move, we update always to the closest position for convenience
                // 2. If the object is rigid (like a wall), to the updates when we are far from the fixed position
                if (other.CompareTag("Dynamic Obstacle"))
                {
                    // Create an offset, in case it is necessary
                    Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);

                    // Two modes: First one is experimental
                    if (fixHandToDynamicObject)
                    {
                        hitLeftFixed = other.transform.position + localHitLeftFixed + offset;
                    }
                    else
                    {
                        // During the contact, we update constantly to the hit
                        // If the system detects a change of obstacle, we update the bools, that will induce again the smooth transition
                        if (!colliderChanged && !hasLeftStartedMovingIn)
                        {
                            Debug.Log("[TEST] Fixing to pose because colliderChanged: " + colliderChanged + " and hasLeftStartedMovingIn: " + hasLeftStartedMovingIn);
                            hitLeftFixed = hitLeft + offset;
                        }
                        else if (colliderChanged)
                        {
                            // Start moving to the target
                            hasLeftStartedMovingIn = true;
                            hasLeftStartedMovingOut = false;
                            hasLeftTargetReached = false;
                            Debug.Log("[TEST] 2) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);
                        }
                    }
                }
                else if (other.CompareTag("Static Obstacle"))
                {
                    if (!isLeftInRange)
                    {
                        Debug.Log("WALL UPDATE LEFT");
                        handRB.freezeRotation = false;

                        Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);
                        leftTarget.SetTargetUpdate(hitLeftFixed, offset, 0.5f); // TODO: Time that takes to make the small jump
                    }
                    else
                    {
                        Debug.Log("NOW FIX HAND");
                        handRB.freezeRotation = true;
                    }
                }
            }

            // =====

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray)
            if (Physics.Raycast(raycastOriginLeft, (hitLeftFixed - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                hitNormalLeft = hit.normal;

                if (drawIK)
                {
                    Debug.DrawRay(raycastOriginLeft, (hitLeftFixed - originRegion.position), Color.blue);
                    Debug.DrawRay(hitLeftFixed, hitNormalLeft * 0.2f, Color.red);
                }

                // Method in charge of moving behaviour, which changes if we are on the move or we arrived
                // hasLeftStartedMovingIn will be TRUE until we reach the object
                leftTarget.SetTargetStay(reactionTime, hasLeftStartedMovingIn);
            }

            // Allow IK from this point
            leftTarget.activateIK = true;

            // Once we have finished, we update the variables
            hasLeftStartedMovingOut = false;
            if (hasLeftTargetReached)
                hasLeftStartedMovingIn = false;

            Debug.Log("[TEST] 3) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);

            colliderChanged = false;
            Debug.Log("[TEST] colliderChanged: " + colliderChanged);
        }
    }

    // When an object exits in the Safety Region
    private void OnTriggerExit(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            // TEST - Setting the spring based on the reaction time
            if (modifySpring)
                StartCoroutine(DecreaseSpring());

            //Debug.Log("[TEST] Obstacle EXITS LEFT: " + other.name);

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
                hasLeftStartedMovingIn = false;
                hasLeftStartedMovingOut = true;
                hasLeftTargetReached = false;
                Debug.Log("[TEST] 4) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargeReached: " + hasLeftTargetReached);

                // Method in charge of moving back the hand
                leftTarget.SetTargetBack(reactionTime, hasLeftStartedMovingOut);
            }
        }
    }

    // Extension Colliders - New

    void OnEnable()
    {
        // Safety 2
        onTriggerEventLeft2.onTriggerEnter.AddListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventLeft2.onTriggerStay.AddListener(OnTheOtherTriggerStayMethod);
        onTriggerEventLeft2.onTriggerExit.AddListener(OnTheOtherTriggerExitMethod);

        // Safety 3
        onTriggerEventLeft3.onTriggerEnter.AddListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventLeft3.onTriggerStay.AddListener(OnTheOtherTriggerStayMethod);
        onTriggerEventLeft3.onTriggerExit.AddListener(OnTheOtherTriggerExitMethod);
    }

    void OnDisable()
    {
        // Safety 2
        onTriggerEventLeft2.onTriggerEnter.RemoveListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventLeft2.onTriggerStay.RemoveListener(OnTheOtherTriggerStayMethod);
        onTriggerEventLeft2.onTriggerExit.RemoveListener(OnTheOtherTriggerExitMethod);

        // Safety 3
        onTriggerEventLeft3.onTriggerEnter.RemoveListener(OnTheOtherTriggerEnterMethod);
        onTriggerEventLeft3.onTriggerStay.RemoveListener(OnTheOtherTriggerStayMethod);
        onTriggerEventLeft3.onTriggerExit.RemoveListener(OnTheOtherTriggerExitMethod);
    }

    void OnTheOtherTriggerEnterMethod(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[INFO] New obstacle ENTERS LEFT: " + other.name);

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginLeft = originRegion.position;

            // Create an offset, in case it is necessary
            Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);

            // For the first obstacle, the list is empty at this point
            if (obstacles.Count == 0)
            {
                // Start moving to the target
                hasLeftStartedMovingIn = true;
                hasLeftStartedMovingOut = false;
                hasLeftTargetReached = false;
                Debug.Log("[TEST] 1) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargetReached: " + hasLeftTargetReached);
            }

            // Add new obstacle to the dynamic list
            Vector3 closestPoint = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation) + offset;

            // TODO: Retrieve mass
            if (other.gameObject.GetComponent<Rigidbody>())
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginLeft), other.gameObject.GetComponent<ObstacleDynamics>().realMass, other.gameObject.GetComponent<ObstacleDynamics>().expectedMass, other.gameObject.GetComponent<ObstacleDynamics>().realVelocity, other.gameObject.GetComponent<ObstacleDynamics>().expectedVelocity));
            else
                obstacles.Add(new Obstacle(other, closestPoint, Vector3.Distance(closestPoint, raycastOriginLeft), other.GetComponentInParent<ObstacleDynamics>().realMass, other.GetComponentInParent<ObstacleDynamics>().expectedMass, other.GetComponentInParent<ObstacleDynamics>().realVelocity, other.GetComponentInParent<ObstacleDynamics>().expectedVelocity));

        }
    }

    void OnTheOtherTriggerStayMethod(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[INFO] Obstacle STAYS LEFT: " + other.name);

            // We protect the origin, and get the closest point in the external object to the previous body part to protect
            raycastOriginLeft = originRegion.position;

            // Take the instance of the obstacle that corresponds, and update at each time the closest point and distance to it
            Obstacle currentObstacle = obstacles.Find(x => x.obstacle == other);
            Vector3 closestPoint = Physics.ClosestPoint(raycastOriginLeft, other, other.transform.position, other.transform.rotation);
            currentObstacle.location = closestPoint;
            currentObstacle.distance = Vector3.Distance(closestPoint, raycastOriginLeft);

            // In case of multiple objects inside the region, we take the closest one -> TODO Can be improved
            var min = 100f;
            for (int i = 0; i < obstacles.Count; i++)
            {
                if (min > obstacles[i].distance)
                {
                    // We update the closest obstacle
                    targetObstacle = obstacles[i]; // To provide to the Antagonistic Gains
                    nowObstacle = obstacles[i].obstacle;
                    hitLeft = obstacles[i].location;
                    min = obstacles[i].distance;
                }
            }

            // If we update the obstacle for a new one, we make colliderChanged TRUE
            if (prevObstacle != nowObstacle)
            {
                colliderChanged = true;
                Debug.Log("[TEST] colliderChanged LEFT: " + colliderChanged);
                prevObstacle = nowObstacle;
            }

            // Here, we already have the hitLeft that corresponds to the closest point to the origin
            // Now, we need to take hitLeftFixed and updated with the constantly tracked value of hitLeft when is convenient

            // =====

            // When do we update?
            // 1. We update constantly to the hit if we did not reach yet the position or if we stop being in contact when the distance to the hit is larger than the arm itself
            // 2. Otherwise, hasLeftStartedMovingIn is false
            if (hasLeftStartedMovingIn)
            {
                // Create an offset, in case it is necessary
                Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);

                Debug.Log("[TEST] Fixing to pose because hasLeftStartedMovingIn: " + hasLeftStartedMovingIn);
                hitLeftFixed = hitLeft + offset;
                localHitLeftFixed = (hitLeftFixed - other.transform.position);

            }
            else
            {
                // Two types of objects: Dynamic and Static
                // 1. Dynamic: If the object can move, we update always to the closest position for convenience
                // 2. If the object is rigid (like a wall), to the updates when we are far from the fixed position
                if (other.CompareTag("Dynamic Obstacle"))
                {
                    // Create an offset, in case it is necessary
                    Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);

                    // Two modes: First one is experimental
                    if (fixHandToDynamicObject)
                    {
                        hitLeftFixed = other.transform.position + localHitLeftFixed + offset;
                    }
                    else
                    {
                        // During the contact, we update constantly to the hit
                        // If the system detects a change of obstacle, we update the bools, that will induce again the smooth transition
                        if (!colliderChanged && !hasLeftStartedMovingIn)
                        {
                            Debug.Log("[TEST] Fixing to pose because colliderChanged: " + colliderChanged + " and hasLeftStartedMovingIn: " + hasLeftStartedMovingIn);
                            hitLeftFixed = hitLeft + offset;
                        }
                        else if (colliderChanged)
                        {
                            // Start moving to the target
                            hasLeftStartedMovingIn = true;
                            hasLeftStartedMovingOut = false;
                            hasLeftTargetReached = false;
                            Debug.Log("[TEST] 2) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargetReached: " + hasLeftTargetReached);
                        }
                    }
                }
                else if (other.CompareTag("Static Obstacle"))
                {
                    if (!isLeftInRange)
                    {
                        Debug.Log("WALL UPDATE LEFT");
                        Vector3 offset = (leftTargetTransform.up * hitOffsetLeft.y) + (leftTargetTransform.right * hitOffsetLeft.x) + (leftTargetTransform.forward * hitOffsetLeft.z);
                        leftTarget.SetTargetUpdate(hitLeftFixed, offset, 0.5f); // TODO: Time that takes to make the small jump
                    }
                }
            }

            // =====

            // Launch a ray from the body part to protect, in the direction the closest point (like in the previous red ray)
            if (Physics.Raycast(raycastOriginLeft, (hitLeftFixed - originRegion.position), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Obstacle")))
            {
                hitNormalLeft = hit.normal;

                if (drawIK)
                {
                    Debug.DrawRay(raycastOriginLeft, (hitLeftFixed - originRegion.position), Color.blue);
                    Debug.DrawRay(hitLeftFixed, hitNormalLeft * 0.2f, Color.red);
                }

                // Method in charge of moving behaviour, which changes if we are on the move or we arrived
                // hasLeftStartedMovingIn will be TRUE until we reach the object
                leftTarget.SetTargetStay(reactionTime, hasLeftStartedMovingIn);
            }

            // Allow IK from this point
            leftTarget.activateIK = true;

            // Once we have finished, we update the variables
            hasLeftStartedMovingOut = false;
            if (hasLeftTargetReached)
                hasLeftStartedMovingIn = false;

            Debug.Log("[TEST] 3) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargetReached: " + hasLeftTargetReached);

            colliderChanged = false;
            Debug.Log("[TEST] colliderChanged: " + colliderChanged);
        }
    }

    void OnTheOtherTriggerExitMethod(Collider other)
    {
        if ((other.CompareTag("Dynamic Obstacle") || other.CompareTag("Static Obstacle")))
        {
            //Debug.Log("[TEST] Obstacle EXITS LEFT: " + other.name);

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
                hasLeftStartedMovingIn = false;
                hasLeftStartedMovingOut = true;
                hasLeftTargetReached = false;
                Debug.Log("[TEST] 4) hasLeftStartedMovingIn: " + hasLeftStartedMovingIn + " | hasLeftStartedMovingOut: " + hasLeftStartedMovingOut + " | hasLeftTargetReached: " + hasLeftTargetReached);

                // Method in charge of moving back the hand
                leftTarget.SetTargetBack(reactionTime, hasLeftStartedMovingOut);
            }
        }
    }

    // Hand Springs

    IEnumerator IncreaseSpring()
    {
        float timeElapsed = 0;
        while (timeElapsed < reactionTime)
        {
            leftHandSpring.spring = Mathf.Lerp(0f, 10000f, timeElapsed / reactionTime);
            timeElapsed += Time.deltaTime;
            yield return null;
        }
        leftHandSpring.spring = 10000f;
    }

    IEnumerator DecreaseSpring()
    {
        float timeElapsed = 0;
        while (timeElapsed < reactionTime)
        {
            leftHandSpring.spring = Mathf.Lerp(10000f, 0f, timeElapsed / reactionTime);
            timeElapsed += Time.deltaTime;
            yield return null;
        }
        leftHandSpring.spring = 0f;
    }
    private void OnDrawGizmos()
    {
        //Gizmos.color = Color.red;
        //Gizmos.DrawSphere(hitLeftFixed, 0.05f);
        //Gizmos.color = Color.green;
        //Gizmos.DrawSphere(hitLeft, 0.05f);
    }
}

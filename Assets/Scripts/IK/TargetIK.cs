/****************************************************
 * File: ArmsFaskIK.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class TargetIK : MonoBehaviour
{
    #region Read-only & Static Fields

    protected float[] bonesLength; // Target to Origin
    protected float completeLength;
    protected Transform[] bones;
    protected Vector3[] positions;
    protected Vector3[] startDirectionSucc;
    protected Quaternion[] startRotationBone;
    protected Quaternion startRotationTarget;
    protected Transform root;

    private static IKArmsPlacement armsIK;

    #endregion

    #region Instance Fields

    [Header("FABRIK - Settings")]
    public int chainLength = 2;
    public Transform target;
    public Transform targetConstant;
    public Transform pole;
    public int iterations = 10; // Solver iterations per update
    public float delta = 0.001f; // Distance when the solver stops
    [Range(0, 1)] public float snapBackStrength = 1f; // Strength of going back to the start position

    [Header("Safety Region - Settings")]
    public bool activateIK = false;
    public Transform kinematicBody;
    public SafetyRegionLeft safetyRegionLeft;
    public SafetyRegionRight safetyRegionRight;
    public Quaternion rotationToNormal;

    #endregion

    #region Unity Methods

    void Awake()
    {
        // Initialization for FABRIK
        Init();
    }

    void Init()
    {
        // Initial array
        bones = new Transform[chainLength + 1];
        positions = new Vector3[chainLength + 1];
        bonesLength = new float[chainLength];
        startDirectionSucc = new Vector3[chainLength + 1];
        startRotationBone = new Quaternion[chainLength + 1];

        // Find root
        root = transform;
        for (var i = 0; i <= chainLength; i++)
        {
            if (root == null)
                throw new UnityException("The chain value is longer than the ancestor chain!");
            root = root.parent;
        }

        // Init target
        if (target == null)
        {
            target = new GameObject(gameObject.name + " Target").transform;
            SetPositionRootSpace(target, GetPositionRootSpace(transform));
        }
        startRotationTarget = GetRotationRootSpace(target);


        // Init data
        var current = transform;
        completeLength = 0;
        for (var i = bones.Length - 1; i >= 0; i--)
        {
            bones[i] = current;
            startRotationBone[i] = GetRotationRootSpace(current);

            if (i == bones.Length - 1)
            {
                // Leaf
                startDirectionSucc[i] = GetPositionRootSpace(target) - GetPositionRootSpace(current);
            }
            else
            {
                // Mid bone
                startDirectionSucc[i] = GetPositionRootSpace(bones[i + 1]) - GetPositionRootSpace(current);
                bonesLength[i] = startDirectionSucc[i].magnitude;
                completeLength += bonesLength[i];
            }

            current = current.parent;
        }
    }

    private void Start()
    {
        armsIK = FindObjectOfType<IKArmsPlacement>();
    }

    private void Update()
    {
        // As long as IK is disable, we place the targets in the hands to follow the kinematic actions
        if (!activateIK)
        {
            SetTarget(target, this.transform);
            SetTarget(targetConstant, this.transform);
        }
    }

    void LateUpdate()
    {
        if (activateIK)
        {
            // We are going to replace this IK system with the one from Unity with built-in weights. The target system remains the same.
            //ResolveIK(); 
        }
    }

    #endregion

    #region Instance Methods

    /// <summary>
    /// Place the IK target in certain position and rotation.
    /// </summary>
    /// <param name="target"></param>
    /// <param name="trf"></param>
    public void SetTarget(Transform target, Transform trf)
    {
        // In rest pose, the position of each target will be the hand position, and hits rotation will be fixed to those rest rotations.
        target.transform.position = trf.transform.position;        

        if (target.CompareTag("LeftHand"))
        {
            target.transform.rotation = kinematicBody.rotation * Quaternion.Euler(new Vector3(0, 0, 180f));
        }

        if (target.CompareTag("RightHand"))
        {
            target.transform.rotation = kinematicBody.rotation * Quaternion.Euler(new Vector3(0, 0, 0));
        }
    }

    /// <summary>
    /// Place the IK target during the contact in certain position and rotation.
    /// </summary>
    /// <param name="reactionTime"></param>
    /// <param name="hasStartedMovingIn"></param>
    public void SetTargetStay(float reactionTime, bool hasStartedMovingIn)
    {
        if (hasStartedMovingIn) // We are stíll moving
        {
            StartCoroutine(MoveTarget(reactionTime));
        }
        else // We arrived
        {
            if (target.CompareTag("LeftHand"))
            {
                Vector3 forwardHit = new Vector3(-safetyRegionLeft.hitNormalLeft.z, 0, safetyRegionLeft.hitNormalLeft.x);
                Debug.DrawRay(safetyRegionLeft.hitLeftFixed, forwardHit * 0.2f, Color.green);

                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionLeft.hitNormalLeft));

                target.transform.position = safetyRegionLeft.hitLeftFixed;
                target.transform.rotation = rotationToNormal;
            }
            
            if (target.CompareTag("RightHand"))
            {
                Vector3 forwardHit = new Vector3(safetyRegionRight.hitNormalRight.z, 0, -safetyRegionRight.hitNormalRight.x);
                Debug.DrawRay(safetyRegionRight.hitRight, forwardHit * 0.2f, Color.green);

                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionRight.hitNormalRight));

                target.transform.position = safetyRegionRight.hitRightFixed;
                target.transform.rotation = rotationToNormal;
            }
        }
    }

    /// <summary>
    /// Update IK target to new position if we get to far from the original fixed position.
    /// </summary>
    /// <param name="startPos"></param>
    /// <param name="offset"></param>
    /// <param name="reactionTime"></param>
    public void SetTargetUpdate(Vector3 startPos, Vector3 offset, float reactionTime)
    {
        StartCoroutine(MoveTargetUpdate(startPos, offset, reactionTime));
    }

    /// <summary>
    /// Place the IK target back to the original position.
    /// </summary>
    /// <param name="reactionTime"></param>
    /// <param name="hasStartedMovingOut"></param>
    public void SetTargetBack(float reactionTime, bool hasStartedMovingOut)
    {
        if (hasStartedMovingOut)
        {
            StartCoroutine(MoveTargetBack(reactionTime));
        }
    }

    private void ResolveIK()
    {
        if (target == null)
            return;

        if (bonesLength.Length != chainLength)
            Init();

        //Fabric

        //  root
        //  (bone0) (bonelen 0) (bone1) (bonelen 1) (bone2)...
        //   x--------------------x--------------------x---...

        //get position
        for (int i = 0; i < bones.Length; i++)
        {
            positions[i] = GetPositionRootSpace(bones[i]);
        }

        var targetPosition = GetPositionRootSpace(target);
        var targetRotation = GetRotationRootSpace(target);

        //1st is possible to reach?
        if ((targetPosition - GetPositionRootSpace(bones[0])).sqrMagnitude >= completeLength * completeLength)
        {
            // Hand did not touch it yet
            if (target.CompareTag("LeftHand") && safetyRegionLeft.hasLeftTargetReached)
            {
                safetyRegionLeft.isLeftInRange = false;
            }

            if (target.CompareTag("RightHand") && safetyRegionRight.hasRightTargetReached)
            {
                safetyRegionRight.isRightInRange = false;
            }

            //just strech it
            var direction = (targetPosition - positions[0]).normalized;
            //set everything after root
            for (int i = 1; i < positions.Length; i++)
                positions[i] = positions[i - 1] + direction * bonesLength[i - 1];
        }
        else
        {
            // Hand did touch it 
            if (target.CompareTag("LeftHand") && safetyRegionLeft.hasLeftTargetReached)
            {
                safetyRegionLeft.isLeftInRange = true;
            }

            if (target.CompareTag("RightHand") && safetyRegionRight.hasRightTargetReached)
            {
                safetyRegionRight.isRightInRange = true;
            }

            for (int i = 0; i < positions.Length - 1; i++)
                positions[i + 1] = Vector3.Lerp(positions[i + 1], positions[i] + startDirectionSucc[i], snapBackStrength);

            for (int iteration = 0; iteration < iterations; iteration++)
            {
                //https://www.youtube.com/watch?v=UNoX65PRehA
                //back
                for (int i = positions.Length - 1; i > 0; i--)
                {
                    if (i == positions.Length - 1)
                        positions[i] = targetPosition; //set it to target
                    else
                        positions[i] = positions[i + 1] + (positions[i] - positions[i + 1]).normalized * bonesLength[i]; //set in line on distance
                }

                //forward
                for (int i = 1; i < positions.Length; i++)
                    positions[i] = positions[i - 1] + (positions[i] - positions[i - 1]).normalized * bonesLength[i - 1];

                //close enough?
                if ((positions[positions.Length - 1] - targetPosition).sqrMagnitude < delta * delta)
                    break;
            }

            // Hand is coming back 
            if (target.CompareTag("LeftHand") && safetyRegionLeft.hasLeftStartedMovingOut)
            {
                safetyRegionLeft.isLeftInRange = false;
            }

            if (target.CompareTag("RightHand") && safetyRegionRight.hasRightStartedMovingOut)
            {
                safetyRegionRight.isRightInRange = false;
            }
        }

        //move towards pole
        if (pole != null)
        {
            var polePosition = GetPositionRootSpace(pole);
            for (int i = 1; i < positions.Length - 1; i++)
            {
                var plane = new Plane(positions[i + 1] - positions[i - 1], positions[i - 1]);
                var projectedPole = plane.ClosestPointOnPlane(polePosition);
                var projectedBone = plane.ClosestPointOnPlane(positions[i]);
                var angle = Vector3.SignedAngle(projectedBone - positions[i - 1], projectedPole - positions[i - 1], plane.normal);
                positions[i] = Quaternion.AngleAxis(angle, plane.normal) * (positions[i] - positions[i - 1]) + positions[i - 1];
            }
        }

        //set position & rotation
        for (int i = 0; i < positions.Length; i++)
        {
            if (i == positions.Length - 1)
                SetRotationRootSpace(bones[i], Quaternion.Inverse(targetRotation) * startRotationTarget * Quaternion.Inverse(startRotationBone[i]));
            else
                SetRotationRootSpace(bones[i], Quaternion.FromToRotation(startDirectionSucc[i], positions[i + 1] - positions[i]) * Quaternion.Inverse(startRotationBone[i]));
            SetPositionRootSpace(bones[i], positions[i]);
        }
    }

    #endregion

    #region Coroutines

    IEnumerator MoveTarget(float moveTime)
    {
        Debug.Log("[COROUTINE] Executing MoveTarget");

        // Store the initial, current position and rotation for the interpolation.
        Vector3 startPos = target.transform.position;
        Quaternion startRot = target.transform.rotation;

        if(target.CompareTag("LeftHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                target.transform.position = Vector3.Lerp(startPos, safetyRegionLeft.hitLeftFixed, normalizedTime);

                if (activateIK)
                {
                    // TEST
                    armsIK.leftArmWeight = 1f; 
                }

                Vector3 forwardHit = new Vector3(-safetyRegionLeft.hitNormalLeft.z, 0, safetyRegionLeft.hitNormalLeft.x);
                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionLeft.hitNormalLeft));

                //Target.transform.rotation = Quaternion.Slerp(startRot, rotationToNormal, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                target.transform.rotation = rotationToNormal;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionLeft.hasLeftStartedMovingOut));

            safetyRegionLeft.hasLeftTargetReached = true;
        }

        if (target.CompareTag("RightHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                target.transform.position = Vector3.Lerp(startPos, safetyRegionRight.hitRightFixed, normalizedTime);

                if (activateIK)
                {
                    // TEST
                    armsIK.rightArmWeight = 1f;
                }

                Vector3 forwardHit = new Vector3(safetyRegionRight.hitNormalRight.z, 0, -safetyRegionRight.hitNormalRight.x);
                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionRight.hitNormalRight));

                //Target.transform.rotation = Quaternion.Slerp(startRot, rotationToNormal, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                target.transform.rotation = rotationToNormal;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionRight.hasRightStartedMovingOut));

            safetyRegionRight.hasRightTargetReached = true;
        }
    }

    IEnumerator MoveTargetUpdate(Vector3 startPos, Vector3 offset, float moveTime)
    {
        Debug.Log("[COROUTINE] Executing MoveTargetUpdate");

        if(target.CompareTag("LeftHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                Vector3 centerPoint = (startPos + safetyRegionLeft.hitLeft + offset) / 2;
                centerPoint += safetyRegionLeft.hitNormalLeft * 0.1f;

                safetyRegionLeft.hitLeftFixed = Vector3.Lerp(Vector3.Lerp(startPos, centerPoint, normalizedTime), Vector3.Lerp(centerPoint, safetyRegionLeft.hitLeft + offset, normalizedTime), normalizedTime);

                yield return null;
            }
            while (timeElapsed < moveTime);
        }

        if(target.CompareTag("RightHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                Vector3 centerPoint = (startPos + safetyRegionRight.hitRight + offset) / 2;
                centerPoint += safetyRegionRight.hitNormalRight * 0.1f;

                safetyRegionRight.hitRightFixed = Vector3.Lerp(Vector3.Lerp(startPos, centerPoint, normalizedTime), Vector3.Lerp(centerPoint, safetyRegionRight.hitRight + offset, normalizedTime), normalizedTime);

                yield return null;
            }
            while (timeElapsed < moveTime);
        }
    }

    IEnumerator MoveTargetBack(float moveTime)
    {
        Debug.Log("[COROUTINE] Executing MoveTargetBack");

        // Store the initial, current position and rotation for the interpolation.
        Vector3 startPos = target.transform.position;
        Quaternion startRot = target.transform.rotation;

        if (target.CompareTag("LeftHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                target.transform.position = Vector3.Lerp(startPos, targetConstant.transform.position, normalizedTime);

                // TEST
                armsIK.leftArmWeight = Mathf.Lerp(1f, 0f, normalizedTime);

                //Target.transform.rotation = Quaternion.Slerp(startRot, TargetConstant.transform.rotation, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                target.transform.rotation = targetConstant.transform.rotation;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionLeft.hasLeftStartedMovingIn));
        }

        if(target.CompareTag("RightHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                target.transform.position = Vector3.Lerp(startPos, targetConstant.transform.position, normalizedTime);

                // TEST
                armsIK.rightArmWeight = Mathf.Lerp(1f, 0f, normalizedTime);

                //Target.transform.rotation = Quaternion.Slerp(startRot, TargetConstant.transform.rotation, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                target.transform.rotation = targetConstant.transform.rotation;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionRight.hasRightStartedMovingIn));
        }

        // Once the coroutine finishes, deactivate IK for this hand and follow kinematic animation.
        if (target.CompareTag("LeftHand") && !safetyRegionLeft.hasLeftStartedMovingIn)
        {
            activateIK = false;
        }

        if (target.CompareTag("RightHand") && !safetyRegionRight.hasRightStartedMovingIn)
        {
            activateIK = false;
        }
    }

    #endregion

    #region Setters/Getters

    private Vector3 GetPositionRootSpace(Transform current)
    {
        if (root == null)
            return current.position;
        else
            return Quaternion.Inverse(root.rotation) * (current.position - root.position);
    }

    private void SetPositionRootSpace(Transform current, Vector3 position)
    {
        if (root == null)
            current.position = position;
        else
            current.position = root.rotation * position + root.position;
    }

    private Quaternion GetRotationRootSpace(Transform current)
    {
        //inverse(after) * before => rot: before -> after
        if (root == null)
            return current.rotation;
        else
            return Quaternion.Inverse(current.rotation) * root.rotation;
    }

    private void SetRotationRootSpace(Transform current, Quaternion rotation)
    {
        if (root == null)
            current.rotation = rotation;
        else
            current.rotation = root.rotation * rotation;
    }

#endregion

    void OnDrawGizmos()
    {
#if UNITY_EDITOR
        var current = this.transform;
        for (int i = 0; i < chainLength && current != null && current.parent != null; i++)
        {
            var scale = Vector3.Distance(current.position, current.parent.position) * 0.1f;
            Handles.matrix = Matrix4x4.TRS(current.position, Quaternion.FromToRotation(Vector3.up, current.parent.position - current.position), new Vector3(scale, Vector3.Distance(current.parent.position, current.position), scale));
            Handles.color = Color.green;
            Handles.DrawWireCube(Vector3.up * 0.5f, Vector3.one);
            current = current.parent;
        }
#endif
    }
}

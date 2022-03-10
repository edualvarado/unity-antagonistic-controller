/****************************************************
 * File: ArmsFaskIK.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 18/02/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class ArmsFastIK : MonoBehaviour
{

    #region Instance Fields

    [Header("Fast IK - Settings")]
    public int ChainLength = 2;
    public Transform Target;
    public Transform TargetConstant;
    public Transform Pole;
    public int Iterations = 10; // Solver iterations per update
    public float Delta = 0.001f; // Distance when the solver stops
    [Range(0, 1)]
    public float SnapBackStrength = 1f; // Strength of going back to the start position

    [Header("Safety Region - Settings")]
    public bool activateIK;
    public Quaternion rotationToNormal;
    public Transform kinBody;
    public SafetyRegionLeft safetyRegionLeft;
    public SafetyRegionRight safetyRegionRight;
    public Collider _col;

    #endregion

    #region Read-only & Static Fields

    protected float[] BonesLength; // Target to Origin
    protected float CompleteLength;
    protected Transform[] Bones;
    protected Vector3[] Positions;
    protected Vector3[] StartDirectionSucc;
    protected Quaternion[] StartRotationBone;
    protected Quaternion StartRotationTarget;
    protected Transform Root;

    #endregion

    void Awake()
    {
        Init();
    }

    void Init()
    {
        //initial array
        Bones = new Transform[ChainLength + 1];
        Positions = new Vector3[ChainLength + 1];
        BonesLength = new float[ChainLength];
        StartDirectionSucc = new Vector3[ChainLength + 1];
        StartRotationBone = new Quaternion[ChainLength + 1];

        //find root
        Root = transform;
        for (var i = 0; i <= ChainLength; i++)
        {
            if (Root == null)
                throw new UnityException("The chain value is longer than the ancestor chain!");
            Root = Root.parent;
        }

        //init target
        if (Target == null)
        {
            Target = new GameObject(gameObject.name + " Target").transform;
            SetPositionRootSpace(Target, GetPositionRootSpace(transform));
        }
        StartRotationTarget = GetRotationRootSpace(Target);


        //init data
        var current = transform;
        CompleteLength = 0;
        for (var i = Bones.Length - 1; i >= 0; i--)
        {
            Bones[i] = current;
            StartRotationBone[i] = GetRotationRootSpace(current);

            if (i == Bones.Length - 1)
            {
                //leaf
                StartDirectionSucc[i] = GetPositionRootSpace(Target) - GetPositionRootSpace(current);
            }
            else
            {
                //mid bone
                StartDirectionSucc[i] = GetPositionRootSpace(Bones[i + 1]) - GetPositionRootSpace(current);
                BonesLength[i] = StartDirectionSucc[i].magnitude;
                CompleteLength += BonesLength[i];
            }

            current = current.parent;
        }
    }

    private void Update()
    {
        if (!activateIK)
        {
            SetInitialTarget(this.transform);
            SetConstantTarget(this.transform);
        }
    }

    void LateUpdate()
    {
        if (activateIK)
        {
            ResolveIK();
        }
    }

    // Methods for the Safety Region //
    // ============================= //

    /// <summary>
    /// Place the target in the initial position, before executing the IK.
    /// </summary>
    /// <param name="trf"></param>
    public void SetInitialTarget(Transform trf)
    {
        // In rest pose, the position of each target will be the hand position, and hits rotation will be fixed to those rest rotations.
        Target.transform.position = trf.transform.position;        
        //Target.transform.position = Vector3.Lerp(Target.transform.position, trf.position, 0.1f * Time.deltaTime);

        if (Target.CompareTag("LeftHand"))
        {
            Target.transform.rotation = kinBody.rotation * Quaternion.Euler(new Vector3(0, 0, 180f));
        }
        
        if (Target.CompareTag("RightHand"))
        {
            Target.transform.rotation = kinBody.rotation * Quaternion.Euler(new Vector3(0, 0, 0));
        }
    }

    /// <summary>
    /// Place the target in the initial position, before executing the IK.
    /// </summary>
    /// <param name="trf"></param>
    public void SetConstantTarget(Transform trf)
    {
        // In rest pose, the position of each target will be the hand position, and hits rotation will be fixed to those rest rotations.
        TargetConstant.transform.position = trf.transform.position;
        //TargetConstant.transform.position = Vector3.Lerp(TargetConstant.transform.position, trf.position, 0.1f * Time.deltaTime);

        if (Target.CompareTag("LeftHand"))
        {
            TargetConstant.transform.rotation = kinBody.rotation * Quaternion.Euler(new Vector3(0, 0, 180f));
        }

        if (Target.CompareTag("RightHand"))
        {
            TargetConstant.transform.rotation = kinBody.rotation * Quaternion.Euler(new Vector3(0, 0, 0));
        }
    }

    /// <summary>
    /// Set Target during the contact.
    /// </summary>
    /// <param name="pos"></param>
    /// <param name="normal"></param>
    /// <param name="reactionTime"></param>
    /// <param name="hasStartedMovingIn"></param>
    /// <param name="hand"></param>
    public void SetTargetStay(float reactionTime, bool hasStartedMovingIn)
    {
        //StartCoroutine(MoveHand(reactionTime));

        if (hasStartedMovingIn)
        {
            StartCoroutine(MoveHand(reactionTime));
        }
        else
        {

            if (Target.CompareTag("LeftHand"))
            {
                Vector3 forwardHit = new Vector3(-safetyRegionLeft.hitNormalLeft.z, 0, safetyRegionLeft.hitNormalLeft.x);
                Debug.DrawRay(safetyRegionLeft.hitLeftFixed, forwardHit * 0.2f, Color.green);

                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionLeft.hitNormalLeft));

                Target.transform.position = safetyRegionLeft.hitLeftFixed;
                Target.transform.rotation = rotationToNormal;
            }
            
            if (Target.CompareTag("RightHand"))
            {
                Vector3 forwardHit = new Vector3(safetyRegionRight.hitNormalRight.z, 0, -safetyRegionRight.hitNormalRight.x);
                Debug.DrawRay(safetyRegionRight.hitRight, forwardHit * 0.2f, Color.green);

                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionRight.hitNormalRight));

                Target.transform.position = safetyRegionRight.hitRightFixed;
                Target.transform.rotation = rotationToNormal;
            }
        }
    }

    /// <summary>
    /// Update target to new position if we get to far from the original fixed position.
    /// </summary>
    /// <param name="startPos"></param>
    /// <param name="offset"></param>
    /// <param name="reactionTime"></param>
    public void SetTargetUpdate(Vector3 startPos, Vector3 offset, float reactionTime)
    {
        StartCoroutine(MoveHandUpdate(startPos, offset, reactionTime));
    }

    /// <summary>
    /// Set target back to the original position after contact.
    /// </summary>
    /// <param name="pos"></param>
    /// <param name="body"></param>
    /// <param name="rot"></param>
    /// <param name="reactionTime"></param>
    /// <param name="hasStartedMovingOut"></param>
    public void SetTargetBack(float reactionTime, bool hasStartedMovingOut)
    {
        if (hasStartedMovingOut)
        {
            StartCoroutine(MoveHandBack(reactionTime));
        }
    }

    // Coroutines //
    // ========== //
    IEnumerator MoveHand(float moveTime)
    {
        Debug.Log("[COROUTINE] Executing MoveHand");

        // Store the initial, current position and rotation for the interpolation.
        Vector3 startPos = Target.transform.position;
        Quaternion startRot = Target.transform.rotation;

        if(Target.CompareTag("LeftHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                Target.transform.position = Vector3.Lerp(startPos, safetyRegionLeft.hitLeftFixed, normalizedTime);

                Vector3 forwardHit = new Vector3(-safetyRegionLeft.hitNormalLeft.z, 0, safetyRegionLeft.hitNormalLeft.x);
                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionLeft.hitNormalLeft));

                //Target.transform.rotation = Quaternion.Slerp(startRot, rotationToNormal, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = rotationToNormal;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionLeft.hasLeftStartedMovingOut));

            safetyRegionLeft.hasLeftTargetReached = true;
        }

        if (Target.CompareTag("RightHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                Target.transform.position = Vector3.Lerp(startPos, safetyRegionRight.hitRightFixed, normalizedTime);

                Vector3 forwardHit = new Vector3(safetyRegionRight.hitNormalRight.z, 0, -safetyRegionRight.hitNormalRight.x);
                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionRight.hitNormalRight));

                //Target.transform.rotation = Quaternion.Slerp(startRot, rotationToNormal, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = rotationToNormal;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionRight.hasRightStartedMovingOut));

            safetyRegionRight.hasRightTargetReached = true;
        }
    }

    IEnumerator MoveHandUpdate(Vector3 startPos, Vector3 offset, float moveTime)
    {
        Debug.Log("[COROUTINE] Executing MoveHandUpdate");

        if(Target.CompareTag("LeftHand"))
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

        if(Target.CompareTag("RightHand"))
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

    IEnumerator MoveHandBack(float moveTime)
    {
        Debug.Log("[COROUTINE] Executing MoveHandBack");

        // Store the initial, current position and rotation for the interpolation.
        Vector3 startPos = Target.transform.position;
        Quaternion startRot = Target.transform.rotation;

        if (Target.CompareTag("LeftHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                Target.transform.position = Vector3.Lerp(startPos, TargetConstant.transform.position, normalizedTime);

                //Target.transform.rotation = Quaternion.Slerp(startRot, TargetConstant.transform.rotation, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = TargetConstant.transform.rotation;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionLeft.hasLeftStartedMovingIn));
        }

        if(Target.CompareTag("RightHand"))
        {
            // Initialize the time.
            float timeElapsed = 0;

            do
            {
                timeElapsed += Time.deltaTime;
                float normalizedTime = timeElapsed / moveTime;

                normalizedTime = Easing.EaseInOutCubic(normalizedTime);

                Target.transform.position = Vector3.Lerp(startPos, TargetConstant.transform.position, normalizedTime);

                //Target.transform.rotation = Quaternion.Slerp(startRot, TargetConstant.transform.rotation, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = TargetConstant.transform.rotation;

                yield return null;
            }
            while ((timeElapsed < moveTime) && !(safetyRegionRight.hasRightStartedMovingIn));
        }

        // Once the coroutine finishes, deactivate IK for this hand and follow kinematic animation.
        if (Target.CompareTag("LeftHand") && !safetyRegionLeft.hasLeftStartedMovingIn)
        {
            activateIK = false;
        }

        if (Target.CompareTag("RightHand") && !safetyRegionRight.hasRightStartedMovingIn)
        {
            activateIK = false;
        }
    }

    // IK //
    // == //

    private void ResolveIK()
    {
        if (Target == null)
            return;

        if (BonesLength.Length != ChainLength)
            Init();

        //Fabric

        //  root
        //  (bone0) (bonelen 0) (bone1) (bonelen 1) (bone2)...
        //   x--------------------x--------------------x---...

        //get position
        for (int i = 0; i < Bones.Length; i++)
        {
            Positions[i] = GetPositionRootSpace(Bones[i]);
        }

        var targetPosition = GetPositionRootSpace(Target);
        var targetRotation = GetRotationRootSpace(Target);

        //1st is possible to reach?
        if ((targetPosition - GetPositionRootSpace(Bones[0])).sqrMagnitude >= CompleteLength * CompleteLength)
        {
            // Hand did not touch it yet
            if (Target.CompareTag("LeftHand") && safetyRegionLeft.hasLeftTargetReached)
            {
                safetyRegionLeft.isLeftInRange = false;
            }
            
            if (Target.CompareTag("RightHand") && safetyRegionRight.hasRightTargetReached)
            {
                safetyRegionRight.isRightInRange = false;
            }

            //just strech it
            var direction = (targetPosition - Positions[0]).normalized;
            //set everything after root
            for (int i = 1; i < Positions.Length; i++)
                Positions[i] = Positions[i - 1] + direction * BonesLength[i - 1];
        }
        else
        {
            // Hand did touch it 
            if (Target.CompareTag("LeftHand") && safetyRegionLeft.hasLeftTargetReached)
            {
                safetyRegionLeft.isLeftInRange = true;
            }
            
            if (Target.CompareTag("RightHand") && safetyRegionRight.hasRightTargetReached)
            {
                safetyRegionRight.isRightInRange = true;
            }

            for (int i = 0; i < Positions.Length - 1; i++)
                Positions[i + 1] = Vector3.Lerp(Positions[i + 1], Positions[i] + StartDirectionSucc[i], SnapBackStrength);

            for (int iteration = 0; iteration < Iterations; iteration++)
            {
                //https://www.youtube.com/watch?v=UNoX65PRehA
                //back
                for (int i = Positions.Length - 1; i > 0; i--)
                {
                    if (i == Positions.Length - 1)
                        Positions[i] = targetPosition; //set it to target
                    else
                        Positions[i] = Positions[i + 1] + (Positions[i] - Positions[i + 1]).normalized * BonesLength[i]; //set in line on distance
                }

                //forward
                for (int i = 1; i < Positions.Length; i++)
                    Positions[i] = Positions[i - 1] + (Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1];

                //close enough?
                if ((Positions[Positions.Length - 1] - targetPosition).sqrMagnitude < Delta * Delta)
                    break;
            }

            // Hand is coming back 
            if (Target.CompareTag("LeftHand") && safetyRegionLeft.hasLeftStartedMovingOut)
            {
                safetyRegionLeft.isLeftInRange = false;
            }
            
            if (Target.CompareTag("RightHand") && safetyRegionRight.hasRightStartedMovingOut)
            {
                safetyRegionRight.isRightInRange = false;
            }
        }

        //move towards pole
        if (Pole != null)
        {
            var polePosition = GetPositionRootSpace(Pole);
            for (int i = 1; i < Positions.Length - 1; i++)
            {
                var plane = new Plane(Positions[i + 1] - Positions[i - 1], Positions[i - 1]);
                var projectedPole = plane.ClosestPointOnPlane(polePosition);
                var projectedBone = plane.ClosestPointOnPlane(Positions[i]);
                var angle = Vector3.SignedAngle(projectedBone - Positions[i - 1], projectedPole - Positions[i - 1], plane.normal);
                Positions[i] = Quaternion.AngleAxis(angle, plane.normal) * (Positions[i] - Positions[i - 1]) + Positions[i - 1];
            }
        }

        //set position & rotation
        for (int i = 0; i < Positions.Length; i++)
        {
            if (i == Positions.Length - 1)
                SetRotationRootSpace(Bones[i], Quaternion.Inverse(targetRotation) * StartRotationTarget * Quaternion.Inverse(StartRotationBone[i]));
            else
                SetRotationRootSpace(Bones[i], Quaternion.FromToRotation(StartDirectionSucc[i], Positions[i + 1] - Positions[i]) * Quaternion.Inverse(StartRotationBone[i]));
            SetPositionRootSpace(Bones[i], Positions[i]);
        }
    }

    private Vector3 GetPositionRootSpace(Transform current)
    {
        if (Root == null)
            return current.position;
        else
            return Quaternion.Inverse(Root.rotation) * (current.position - Root.position);
    }

    private void SetPositionRootSpace(Transform current, Vector3 position)
    {
        if (Root == null)
            current.position = position;
        else
            current.position = Root.rotation * position + Root.position;
    }

    private Quaternion GetRotationRootSpace(Transform current)
    {
        //inverse(after) * before => rot: before -> after
        if (Root == null)
            return current.rotation;
        else
            return Quaternion.Inverse(current.rotation) * Root.rotation;
    }

    private void SetRotationRootSpace(Transform current, Quaternion rotation)
    {
        if (Root == null)
            current.rotation = rotation;
        else
            current.rotation = Root.rotation * rotation;
    }  

    void OnDrawGizmos()
    {
#if UNITY_EDITOR
        var current = this.transform;
        for (int i = 0; i < ChainLength && current != null && current.parent != null; i++)
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

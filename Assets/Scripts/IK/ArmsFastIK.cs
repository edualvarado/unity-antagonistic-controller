using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class ArmsFastIK : MonoBehaviour
{
    /// <summary>
    /// Chain length of bones
    /// </summary>
    public int ChainLength = 2;

    /// <summary>
    /// Target the chain should bent to
    /// </summary>
    public Transform Target;
    public Transform TargetConstant;
    public Transform Pole;

    /// <summary>
    /// Solver iterations per update
    /// </summary>
    [Header("Solver Parameters")]
    public int Iterations = 10;

    /// <summary>
    /// Distance when the solver stops
    /// </summary>
    public float Delta = 0.001f;

    /// <summary>
    /// Strength of going back to the start position.
    /// </summary>
    [Range(0, 1)]
    public float SnapBackStrength = 1f;

    protected float[] BonesLength; //Target to Origin
    protected float CompleteLength;
    protected Transform[] Bones;
    protected Vector3[] Positions;
    protected Vector3[] StartDirectionSucc;
    protected Quaternion[] StartRotationBone;
    protected Quaternion StartRotationTarget;
    protected Transform Root;

    // NEW
    [Header("Safety Region")]
    public bool activateIK;
    public Quaternion rotationToNormal;
    public Transform kinBody;
    public SafetyRegionLeft safetyRegionLeft;
    public SafetyRegionRight safetyRegionRight;

    // Start is called before the first frame update
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

    // Update is called once per frame
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

        if (Target.CompareTag("LeftHand"))
        {
            Target.transform.rotation = kinBody.rotation * Quaternion.Euler(new Vector3(0, 0, 180f));
        }
        else if (Target.CompareTag("RightHand"))
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
        TargetConstant.transform.position = trf.transform.position;

        if (Target.CompareTag("LeftHand"))
        {
            TargetConstant.transform.rotation = kinBody.rotation * Quaternion.Euler(new Vector3(0, 0, 180f));
        }
        else if (Target.CompareTag("RightHand"))
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
        if (hasStartedMovingIn) // For the first time, run the coroutine
            StartCoroutine(MoveHand(reactionTime));
        else
        {
            if (Target.CompareTag("LeftHand"))
            {
                Vector3 surfaceHit = new Vector3(-safetyRegionLeft.hitNormalLeft.z, 0, safetyRegionLeft.hitNormalLeft.x);
                Debug.DrawRay(safetyRegionLeft.hitLeft, surfaceHit * 0.2f, Color.green);

                if (surfaceHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(surfaceHit, Vector3.Cross(surfaceHit, safetyRegionLeft.hitNormalLeft));

                Target.transform.position = safetyRegionLeft.hitLeft;
                Target.transform.rotation = rotationToNormal;
            }
            else if (Target.CompareTag("RightHand"))
            {
                Vector3 forwardHit = new Vector3(safetyRegionRight.hitNormalRight.z, 0, -safetyRegionRight.hitNormalRight.x);
                Debug.DrawRay(safetyRegionRight.hitRight, forwardHit * 0.2f, Color.green);

                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionRight.hitNormalRight));

                Target.transform.position = safetyRegionRight.hitRight;
                Target.transform.rotation = rotationToNormal;
            }
        }
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

    /// <summary>
    /// Coroutine that performs the motion part.
    /// </summary>
    /// <param name="endPos"></param>
    /// <param name="endRot"></param>
    /// <param name="moveTime"></param>
    /// <returns></returns>
    IEnumerator MoveHand(float moveTime)
    {
        // Store the initial, current position and rotation for the interpolation.
        Vector3 startPos = Target.transform.position;
        Quaternion startRot = Target.transform.rotation;

        // Initialize the time.
        float timeElapsed = 0;

        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / moveTime;

            normalizedTime = Easing.EaseInOutCubic(normalizedTime);

            if (Target.CompareTag("LeftHand"))
            {
                Target.transform.position = Vector3.Lerp(startPos, safetyRegionLeft.hitLeft, normalizedTime);
                
                Vector3 forwardHit = new Vector3(-safetyRegionLeft.hitNormalLeft.z, 0, safetyRegionLeft.hitNormalLeft.x);
                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionLeft.hitNormalLeft));

                //Target.transform.rotation = Quaternion.Slerp(startRot, rotationToNormal, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = rotationToNormal;
            }
            else if (Target.CompareTag("RightHand"))
            {
                Target.transform.position = Vector3.Lerp(startPos, safetyRegionRight.hitRight, normalizedTime);

                Vector3 forwardHit = new Vector3(safetyRegionRight.hitNormalRight.z, 0, -safetyRegionRight.hitNormalRight.x);
                if (forwardHit != Vector3.zero)
                    rotationToNormal = Quaternion.LookRotation(forwardHit, Vector3.Cross(forwardHit, safetyRegionRight.hitNormalRight));

                //Target.transform.rotation = Quaternion.Lerp(startRot, rotationToNormal, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = rotationToNormal;
            }

            yield return null;
        }
        while ((timeElapsed < moveTime) && !(safetyRegionLeft.hasLeftStartedMovingOut));

        // Hand has arrived
        if (Target.CompareTag("LeftHand"))
        {
            safetyRegionLeft.hasLeftContact = true;
        }
    }

    IEnumerator MoveHandBack(float moveTime)
    {
        // Store the initial, current position and rotation for the interpolation.
        Vector3 startPos = Target.transform.position;
        Quaternion startRot = Target.transform.rotation;

        // Initialize the time.
        float timeElapsed = 0;

        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / moveTime;

            normalizedTime = Easing.EaseInOutCubic(normalizedTime);

            if (Target.CompareTag("LeftHand"))
            {
                Target.transform.position = Vector3.Lerp(startPos, TargetConstant.transform.position, normalizedTime);

                //Target.transform.rotation = Quaternion.Slerp(startRot, TargetConstant.transform.rotation, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = TargetConstant.transform.rotation;

            }
            else if(Target.CompareTag("RightHand"))
            {
                Target.transform.position = Vector3.Lerp(startPos, TargetConstant.transform.position, normalizedTime);

                //Target.transform.rotation = Quaternion.Slerp(startRot, TargetConstant.transform.rotation, normalizedTime);
                //Target.transform.rotation = Quaternion.Euler(new Vector3(0f, Target.transform.rotation.eulerAngles.y, 0f));

                // Not a priority, we just set the final rotation directly
                Target.transform.rotation = TargetConstant.transform.rotation;

            }

            yield return null;
        }
        while ((timeElapsed < moveTime) && !(safetyRegionLeft.hasLeftStartedMovingIn));

        // Once the coroutine finishes, deactivate IK for this hand and follow kinematic animation.
        if(!safetyRegionLeft.hasLeftStartedMovingIn)
        {
            activateIK = false;
        }
    }

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
            Positions[i] = GetPositionRootSpace(Bones[i]);

        var targetPosition = GetPositionRootSpace(Target);
        var targetRotation = GetRotationRootSpace(Target);

        //1st is possible to reach?
        if ((targetPosition - GetPositionRootSpace(Bones[0])).sqrMagnitude >= CompleteLength * CompleteLength)
        {
            //just strech it
            var direction = (targetPosition - Positions[0]).normalized;
            //set everything after root
            for (int i = 1; i < Positions.Length; i++)
                Positions[i] = Positions[i - 1] + direction * BonesLength[i - 1];
        }
        else
        {
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

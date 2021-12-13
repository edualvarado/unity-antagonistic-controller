using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InterpolationSkeletonsV2 : MonoBehaviour
{
    // --- NEW --- //

    #region Classes

    private TerrainMaster _terrain;
    private SetSkeletonsV2 _setSkeletons;

    #endregion

    #region Interpolation

    [Header("Blending Factor")]
    public bool anchorInterpolation = false;
    public bool interpolateLeftArm = false;
    public bool interpolateRightArm = false;
    public bool lowerInterpolation = false;
    public bool fixArmsToKinematic = false;
    [Range(0, 1f)] public float blendingFactorSpine = 0f;

    #endregion

    #region Bones

    // Lists of bones ranges
    [Header("Upper Bones")]
    //public Transform[] variableKinematicUpperBones = new Transform[SetSkeletons.UpperPartLength];
    //public Transform[] variablePhysicalUpperBones = new Transform[SetSkeletons.UpperPartLength];
    //public Transform[] variableInterpolatedUpperBones = new Transform[SetSkeletons.UpperPartLength];

    [Header("Lower Bones")]
    //public Transform[] variableKinematicLowerBones = new Transform[SetSkeletons.LowerPartLength];
    //public Transform[] variablePhysicalLowerBones = new Transform[SetSkeletons.LowerPartLength];
    //public Transform[] variableInterpolatedLowerBones = new Transform[SetSkeletons.LowerPartLength];

    #endregion

    #region Transforms

    // Affine Transformations
    private AffineTransform affine_rt_from;
    private AffineTransform affine_rt_to;
    private AffineTransform T;

    #endregion

    // Start is called before the first frame update
    void Start()
    {
        // From other GameObjects
        _terrain = FindObjectOfType<TerrainMaster>();
        _setSkeletons = FindObjectOfType<SetSkeletonsV2>();

        InitialMatch();

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        InitialMatch();
        BlendingKeyLowerBodyVariableAnchorMethod();
        BlendingKeyUpperBodyVariableAnchorMethod();

        if (fixArmsToKinematic)
            FixArms();
    }

    private void InitialMatch()
    {
        // To match initially the kinematic skeleton - TODO CHECK BELOW OR ABOVE
        for (int i = 0; i < SetSkeletonsV2.AboveAnchorLength; i++)
        {
            _setSkeletons.interpolatedAboveAnchorBones[i].position = _setSkeletons.kinematicAboveAnchorBones[i].position;
            _setSkeletons.interpolatedAboveAnchorBones[i].rotation = _setSkeletons.kinematicAboveAnchorBones[i].rotation;
        }
    }

    private void BlendingKeyUpperBodyVariableAnchorMethod()
    {
        if (anchorInterpolation)
        {
            for (int i = 0; i < SetSkeletonsV2.AboveAnchorLength; i++)
            {
                Debug.Log("[UPPER INT] Interpolatation rot of " + _setSkeletons.interpolatedAboveAnchorBones[i].name);

                affine_rt_from.rotation = _setSkeletons.physicalAboveAnchorBones[i].rotation;
                affine_rt_to.rotation = _setSkeletons.kinematicAboveAnchorBones[i].rotation;

                T.rotation = Quaternion.Slerp(affine_rt_from.rotation, affine_rt_to.rotation, blendingFactorSpine);
                _setSkeletons.interpolatedAboveAnchorBones[i].rotation = T.rotation;

            }
        }

        if(interpolateLeftArm)
        {
            for (int i = 0; i < SetSkeletonsV2.bonesLeftArm; i++)
            {
                Debug.Log("[LEFT ARM INT] Interpolatation rot of " + _setSkeletons.interpolatedLeftArmsBones[i].name);

                affine_rt_from.rotation = _setSkeletons.physicalLeftArmsBones[i].rotation;
                affine_rt_to.rotation = _setSkeletons.kinematicLeftArmsBones[i].rotation;

                T.rotation = Quaternion.Slerp(affine_rt_from.rotation, affine_rt_to.rotation, blendingFactorSpine);
                _setSkeletons.interpolatedLeftArmsBones[i].rotation = T.rotation;

            }
        }

        if (interpolateRightArm)
        {
            for (int i = 0; i < SetSkeletonsV2.bonesRightArm; i++)
            {
                Debug.Log("[LEFT ARM INT] Interpolatation rot of " + _setSkeletons.interpolatedRightArmsBones[i].name);

                affine_rt_from.rotation = _setSkeletons.physicalRightArmsBones[i].rotation;
                affine_rt_to.rotation = _setSkeletons.kinematicRightArmsBones[i].rotation;

                T.rotation = Quaternion.Slerp(affine_rt_from.rotation, affine_rt_to.rotation, blendingFactorSpine);
                _setSkeletons.interpolatedRightArmsBones[i].rotation = T.rotation;

            }
        }
    }

    private void BlendingKeyLowerBodyVariableAnchorMethod()
    {
        if (lowerInterpolation)
        {
            for (int i = 0; i < SetSkeletonsV2.BelowAnchorLength; i++)
            {
                Debug.Log("[LOWER INT] Fixing pos and rot of " + _setSkeletons.interpolatedBelowAnchorBones[i].name);

                _setSkeletons.interpolatedBelowAnchorBones[i].position = _setSkeletons.kinematicBelowAnchorBones[i].position;
                _setSkeletons.interpolatedBelowAnchorBones[i].rotation = _setSkeletons.kinematicBelowAnchorBones[i].rotation;
            }
        }
    }

    private void FixArms()
    {
        // Left Arm
        for (int i = 0; i < SetSkeletonsV2.bonesLeftArm; i++)
        {
            //_setSkeletons.interpolatedLeftArmsBones[i].position = _setSkeletons.kinematicLeftArmsBones[i].position;
            _setSkeletons.interpolatedLeftArmsBones[i].rotation = _setSkeletons.kinematicLeftArmsBones[i].rotation;
        }

        // Right Arm
        for (int i = 0; i < SetSkeletonsV2.bonesRightArm; i++)
        {
            //_setSkeletons.interpolatedRightArmsBones[i].position = _setSkeletons.kinematicRightArmsBones[i].position;
            _setSkeletons.interpolatedRightArmsBones[i].rotation = _setSkeletons.kinematicRightArmsBones[i].rotation;
        }
    }

}

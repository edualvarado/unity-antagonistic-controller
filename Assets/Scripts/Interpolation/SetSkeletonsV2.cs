/****************************************************
 * File: SetSkeletons.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Hybrid Blending Animations
   * Last update: 01/08/2021
*****************************************************/

// TODO: Organize/clean/comment code
// TODO: If bones are taking from interpolation, the only purpose of this right now is matching/sync options

using System.Collections.Generic;
using UnityEngine;

public class SetSkeletonsV2 : MonoBehaviour
{
    #region Settings

    [Header("Settings")]
    public AnchorPoint anchor;
    //public bool syncLowerBody = false;
    public bool syncBelowAnchor = false;
    public bool syncRootsPositions = false;
    public bool syncRootsRotations = false;
    public bool syncHipConnector = false;
    public bool syncHipsBones = false;

    #endregion

    #region Physical Anchors

    [Header("Physical Anchors")]
    public bool isHipsConnected = false;
    public bool isSpine1Connected = false;
    public bool isSpine2Connected = false;
    public bool isHeadConnected = false;

    public GameObject hipsConnector;
    public GameObject spine1Connector;
    public GameObject spine2Connector;
    public GameObject headConnector;
    public GameObject leftArmConnector;
    public GameObject leftForeArmConnector;
    public GameObject rightArmConnector;
    public GameObject rightForeArmConnector;
    public FixedJoint spine1Joint;
    public FixedJoint spine2Joint;
    public FixedJoint headJoint;
    public FixedJoint leftArmJoint;
    public FixedJoint leftForeArmJoint;
    public FixedJoint rightArmJoint;
    public FixedJoint rightForeArmJoint;

    #endregion

    #region Skeletons

    [Header("Info Bones")]
    public static int belowAnchorLength;
    public static int aboveAnchorLength;

    [Header("Kinematic Skeleton (Blue)")]
    public Transform rootKinematicSkeleton; // Division or root of the skeleton (Hips in the default case)
    //public GameObject kinematicSpine;
    //public GameObject kinematicUpperLeftLeg;
    //public GameObject kinematicUpperRightLeg;
    //public GameObject kinematicLeftShoulder;
    //public GameObject kinematicRightShoulder;
    public List<Transform> kinematicBelowAnchorBones = new List<Transform>();
    public List<Transform> kinematicAboveAnchorBones = new List<Transform>();

    [Header("Physical/Ragdoll Skeleton (Green)")]
    public Transform rootPhysicalSkeleton; // Division or root of the skeleton (Hips in the default case)
    //public GameObject physicalSpine;
    public List<Transform> physicalBelowAnchorBones = new List<Transform>();
    public List<Transform> physicalAboveAnchorBones = new List<Transform>();

    [Header("Interpolated Skeleton (Purple)")]
    public Transform rootInterpolatedSkeleton = null;  // Division or root of the skeleton (Hips in the default case)
    //public GameObject interpolatedSpine = null;
    public List<Transform> interpolatedBelowAnchorBones = new List<Transform>();
    public List<Transform> interpolatedAboveAnchorBones = new List<Transform>();

    [Header("Arms")]
    public List<Transform> kinematicLeftArmsBones = new List<Transform>();
    public List<Transform> kinematicRightArmsBones = new List<Transform>();
    public List<Transform> physicalLeftArmsBones = new List<Transform>();
    public List<Transform> physicalRightArmsBones = new List<Transform>();
    public List<Transform> interpolatedLeftArmsBones = new List<Transform>();
    public List<Transform> interpolatedRightArmsBones = new List<Transform>();

    [Header("Hips Connector")]
    public GameObject hipConnector;

    #endregion

    #region Anchor

    public enum AnchorPoint
    {
        Hips,
        //Spine,
        Chest,
        UpperChest,
        //Neck,
        Head,

        //LeftShoulder,
        LeftUpperArm,
        LeftLowerArm,

        //RightShoulder,
        RightUpperArm,
        RightLowerArm
    }
    private AnchorPoint previousAnchor;

    [Header("Anchor - Debug")]
    public bool throwAnchorUpperBody;
    public bool throwAnchorLeftArm;
    public bool throwAnchorRightArm;

    #endregion

    #region Bones Lists

    [Header("Animators")]
    [SerializeField] public Animator _kinAnim = null;
    [SerializeField] public Animator _phyAnim = null;
    [SerializeField] public Animator _intpAnim = null;

    /// <summary>
    /// Creates an array instance of the enum HumanBodyBones with all the important spine bones we need to use.
    /// This is usefull to select which bones (and in which order) want to interpolate.
    /// </summary>
    public static HumanBodyBones[] humanUpperBody = new HumanBodyBones[]
    {
            HumanBodyBones.Hips,
            HumanBodyBones.Spine,
            HumanBodyBones.Chest,
            HumanBodyBones.UpperChest,
            HumanBodyBones.Neck,
            HumanBodyBones.Head,

            HumanBodyBones.LeftShoulder,
            HumanBodyBones.LeftUpperArm,
            HumanBodyBones.LeftLowerArm,

            HumanBodyBones.RightShoulder,
            HumanBodyBones.RightUpperArm,
            HumanBodyBones.RightLowerArm
    };

    public static HumanBodyBones[] humanUpperSpine = new HumanBodyBones[]
{
            HumanBodyBones.Hips,
            HumanBodyBones.Spine,
            HumanBodyBones.Chest,
            HumanBodyBones.UpperChest,
            HumanBodyBones.Neck,
            HumanBodyBones.Head,
    };

    public static HumanBodyBones[] humanLeftArm = new HumanBodyBones[]
    {
            HumanBodyBones.LeftShoulder,
            HumanBodyBones.LeftUpperArm,
            HumanBodyBones.LeftLowerArm
    };

    public static HumanBodyBones[] humanRightArm = new HumanBodyBones[]
    {
            HumanBodyBones.RightShoulder,
            HumanBodyBones.RightUpperArm,
            HumanBodyBones.RightLowerArm
    };

    // Lengths for the previous lists
    public readonly static int bonesSpine = humanUpperSpine.Length;
    public readonly static int bonesLeftArm = humanLeftArm.Length;
    public readonly static int bonesRightArm = humanRightArm.Length;

    #endregion

    #region Readonly Properties

    public static int BelowAnchorLength
    {
        get { return belowAnchorLength; }
    }

    public static int AboveAnchorLength
    {
        get { return aboveAnchorLength; }
    }

    #endregion

    private IKFeetPlacement feetIK;

    // Awake is called when the script instance is being loaded
    void Awake()
    {
        feetIK = FindObjectOfType<IKFeetPlacement>();
        InitSkeletons();
    }

    // Start when is called the first frame update
    void Start()
    {
        // Choose at the beginning which physical anchor we have
        // OK to use at the beginning
        //ChoosePhysicalAnchor();
        InitPhysicalAnchor();
    }

    private void Update()
    {
        if (syncBelowAnchor)
        {
            // Sync below anchor
            SyncBelowAnchor();
        }

        if (syncRootsPositions)
        {
            SyncRootsPositions();
        }

        if (syncRootsRotations)
        {
            SyncRootsRotations();
        }

        if (syncHipConnector)
        {
            SyncHipConnector();
        }

        if (syncHipsBones)
        {
            SyncHipsBones();
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Check when anchor changes
        if (anchor != previousAnchor)
        {
            Debug.Log("[Info] CHANGE ANCHOR");
            previousAnchor = anchor;

            // Reset both skeletons to the kinematic model
            ResetInterpolatedSkeleton();
            ResetPhysicalSkeleton();
            ChoosePhysicalAnchor();

            kinematicAboveAnchorBones.Clear();
            kinematicBelowAnchorBones.Clear();
            physicalAboveAnchorBones.Clear();
            physicalBelowAnchorBones.Clear();
            interpolatedAboveAnchorBones.Clear();
            interpolatedBelowAnchorBones.Clear();

            kinematicLeftArmsBones.Clear();
            kinematicRightArmsBones.Clear();
            physicalLeftArmsBones.Clear();
            physicalRightArmsBones.Clear();
            interpolatedLeftArmsBones.Clear();
            interpolatedRightArmsBones.Clear();

            InitSkeletons();
        }

        //if (syncLowerBody)
        //{
        //    // Sync lower-body - In new model, we still need to create legs transforms
        //    SyncBonesLower(kinematicLowerBones, physicalLowerBones);
        //    SyncBonesLower(kinematicLowerBones, interpolatedLowerBones);
        //}

        /*
        if (syncBelowAnchor)
        {
            // Sync below anchor
            SyncBelowAnchor();
        }

        if (syncRootsPositions)
        {
            SyncRootsPositions();
        }

        if (syncRootsRotations)
        {
            SyncRootsRotations();
        }

        if(syncHipConnector)
        {
            SyncHipConnector();
        }

        if (syncHipsBones)
        {
            SyncHipsBones();
        }
        */
    }

    private void InitPhysicalAnchor()
    {
        Debug.Log("INIT PHYSICAL ANCHOR");

        /*
        hipsConnector.SetActive(true);
        spine1Connector.SetActive(true);
        spine2Connector.SetActive(true);
        headConnector.SetActive(true);
        */

        hipsConnector.GetComponent<Rigidbody>().isKinematic = true;
        hipsConnector.SetActive(true);

        // ==== Rest of physical anchors

        spine1Connector.GetComponent<Rigidbody>().isKinematic = false;
        spine1Connector.SetActive(false); 

        spine2Connector.GetComponent<Rigidbody>().isKinematic = false;
        spine2Connector.SetActive(false); 

        headConnector.GetComponent<Rigidbody>().isKinematic = false;
        headConnector.SetActive(false);

        // ==

        leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
        leftArmConnector.SetActive(false);

        leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
        leftForeArmConnector.SetActive(false);

        rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
        rightArmConnector.SetActive(false);

        rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
        rightForeArmConnector.SetActive(false);

    }

    private void ChoosePhysicalAnchor()
    {
        switch (Anchor2Index(anchor))
        {
            case 0:
                Debug.Log("CASE 0");
                isHipsConnected = true;
                isSpine1Connected = false;
                isSpine2Connected = false;
                isHeadConnected = false;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = false;
                spine1Connector.SetActive(false);

                spine2Connector.GetComponent<Rigidbody>().isKinematic = false;
                spine2Connector.SetActive(false);

                headConnector.GetComponent<Rigidbody>().isKinematic = false;
                headConnector.SetActive(false);

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 2:
                isHipsConnected = false;
                isSpine1Connected = true;
                isSpine2Connected = false;
                isHeadConnected = false;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null; 
                spine1Connector.SetActive(true); 
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>(); 

                spine2Connector.GetComponent<Rigidbody>().isKinematic = false;
                spine2Connector.SetActive(false);

                headConnector.GetComponent<Rigidbody>().isKinematic = false;
                headConnector.SetActive(false);

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 3:
                isHipsConnected = false;
                isSpine1Connected = false;
                isSpine2Connected = true;
                isHeadConnected = false;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null; 
                spine1Connector.SetActive(true); 
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>(); 

                spine2Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine2Joint.connectedBody = null;
                spine2Connector.SetActive(true);
                spine2Joint.connectedBody = spine2Connector.GetComponent<Rigidbody>();

                headConnector.GetComponent<Rigidbody>().isKinematic = false;
                headConnector.SetActive(false);

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 5:
                isHipsConnected = false;
                isSpine1Connected = false;
                isSpine2Connected = false;
                isHeadConnected = true;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null; 
                spine1Connector.SetActive(true); 
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>();

                spine2Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine2Joint.connectedBody = null;
                spine2Connector.SetActive(true);
                spine2Joint.connectedBody = spine2Connector.GetComponent<Rigidbody>();

                headConnector.GetComponent<Rigidbody>().isKinematic = true;
                headJoint.connectedBody = null;
                headConnector.SetActive(true);
                headJoint.connectedBody = headConnector.GetComponent<Rigidbody>();

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 7:
                isHipsConnected = false;
                isSpine1Connected = false;
                isSpine2Connected = false;
                isHeadConnected = true;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null; 
                spine1Connector.SetActive(true); 
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>(); 

                spine2Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine2Joint.connectedBody = null;
                spine2Connector.SetActive(true);
                spine2Joint.connectedBody = spine2Connector.GetComponent<Rigidbody>();

                headConnector.GetComponent<Rigidbody>().isKinematic = true;
                headJoint.connectedBody = null;
                headConnector.SetActive(true);
                headJoint.connectedBody = headConnector.GetComponent<Rigidbody>();

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftArmJoint.connectedBody = null;
                leftArmConnector.SetActive(true);
                leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);
                break;

            case 8:
                isHipsConnected = false;
                isSpine1Connected = false;
                isSpine2Connected = false;
                isHeadConnected = true;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null;
                spine1Connector.SetActive(true);
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>();

                spine2Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine2Joint.connectedBody = null;
                spine2Connector.SetActive(true);
                spine2Joint.connectedBody = spine2Connector.GetComponent<Rigidbody>();

                headConnector.GetComponent<Rigidbody>().isKinematic = true;
                headJoint.connectedBody = null;
                headConnector.SetActive(true);
                headJoint.connectedBody = headConnector.GetComponent<Rigidbody>();

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftArmJoint.connectedBody = null;
                leftArmConnector.SetActive(true);
                leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftForeArmJoint.connectedBody = null;
                leftForeArmConnector.SetActive(true);
                leftForeArmJoint.connectedBody = leftForeArmConnector.GetComponent<Rigidbody>();

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);
                break;

            case 10:
                isHipsConnected = false;
                isSpine1Connected = false;
                isSpine2Connected = false;
                isHeadConnected = true;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null;
                spine1Connector.SetActive(true);
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>();

                spine2Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine2Joint.connectedBody = null;
                spine2Connector.SetActive(true);
                spine2Joint.connectedBody = spine2Connector.GetComponent<Rigidbody>();

                headConnector.GetComponent<Rigidbody>().isKinematic = true;
                headJoint.connectedBody = null;
                headConnector.SetActive(true);
                headJoint.connectedBody = headConnector.GetComponent<Rigidbody>();

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                rightArmJoint.connectedBody = null;
                rightArmConnector.SetActive(true);
                rightArmJoint.connectedBody = rightArmConnector.GetComponent<Rigidbody>();

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);
                break;

            case 11:
                isHipsConnected = false;
                isSpine1Connected = false;
                isSpine2Connected = false;
                isHeadConnected = true;

                hipsConnector.GetComponent<Rigidbody>().isKinematic = true;

                // ==== Rest of physical anchors

                spine1Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine1Joint.connectedBody = null;
                spine1Connector.SetActive(true);
                spine1Joint.connectedBody = spine1Connector.GetComponent<Rigidbody>();

                spine2Connector.GetComponent<Rigidbody>().isKinematic = true;
                spine2Joint.connectedBody = null;
                spine2Connector.SetActive(true);
                spine2Joint.connectedBody = spine2Connector.GetComponent<Rigidbody>();

                headConnector.GetComponent<Rigidbody>().isKinematic = true;
                headJoint.connectedBody = null;
                headConnector.SetActive(true);
                headJoint.connectedBody = headConnector.GetComponent<Rigidbody>();

                // ==

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                rightArmJoint.connectedBody = null;
                rightArmConnector.SetActive(true);
                rightArmJoint.connectedBody = rightArmConnector.GetComponent<Rigidbody>();

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                rightForeArmJoint.connectedBody = null;
                rightForeArmConnector.SetActive(true);
                rightForeArmJoint.connectedBody = rightForeArmConnector.GetComponent<Rigidbody>();
                break;

            default:
                break;
        }
    }

    // ----------------------------------//

    // ================================= //
    // Creating Skeletons using division //
    // ================================= //

    /// <summary>
    /// Save both, kinematic and physical skeletons, in their respective lists.
    /// </summary>
    private void InitSkeletons()
    {
        //CreateKinematicSkeletons();
        //CreatePhysicalSkeletons();
        //CreateInterpolatedSkeleton();

        CreateKinematicSkeletonsNew();
        CreatePhysicalSkeletonsNew();
        CreateInterpolatedSkeletonsNew();

        CreateSkeletonsArms();

        aboveAnchorLength = kinematicAboveAnchorBones.Count;
        belowAnchorLength = kinematicBelowAnchorBones.Count;
    }

    /*
    /// <summary>
    /// Divide the kinematic skeleton in upper- and lower-body parts.
    /// </summary>
    private void CreateKinematicSkeletons()
    {
        FindKinematicLowerSkeleton(rootKinematicSkeleton);
        for (int i = 0; i < kinematicLowerBones.Count; i++)
        {
            FindKinematicLowerSkeleton(kinematicLowerBones[i]);
        }

        FindKinematicUpperSkeleton(kinematicUpperBones[0]);
        for (int i = 1; i < kinematicUpperBones.Count; i++)
        {
            FindKinematicUpperSkeleton(kinematicUpperBones[i]);
        }
    }

    /// <summary>
    /// Divide the physical skeleton in upper- and lower-body parts.
    /// </summary>
    private void CreatePhysicalSkeletons()
    {
        FindPhysicalLowerSkeleton(rootPhysicalSkeleton);
        for (int i = 0; i < physicalLowerBones.Count; i++)
        {
            FindPhysicalLowerSkeleton(physicalLowerBones[i]);
        }

        FindPhysicalUpperSkeleton(physicalUpperBones[0]);
        for (int i = 1; i < physicalUpperBones.Count; i++)
        {
            FindPhysicalUpperSkeleton(physicalUpperBones[i]);
        }
    }

    /// <summary>
    /// Divide the interpolated visible skeleton in upper- and lower-body parts.
    /// </summary>
    private void CreateInterpolatedSkeleton()
    {
        FindInterpolatedLowerSkeleton(rootInterpolatedSkeleton);
        for (int i = 0; i < physicalLowerBones.Count; i++)
        {
            FindInterpolatedLowerSkeleton(interpolatedLowerBones[i]);
        }

        FindInterpolatedUpperSkeleton(interpolatedUpperBones[0]);
        for (int i = 1; i < interpolatedUpperBones.Count; i++)
        {
            FindInterpolatedUpperSkeleton(interpolatedUpperBones[i]);
        }
    }
    */

    // ========================= //

    private void CreateKinematicSkeletonsNew()
    {
        FindAboveAnchorSkeleton(anchor, kinematicAboveAnchorBones, _kinAnim);
        FindBelowAnchorSkeleton(anchor, kinematicBelowAnchorBones, _kinAnim);
    }

    private void CreatePhysicalSkeletonsNew()
    {
        FindAboveAnchorSkeleton(anchor, physicalAboveAnchorBones, _phyAnim);
        FindBelowAnchorSkeleton(anchor, physicalBelowAnchorBones, _phyAnim);
    }

    private void CreateInterpolatedSkeletonsNew()
    {
        FindAboveAnchorSkeleton(anchor, interpolatedAboveAnchorBones, _intpAnim);
        FindBelowAnchorSkeleton(anchor, interpolatedBelowAnchorBones, _intpAnim);
    }

    private void CreateSkeletonsArms()
    {
        for (int i = 0; i < bonesLeftArm; i++)
        {
            kinematicLeftArmsBones.Add(_kinAnim.GetBoneTransform(humanLeftArm[i]));
            physicalLeftArmsBones.Add(_phyAnim.GetBoneTransform(humanLeftArm[i]));
            interpolatedLeftArmsBones.Add(_intpAnim.GetBoneTransform(humanLeftArm[i]));
        }

        for (int i = 0; i < bonesRightArm; i++)
        {
            kinematicRightArmsBones.Add(_kinAnim.GetBoneTransform(humanRightArm[i]));
            physicalRightArmsBones.Add(_phyAnim.GetBoneTransform(humanRightArm[i]));
            interpolatedRightArmsBones.Add(_intpAnim.GetBoneTransform(humanRightArm[i]));
        }
    }


    // ================================= //
    //      Finding bones iteratively    //
    // ================================= //

    public void FindAboveAnchorSkeleton(AnchorPoint anchor, List<Transform> aboveAnchorBones, Animator anim)
    {

        //Debug.Log("[INFO] From anchor: " + anchor + ", we go UP");

        // If the anchor is below the arms, we include the arms as well in the way up
        // If the anchor is above the arms, we do not include the arms in the way up
        // If the anchor is in one arm, take it until the end of that arm
        if ((Anchor2Bone(anchor) == HumanBodyBones.Hips) || (Anchor2Bone(anchor) == HumanBodyBones.Spine) ||
           (Anchor2Bone(anchor) == HumanBodyBones.Chest) || (Anchor2Bone(anchor) == HumanBodyBones.UpperChest))
        {
            throwAnchorUpperBody = true;
            throwAnchorLeftArm = false;
            throwAnchorRightArm = false;

            for (int i = Anchor2Index(anchor); i < bonesSpine; i++)
            {
                aboveAnchorBones.Add(anim.GetBoneTransform(humanUpperSpine[i]));
            }

            /*
            for (int i = 0; i < bonesLeftArm; i++)
            {
                upperBones.Add(anim.GetBoneTransform(humanLeftArm[i]));
            }

            for (int i = 0; i < bonesRightArm; i++)
            {
                upperBones.Add(anim.GetBoneTransform(humanRightArm[i]));
            }
            */
        }
        else if ((Anchor2Bone(anchor) == HumanBodyBones.Neck) || (Anchor2Bone(anchor) == HumanBodyBones.Head))
        {
            throwAnchorUpperBody = true;
            throwAnchorLeftArm = false;
            throwAnchorRightArm = false;

            for (int i = Anchor2Index(anchor); i < bonesSpine; i++)
            {
                aboveAnchorBones.Add(anim.GetBoneTransform(humanUpperSpine[i]));
            }
        }
        else if ((Anchor2Bone(anchor) == HumanBodyBones.LeftShoulder) || (Anchor2Bone(anchor) == HumanBodyBones.LeftUpperArm) ||
            (Anchor2Bone(anchor) == HumanBodyBones.LeftLowerArm))
        {
            throwAnchorUpperBody = false;
            throwAnchorLeftArm = true;
            throwAnchorRightArm = false;

            for (int i = Anchor2Index(anchor); i < 9; i++)
            {
                aboveAnchorBones.Add(anim.GetBoneTransform(humanUpperBody[i]));
            }
        }
        else if ((Anchor2Bone(anchor) == HumanBodyBones.RightShoulder) || (Anchor2Bone(anchor) == HumanBodyBones.RightUpperArm) ||
            (Anchor2Bone(anchor) == HumanBodyBones.RightLowerArm))
        {
            throwAnchorUpperBody = false;
            throwAnchorLeftArm = false;
            throwAnchorRightArm = true;

            for (int i = Anchor2Index(anchor); i < 12; i++)
            {
                aboveAnchorBones.Add(anim.GetBoneTransform(humanUpperBody[i]));
            }
        }
    }

    public void FindBelowAnchorSkeleton(AnchorPoint anchor, List<Transform> belowAnchorBones, Animator anim)
    {
        //Debug.Log("[INFO] From anchor: " + anchor + ", we go DOWN");

        // If the anchor is below the arms, we do not include the arms as well in the way down
        // If the anchor is above the arms, we include the arms in the way down
        // If the anchor is in one arm, take it until the rest of the body
        if ((Anchor2Bone(anchor) == HumanBodyBones.Hips) || (Anchor2Bone(anchor) == HumanBodyBones.Spine) ||
           (Anchor2Bone(anchor) == HumanBodyBones.Chest) || (Anchor2Bone(anchor) == HumanBodyBones.UpperChest))
        {
            for (int i = 0; i < Anchor2Index(anchor); i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanUpperSpine[i]));
            }
        }
        else if ((Anchor2Bone(anchor) == HumanBodyBones.Neck) || (Anchor2Bone(anchor) == HumanBodyBones.Head))
        {
            for (int i = 0; i < Anchor2Index(anchor); i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanUpperSpine[i]));
            }

            /*
            for (int i = 0; i < bonesLeftArm; i++)
            {
                lowerBones.Add(anim.GetBoneTransform(humanLeftArm[i]));
            }

            for (int i = 0; i < bonesRightArm; i++)
            {
                lowerBones.Add(anim.GetBoneTransform(humanRightArm[i]));
            }
            */
        }
        else if ((Anchor2Bone(anchor) == HumanBodyBones.LeftShoulder) || (Anchor2Bone(anchor) == HumanBodyBones.LeftUpperArm) ||
                (Anchor2Bone(anchor) == HumanBodyBones.LeftLowerArm))
        {
            for (int i = 0; i < bonesSpine; i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanUpperSpine[i]));
            }

            for (int i = 6; i < Anchor2Index(anchor); i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanUpperBody[i]));
            }

            for (int i = 0; i < bonesRightArm; i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanRightArm[i]));
            }
        }
        else if ((Anchor2Bone(anchor) == HumanBodyBones.RightShoulder) || (Anchor2Bone(anchor) == HumanBodyBones.RightUpperArm) ||
        (Anchor2Bone(anchor) == HumanBodyBones.RightLowerArm))
        {
            for (int i = 0; i < bonesSpine; i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanUpperSpine[i]));
            }

            for (int i = 0; i < bonesLeftArm; i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanLeftArm[i]));
            }

            for (int i = 9; i < Anchor2Index(anchor); i++)
            {
                belowAnchorBones.Add(anim.GetBoneTransform(humanUpperBody[i]));
            }
        }
    }

    // ========================= //

    /*
    /// <summary>
    /// Deep search thought the kinematic lower-skeleton
    /// </summary>
    /// <param name="rootKinematic"></param>
    public void FindKinematicLowerSkeleton(Transform rootKinematic)
    {
        int count = rootKinematic.childCount;
        for (int i = 0; i < count; i++)
        {
            if (rootKinematic.GetChild(i).gameObject == kinematicSpine)
            {
                kinematicUpperBones.Add(rootKinematic.GetChild(i));
            }
            else if (rootKinematic.GetChild(i).gameObject == hipConnector)
            {
                continue;
            }
            else if ((rootKinematic.GetChild(i).gameObject == feetIK.groundCheckerLeftFoot.gameObject)
                || (rootKinematic.GetChild(i).gameObject == feetIK.groundCheckerLeftFootBack.gameObject)
                || (rootKinematic.GetChild(i).gameObject == feetIK.groundCheckerRightFoot.gameObject)
                || (rootKinematic.GetChild(i).gameObject == feetIK.groundCheckerRightFootBack.gameObject))
            {
                continue;
            }
            else
            {
                kinematicLowerBones.Add(rootKinematic.GetChild(i));
            }
        }
    }

    /// <summary>
    /// Deep search thought the kinematic upper-skeleton
    /// </summary>
    /// <param name="rootKinematic"></param>
    public void FindKinematicUpperSkeleton(Transform rootKinematic)
    {
        int count = rootKinematic.childCount;
        for (int i = 0; i < count; i++)
        {
            kinematicUpperBones.Add(rootKinematic.GetChild(i));
        }
    }
    

    // ========================= //

    /// <summary>
    /// Deep search thought the physical lower-skeleton
    /// </summary>
    /// <param name="rootPhysical"></param>
    public void FindPhysicalLowerSkeleton(Transform rootPhysical)
    {
        int count = rootPhysical.childCount;
        for (int i = 0; i < count; i++)
        {
            if (rootPhysical.GetChild(i).gameObject == physicalSpine)
            {
                physicalUpperBones.Add(rootPhysical.GetChild(i));
            }
            else
            {
                physicalLowerBones.Add(rootPhysical.GetChild(i));
            }
        }
    }

    /// <summary>
    /// Deep search thought the physical upper-skeleton
    /// </summary>
    /// <param name="rootPhysical"></param>
    public void FindPhysicalUpperSkeleton(Transform rootPhysical)
    {
        int count = rootPhysical.childCount;
        for (int i = 0; i < count; i++)
        {
            physicalUpperBones.Add(rootPhysical.GetChild(i));
        }
    }

    // ========================= //

    /// <summary>
    /// Deep search thought the interpolated visible lower-skeleton
    /// </summary>
    /// <param name="rootInterpolated"></param>
    public void FindInterpolatedLowerSkeleton(Transform rootInterpolated)
    {
        int count = rootInterpolated.childCount;
        for (int i = 0; i < count; i++)
        {
            if (rootInterpolated.GetChild(i).gameObject == interpolatedSpine)
            {
                interpolatedUpperBones.Add(rootInterpolated.GetChild(i));
            }
            else
            {
                interpolatedLowerBones.Add(rootInterpolated.GetChild(i));
            }
        }
    }

    /// <summary>
    /// Deep search thought the interpolated visible upper skeleton
    /// </summary>
    /// <param name="rootIntepolated"></param>
    public void FindInterpolatedUpperSkeleton(Transform rootIntepolated)
    {
        int count = rootIntepolated.childCount;
        for (int i = 0; i < count; i++)
        {
            interpolatedUpperBones.Add(rootIntepolated.GetChild(i));
        }
    }
    */

    // =========== //
    //     Sync    //
    // =========== //

    /// <summary>
    /// Synchronizes bones.
    /// </summary>
    /// <param name="toBones"></param>
    /// <param name="fromBones"></param>
    public void SyncBonesLower(List<Transform> toBones, List<Transform> fromBones)
    {
        int idx = 0;
        foreach (Transform trf in fromBones)
        {
            trf.position = toBones[idx].position;
            trf.rotation = toBones[idx].rotation;
            idx++;
        }
    }

    /// <summary>
    /// Synchronizes bones. NOT WORKING PROPERLY
    /// </summary>
    /// <param name="toBones"></param>
    /// <param name="fromBones"></param>
    public void SyncBelowAnchor()
    {
        for (int i = 0; i < SetSkeletonsV2.AboveAnchorLength; i++)
        {
            Debug.Log("[Physical Fix] Fixing pos and rot of " + physicalBelowAnchorBones[i].name);

            physicalBelowAnchorBones[i].position = kinematicBelowAnchorBones[i].position;
            physicalBelowAnchorBones[i].rotation = kinematicBelowAnchorBones[i].rotation;
        }
    }

    private void SyncRootsPositions()
    {
        // Should we fix by hard-code the root and first spine bone for both skeletons? -> Produce vibrations

        // Fix Positions
        rootPhysicalSkeleton.position = rootKinematicSkeleton.position;
        rootInterpolatedSkeleton.position = rootKinematicSkeleton.position;
    }

    private void SyncRootsRotations()
    {
        // Should we fix by hard-code the root and first spine bone for both skeletons? -> Produce vibrations

        // Fix Rotation
        rootPhysicalSkeleton.rotation = rootKinematicSkeleton.rotation;
        rootInterpolatedSkeleton.rotation = rootKinematicSkeleton.rotation;

        rootPhysicalSkeleton.position = hipConnector.transform.position;
        rootPhysicalSkeleton.rotation = hipConnector.transform.rotation;
    }

    private void SyncHipConnector()
    {
        //Attach to hips connector with this is noKinematic
        rootInterpolatedSkeleton.position = hipConnector.transform.position;
        //rootInterpolatedSkeleton.rotation = hipConnector.transform.rotation;
    }

    private void SyncHipsBones()
    {
        //Attach to hips bones

        Debug.Log("SYNC HIPS");
        rootPhysicalSkeleton.position = rootKinematicSkeleton.position;
        rootInterpolatedSkeleton.position = rootKinematicSkeleton.position;

    }

    private void ResetInterpolatedSkeleton()
    {
        // To match initially the kinematic skeleton
        for (int i = 0; i < belowAnchorLength; i++)
        {
            interpolatedBelowAnchorBones[i].position = kinematicBelowAnchorBones[i].position;
            interpolatedBelowAnchorBones[i].rotation = kinematicBelowAnchorBones[i].rotation;
        }

        for (int i = 0; i < aboveAnchorLength; i++)
        {
            interpolatedAboveAnchorBones[i].position = kinematicAboveAnchorBones[i].position;
            interpolatedAboveAnchorBones[i].rotation = kinematicAboveAnchorBones[i].rotation;
        }
    }

    private void ResetPhysicalSkeleton()
    {
        // To match initially the kinematic skeleton
        for (int i = 0; i < belowAnchorLength; i++)
        {
            physicalBelowAnchorBones[i].position = kinematicBelowAnchorBones[i].position;
            physicalBelowAnchorBones[i].rotation = kinematicBelowAnchorBones[i].rotation;
        }

        for (int i = 0; i < aboveAnchorLength; i++)
        {
            physicalAboveAnchorBones[i].position = kinematicAboveAnchorBones[i].position;
            physicalAboveAnchorBones[i].rotation = kinematicAboveAnchorBones[i].rotation;
        }

        physicalLeftArmsBones[1].position = kinematicLeftArmsBones[1].position;
        physicalRightArmsBones[1].position = kinematicRightArmsBones[1].position;


    }

    // =============== //

    // ================= //
    //     Converters    //
    // ================= //

    /// <summary>
    /// Translate bones to ints
    /// </summary>
    /// <param name="bone"></param>
    /// <returns></returns>
    public static int Bone2Index(HumanBodyBones bone)
    {
        switch (bone)
        {
            case HumanBodyBones.Hips: return 0;
            case HumanBodyBones.Spine: return 1;
            case HumanBodyBones.Chest: return 2;
            case HumanBodyBones.UpperChest: return 3;
            case HumanBodyBones.Neck: return 4;
            case HumanBodyBones.Head: return 5;

            case HumanBodyBones.LeftShoulder: return 6;
            case HumanBodyBones.LeftUpperArm: return 7;
            case HumanBodyBones.LeftLowerArm: return 8;

            case HumanBodyBones.RightShoulder: return 9;
            case HumanBodyBones.RightUpperArm: return 10;
            case HumanBodyBones.RightLowerArm: return 11;
        }

        return -1;
    }

    /// <summary>
    /// Translate ints to bones
    /// </summary>
    /// <param name="idx"></param>
    /// <returns></returns>
    public static HumanBodyBones Index2Bone(int idx)
    {
        switch (idx)
        {
            case 0: return HumanBodyBones.Hips;
            case 1: return HumanBodyBones.Spine;
            case 2: return HumanBodyBones.Chest;
            case 3: return HumanBodyBones.UpperChest;
            case 4: return HumanBodyBones.Neck;
            case 5: return HumanBodyBones.Head;

            case 6: return HumanBodyBones.LeftShoulder;
            case 7: return HumanBodyBones.LeftUpperArm;
            case 8: return HumanBodyBones.LeftLowerArm;

            case 9: return HumanBodyBones.RightShoulder;
            case 10: return HumanBodyBones.RightUpperArm;
            case 11: return HumanBodyBones.RightLowerArm;

        }
        return HumanBodyBones.Hips;
    }

    /// <summary>
    /// Transform anchor to bone
    /// </summary>
    /// <param name="anchor"></param>
    /// <returns></returns>
    public static HumanBodyBones Anchor2Bone(AnchorPoint anchor)
    {
        switch (anchor)
        {
            case AnchorPoint.Hips: return HumanBodyBones.Hips;
            //case AnchorPoint.Spine: return HumanBodyBones.Spine;
            case AnchorPoint.Chest: return HumanBodyBones.Chest;
            case AnchorPoint.UpperChest: return HumanBodyBones.UpperChest;
            //case AnchorPoint.Neck: return HumanBodyBones.Neck;
            case AnchorPoint.Head: return HumanBodyBones.Head;

            //case AnchorPoint.LeftShoulder: return HumanBodyBones.LeftShoulder;
            case AnchorPoint.LeftUpperArm: return HumanBodyBones.LeftUpperArm;
            case AnchorPoint.LeftLowerArm: return HumanBodyBones.LeftLowerArm;

            //case AnchorPoint.RightShoulder: return HumanBodyBones.RightShoulder;
            case AnchorPoint.RightUpperArm: return HumanBodyBones.RightUpperArm;
            case AnchorPoint.RightLowerArm: return HumanBodyBones.RightLowerArm;
        }
        return HumanBodyBones.Hips;
    }

    /// <summary>
    /// Transform anchor to ints
    /// </summary>
    /// <param name="anchor"></param>
    /// <returns></returns>
    public static int Anchor2Index(AnchorPoint anchor)
    {
        switch (anchor)
        {
            case AnchorPoint.Hips: return 0;
            //case AnchorPoint.Spine: return 1;
            case AnchorPoint.Chest: return 2;
            case AnchorPoint.UpperChest: return 3;
            //case AnchorPoint.Neck: return 4;
            case AnchorPoint.Head: return 5;

            //case AnchorPoint.LeftShoulder: return 6;
            case AnchorPoint.LeftUpperArm: return 7;
            case AnchorPoint.LeftLowerArm: return 8;

            //case AnchorPoint.RightShoulder: return 9;
            case AnchorPoint.RightUpperArm: return 10;
            case AnchorPoint.RightLowerArm: return 11;
        }

        return -1;
    }
}

/****************************************************
 * File: SetSkeletonsV3.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Antagonistic Controller
   * Last update: 01/08/2021
*****************************************************/

using System.Collections.Generic;
using UnityEngine;

public class SetSkeletonsV3 : MonoBehaviour
{
    #region Settings

    [Header("General - Settings")]
    public AnchorPoint anchor;
    public bool setMirror = false;
    public bool syncRootsPositions = false;
    public bool syncRootsRotations = false;
    public bool syncLeftArm = false;
    public bool syncRightArm = false;

    // JUST PROVISIONAL TO MAKE THE PICTURE
    public bool syncLegs = false;
    public Transform upperLeftLegP;
    public Transform bottomLeftLegP;
    public Transform leftFoot1P;
    public Transform leftFoot2P;
    public Transform leftFoot3P;
    public Transform upperRightLegP;
    public Transform bottomRightLegP;
    public Transform rightFoot1P;
    public Transform rightFoot2P;
    public Transform rightFoot3P;
    public Transform upperLeftLegK;
    public Transform bottomLeftLegK;
    public Transform leftFoot1K;
    public Transform leftFoot2K;
    public Transform leftFoot3K;
    public Transform upperRightLegK;
    public Transform bottomRightLegK;
    public Transform rightFoot1K;
    public Transform rightFoot2K;
    public Transform rightFoot3K;

    public bool syncLeftHand = false;
    public bool syncRightHand = false;
    public Transform leftHandP;
    public Transform rightHandP;
    public Transform leftHandK;
    public Transform rightHandK;

    [Header("Anchor - Debug")]
    public bool throwAnchorUpperBody;
    public bool throwAnchorLeftArm;
    public bool throwAnchorRightArm;

    private bool previousSetMirror;
    #endregion

    #region Skeleton

    [Header("Kinematic Skeleton (Blue) - Debug")]
    public Transform rootKinematicSkeleton; // Division or root of the skeleton (Hips in the default case)
    public List<Transform> kinematicBelowAnchorBones = new List<Transform>();
    public List<Transform> kinematicAboveAnchorBones = new List<Transform>();
    public List<Transform> kinematicLeftArmsBones = new List<Transform>();
    public List<Transform> kinematicRightArmsBones = new List<Transform>();

    [Header("Physical/Ragdoll Skeleton (Green) - Debug")]
    public Transform rootPhysicalSkeleton; // Division or root of the skeleton (Hips in the default case)
    public List<Transform> physicalBelowAnchorBones = new List<Transform>();
    public List<Transform> physicalAboveAnchorBones = new List<Transform>();
    public List<Transform> physicalLeftArmsBones = new List<Transform>();
    public List<Transform> physicalRightArmsBones = new List<Transform>();

    [Header("Interpolated Skeleton (Purple) - Debug")]
    public Transform rootInterpolatedSkeleton = null;  // Division or root of the skeleton (Hips in the default case)
    public List<Transform> interpolatedBelowAnchorBones = new List<Transform>();
    public List<Transform> interpolatedAboveAnchorBones = new List<Transform>();
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

        LeftShoulder,
        LeftUpperArm,
        LeftLowerArm,

        RightShoulder,
        RightUpperArm,
        RightLowerArm,
    }
    public AnchorPoint previousAnchor;

    #endregion

    #region Bones Lists

    [Header("Animators - Settings")]
    [SerializeField] public Animator kinematicAnim = null;
    [SerializeField] public Animator ragdollAnim = null;
    [SerializeField] public Animator interpolatedAnim = null;

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

    #region Physical Anchors

    [Header("Physical Anchors - Debug and Settings")]
    public bool isHipsConnected = false;
    public bool isSpine1Connected = false;
    public bool isSpine2Connected = false;
    public bool isHeadConnected = false;

    public GameObject hipsConnector;
    public GameObject spine1Connector;
    public GameObject spine2Connector;
    public GameObject headConnector;
    public GameObject leftShoulderConnector;
    public GameObject leftArmConnector;
    public GameObject leftForeArmConnector;
    public GameObject rightShoulderConnector;
    public GameObject rightArmConnector;
    public GameObject rightForeArmConnector;
    public FixedJoint spine1Joint;
    public FixedJoint spine2Joint;
    public FixedJoint headJoint;
    public FixedJoint leftShoulderJoint;
    public FixedJoint leftArmJoint;
    public FixedJoint leftForeArmJoint;
    public FixedJoint rightShoulderJoint;
    public FixedJoint rightArmJoint;
    public FixedJoint rightForeArmJoint;

    #endregion

    #region Readonly Properties

    public static int belowAnchorLength;
    public static int aboveAnchorLength;

    public static int BelowAnchorLength
    {
        get { return belowAnchorLength; }
    }

    public static int AboveAnchorLength
    {
        get { return aboveAnchorLength; }
    }

    #endregion

    #region Private Variables

    private IKFeetPlacement feetIK;

    #endregion

    // Awake is called when the script instance is being loaded
    void Awake()
    {
        feetIK = FindObjectOfType<IKFeetPlacement>();
        InitSkeletons();
    }

    // Start when is called the first frame update
    void Start()
    {
        ResetSystem();

        // Choose at the beginning which physical anchor we have
        InitPhysicalAnchor();
    }

    private void Update()
    {
        if (syncRootsPositions)
        {
            SyncRootsPositions();
        }

        if (syncRootsRotations)
        {
            SyncRootsRotations();
        }

        if(syncLegs)
        {
            SyncLegs();
        }

        if (syncLeftHand)
        {
            SyncLeftHand();
        }

        if (syncRightHand)
        {
            SyncRightHand();
        }

        if (syncLeftArm)
        {
            SyncLeftArm();
        }

        if (syncRightArm)
        {
            SyncRightArm();
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Check when anchor changes
        if ((anchor != previousAnchor) || (setMirror != previousSetMirror))
        {
            Debug.Log("[INFO] Anchor changed");
            previousAnchor = anchor;
            previousSetMirror = setMirror;

            ResetSystem();
        }
    }

    private void ResetSystem()
    {
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

    // ----------------------------------//

    // =============== //
    // Setting Anchors //
    // =============== //

    private void InitPhysicalAnchor()
    {
        Debug.Log("[INFO] Initialize Physical Anchor");

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

        leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
        leftShoulderConnector.SetActive(false);

        leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
        leftArmConnector.SetActive(false);

        leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
        leftForeArmConnector.SetActive(false);

        rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
        rightShoulderConnector.SetActive(false);

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
                Debug.Log("CASE 0: Hips");
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

                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftShoulderConnector.SetActive(false);

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightShoulderConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 2:
                Debug.Log("CASE 2: Chest");
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

                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftShoulderConnector.SetActive(false);

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightShoulderConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 3:
                Debug.Log("CASE 3: Upper Chest");
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

                //leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                //leftShoulderJoint.connectedBody = null;
                //leftShoulderConnector.SetActive(true);
                //leftShoulderJoint.connectedBody = leftShoulderConnector.GetComponent<Rigidbody>();
                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftShoulderConnector.SetActive(false);

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                //rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                //rightShoulderJoint.connectedBody = null;
                //rightShoulderConnector.SetActive(true);
                //rightShoulderJoint.connectedBody = rightShoulderConnector.GetComponent<Rigidbody>();
                rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightShoulderConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 5:
                Debug.Log("CASE 5: Head");
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

                // PROVISIONAL: For head, also attach both arms?

                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftShoulderConnector.SetActive(false);

                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                //leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                //leftArmJoint.connectedBody = null;
                //leftArmConnector.SetActive(true);
                //leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();

                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightShoulderConnector.SetActive(false);

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                //rightArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                //rightArmJoint.connectedBody = null;
                //rightArmConnector.SetActive(true);
                //rightArmJoint.connectedBody = rightArmConnector.GetComponent<Rigidbody>();

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 6:
                Debug.Log("CASE 6: Left Shoulder");
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

                // LEFT SHOULDER - TRUE
                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftShoulderJoint.connectedBody = null;
                leftShoulderConnector.SetActive(true);
                leftShoulderJoint.connectedBody = leftShoulderConnector.GetComponent<Rigidbody>();

                // LEFT UPPER ARM - FALSE
                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                // LEFT FORE ARM - FALSE
                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);

                if (!setMirror)
                {
                    // RIGHT SHOULDER - FALSE
                    rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                    rightShoulderConnector.SetActive(false);
                }
                else
                {
                    // RIGHT SHOULDER - TRUE
                    rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                    rightShoulderJoint.connectedBody = null;
                    rightShoulderConnector.SetActive(true);
                    rightShoulderJoint.connectedBody = rightShoulderConnector.GetComponent<Rigidbody>();
                }

                // RIGHT UPPER ARM - FALSE
                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                // RIGHT FORE ARM - FALSE
                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);
                break;

            case 7:
                Debug.Log("CASE 7: Left Upper Arm");
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

                // LEFT SHOULDER - TRUE
                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftShoulderJoint.connectedBody = null;
                leftShoulderConnector.SetActive(true);
                leftShoulderJoint.connectedBody = leftShoulderConnector.GetComponent<Rigidbody>();

                // LEFT UPPER ARM - TRUE
                leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftArmJoint.connectedBody = null;
                leftArmConnector.SetActive(true);
                leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();

                // LEFT FORE ARM - FALSE
                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);
                
                if (!setMirror)
                {
                    // RIGHT SHOULDER - FALSE
                    rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                    rightShoulderConnector.SetActive(false);

                    // RIGHT UPPER ARM - FALSE
                    rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                    rightArmConnector.SetActive(false);
                }
                else
                {
                    // RIGHT SHOULDER - TRUE
                    rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                    rightShoulderJoint.connectedBody = null;
                    rightShoulderConnector.SetActive(true);
                    rightShoulderJoint.connectedBody = rightShoulderConnector.GetComponent<Rigidbody>();

                    // RIGHT UPPER ARM - TRUE
                    rightArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                    rightArmJoint.connectedBody = null;
                    rightArmConnector.SetActive(true);
                    rightArmJoint.connectedBody = rightArmConnector.GetComponent<Rigidbody>();
                }

                // RIGHT FORE ARM - FALSE
                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                break;

            case 8:
                Debug.Log("CASE 8: Left Fore Arm");
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

                // LEFT SHOULDER - TRUE
                leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftShoulderJoint.connectedBody = null;
                leftShoulderConnector.SetActive(true);
                leftShoulderJoint.connectedBody = leftShoulderConnector.GetComponent<Rigidbody>();

                // LEFT UPPER ARM - TRUE
                leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftArmJoint.connectedBody = null;
                leftArmConnector.SetActive(true);
                leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();

                // LEFT FORE ARM - TRUE
                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                leftForeArmJoint.connectedBody = null;
                leftForeArmConnector.SetActive(true);
                leftForeArmJoint.connectedBody = leftForeArmConnector.GetComponent<Rigidbody>();

                if (!setMirror)
                {
                    // RIGHT SHOULDER - FALSE
                    rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                    rightShoulderConnector.SetActive(false);

                    // RIGHT UPPER ARM - FALSE
                    rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                    rightArmConnector.SetActive(false);

                    // RIGHT FORE ARM - FALSE
                    rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                    rightForeArmConnector.SetActive(false);
                }
                else
                {
                    // RIGHT SHOULDER - FALSE
                    rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                    rightShoulderJoint.connectedBody = null;
                    rightShoulderConnector.SetActive(true);
                    rightShoulderJoint.connectedBody = rightShoulderConnector.GetComponent<Rigidbody>();

                    // RIGHT UPPER ARM - FALSE
                    rightArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                    rightArmJoint.connectedBody = null;
                    rightArmConnector.SetActive(true);
                    rightArmJoint.connectedBody = rightArmConnector.GetComponent<Rigidbody>();

                    // RIGHT FORE ARM - FALSE
                    rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                    rightForeArmJoint.connectedBody = null;
                    rightForeArmConnector.SetActive(true);
                    rightForeArmJoint.connectedBody = rightForeArmConnector.GetComponent<Rigidbody>();
                }

                break;

            case 9:
                Debug.Log("CASE 9: Right Shoulder");
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

                // RIGHT SHOULDER - TRUE
                rightShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                rightShoulderJoint.connectedBody = null;
                rightShoulderConnector.SetActive(true);
                rightShoulderJoint.connectedBody = rightShoulderConnector.GetComponent<Rigidbody>();

                // RIGHT UPPER ARM - FALSE
                rightArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightArmConnector.SetActive(false);

                // RIGHT FORE ARM - FALSE
                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                rightForeArmConnector.SetActive(false);

                if (!setMirror)
                {
                    // LEFT SHOULDER - FALSE
                    leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = false;
                    leftShoulderConnector.SetActive(false);
                }
                else
                {
                    // LEFT SHOULDER - TRUE
                    leftShoulderConnector.GetComponent<Rigidbody>().isKinematic = true;
                    leftShoulderJoint.connectedBody = null;
                    leftShoulderConnector.SetActive(true);
                    leftShoulderJoint.connectedBody = leftShoulderConnector.GetComponent<Rigidbody>();
                }

                // LEFT UPPER ARM - FALSE
                leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftArmConnector.SetActive(false);

                // LEFT FORE ARM - FALSE
                leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                leftForeArmConnector.SetActive(false);
                break;

            case 10:
                Debug.Log("CASE 10: Right Upper Arm");
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

                if(!setMirror)
                {
                    leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                    leftArmConnector.SetActive(false);
                }
                else
                {
                    leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                    leftArmJoint.connectedBody = null;
                    leftArmConnector.SetActive(true);
                    leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();
                }

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
                Debug.Log("CASE 11: Right Fore Arm");
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


                if(!setMirror)
                {
                    leftArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                    leftArmConnector.SetActive(false);

                    leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = false;
                    leftForeArmConnector.SetActive(false);
                }
                else
                {
                    leftArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                    leftArmJoint.connectedBody = null;
                    leftArmConnector.SetActive(true);
                    leftArmJoint.connectedBody = leftArmConnector.GetComponent<Rigidbody>();

                    leftForeArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                    leftForeArmJoint.connectedBody = null;
                    leftForeArmConnector.SetActive(true);
                    leftForeArmJoint.connectedBody = leftForeArmConnector.GetComponent<Rigidbody>();
                }

                rightArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                rightArmJoint.connectedBody = null;
                rightArmConnector.SetActive(true);
                rightArmJoint.connectedBody = rightArmConnector.GetComponent<Rigidbody>();

                rightForeArmConnector.GetComponent<Rigidbody>().isKinematic = true;
                rightForeArmJoint.connectedBody = null;
                rightForeArmConnector.SetActive(true);
                rightForeArmJoint.connectedBody = rightForeArmConnector.GetComponent<Rigidbody>();
                break;

            case 12: // TODO: FIX
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
        CreateKinematicSkeletons();
        CreatePhysicalSkeletons();
        CreateInterpolatedSkeletons();

        CreateSkeletonsArms();

        aboveAnchorLength = kinematicAboveAnchorBones.Count;
        belowAnchorLength = kinematicBelowAnchorBones.Count;
    }

    private void CreateKinematicSkeletons()
    {
        FindAboveAnchorSkeleton(anchor, kinematicAboveAnchorBones, kinematicAnim);
        FindBelowAnchorSkeleton(anchor, kinematicBelowAnchorBones, kinematicAnim);
    }

    private void CreatePhysicalSkeletons()
    {
        FindAboveAnchorSkeleton(anchor, physicalAboveAnchorBones, ragdollAnim);
        FindBelowAnchorSkeleton(anchor, physicalBelowAnchorBones, ragdollAnim);
    }

    private void CreateInterpolatedSkeletons()
    {
        FindAboveAnchorSkeleton(anchor, interpolatedAboveAnchorBones, interpolatedAnim);
        FindBelowAnchorSkeleton(anchor, interpolatedBelowAnchorBones, interpolatedAnim);
    }

    private void CreateSkeletonsArms()
    {
        for (int i = 0; i < bonesLeftArm; i++)
        {
            kinematicLeftArmsBones.Add(kinematicAnim.GetBoneTransform(humanLeftArm[i]));
            physicalLeftArmsBones.Add(ragdollAnim.GetBoneTransform(humanLeftArm[i]));
            interpolatedLeftArmsBones.Add(interpolatedAnim.GetBoneTransform(humanLeftArm[i]));
        }

        for (int i = 0; i < bonesRightArm; i++)
        {
            kinematicRightArmsBones.Add(kinematicAnim.GetBoneTransform(humanRightArm[i]));
            physicalRightArmsBones.Add(ragdollAnim.GetBoneTransform(humanRightArm[i]));
            interpolatedRightArmsBones.Add(interpolatedAnim.GetBoneTransform(humanRightArm[i]));
        }
    }

    // ----------------------------------//

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

    // ----------------------------------//

    // =========== //
    //     Sync    //
    // =========== //

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

    private void SyncLegs()
    {
        upperLeftLegP.position = upperLeftLegK.position;
        upperLeftLegP.rotation = upperLeftLegK.rotation;
        bottomLeftLegP.position = bottomLeftLegK.position;
        bottomLeftLegP.rotation = bottomLeftLegK.rotation;
        leftFoot1P.position = leftFoot1K.position;
        leftFoot1P.rotation = leftFoot1K.rotation;
        leftFoot2P.position = leftFoot2K.position;
        leftFoot2P.rotation = leftFoot2K.rotation;
        leftFoot3P.position = leftFoot3K.position;
        leftFoot3P.rotation = leftFoot3K.rotation;

        upperRightLegP.position = upperRightLegK.position;
        upperRightLegP.rotation = upperRightLegK.rotation;
        bottomRightLegP.position = bottomRightLegK.position;
        bottomRightLegP.rotation = bottomRightLegK.rotation;
        rightFoot1P.position = rightFoot1K.position;
        rightFoot1P.rotation = rightFoot1K.rotation;
        rightFoot2P.position = rightFoot2K.position;
        rightFoot2P.rotation = rightFoot2K.rotation;
        rightFoot3P.position = rightFoot3K.position;
        rightFoot3P.rotation = rightFoot3K.rotation;
    }

    private void SyncRightHand()
    {
        //rightHandP.position = rightHandK.position;
        rightHandP.rotation = rightHandK.rotation;
    }

    private void SyncLeftHand()
    {
        //leftHandP.position = leftHandK.position;
        leftHandP.rotation = leftHandK.rotation;
    }

    private void SyncLeftArm()
    {
        // Left Arm
        for (int i = 0; i < SetSkeletonsV3.bonesLeftArm; i++)
        {
            physicalLeftArmsBones[i].rotation = kinematicLeftArmsBones[i].rotation;
            physicalLeftArmsBones[i].position = kinematicLeftArmsBones[i].position;
        }

        leftHandP.position = leftHandK.position;
        leftHandP.rotation = leftHandK.rotation;
    }

    private void SyncRightArm()
    {
        // Right Arm
        for (int i = 0; i < SetSkeletonsV3.bonesRightArm; i++)
        {
            physicalRightArmsBones[i].rotation = kinematicRightArmsBones[i].rotation;
            physicalRightArmsBones[i].position = kinematicRightArmsBones[i].position;
        }

        rightHandP.position = rightHandK.position;
        rightHandP.rotation = rightHandK.rotation;
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

        //physicalLeftArmsBones[1].position = kinematicLeftArmsBones[1].position;
        //physicalRightArmsBones[1].position = kinematicRightArmsBones[1].position;
    }

    // ----------------------------------//

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

            case AnchorPoint.LeftShoulder: return HumanBodyBones.LeftShoulder;
            case AnchorPoint.LeftUpperArm: return HumanBodyBones.LeftUpperArm;
            case AnchorPoint.LeftLowerArm: return HumanBodyBones.LeftLowerArm;

            case AnchorPoint.RightShoulder: return HumanBodyBones.RightShoulder;
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

            case AnchorPoint.LeftShoulder: return 6;
            case AnchorPoint.LeftUpperArm: return 7;
            case AnchorPoint.LeftLowerArm: return 8;

            case AnchorPoint.RightShoulder: return 9;
            case AnchorPoint.RightUpperArm: return 10;
            case AnchorPoint.RightLowerArm: return 11;
        }

        return -1;
    }
}

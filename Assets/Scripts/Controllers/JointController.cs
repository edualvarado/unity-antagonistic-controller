/****************************************************
 * File: JointController.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2022
   * Last update: 27/01/2022
*****************************************************/

/* Status: STABLE */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class JointController : MonoBehaviour
{

    #region Read-only & Static Fields

    // Normal PD Controller
    private readonly PDController _normalPDController = new PDController(1f, 0f, 0.1f);

    // Antagonistic PD Controller
    //private readonly AntagonisticController _antagonisticControllerX = new AntagonisticController(1f, 0f, 0f, 0.1f);
    //private readonly AntagonisticController _antagonisticControllerY = new AntagonisticController(1f, 0f, 0f, 0.1f);
    //private readonly AntagonisticController _antagonisticControllerZ = new AntagonisticController(1f, 0f, 0f, 0.1f);

    // Antagonistic PD Controller middle class array
    private readonly JointControllerImitation _antagonisticControllerXYZ = new JointControllerImitation(1f, 0.0f, 0.0f, 0.01f,
                                                                                                        1f, 0.0f, 0.0f, 0.01f,
                                                                                                        1f, 0.0f, 0.0f, 0.01f);
    // Window Graph - Hand
    private WindowGraph _handGraph;
    private GameObject _handPointX;
    private GameObject _handLineX;
    private GameObject _handPointY;
    private GameObject _handLineY; 
    private GameObject _handPointZ;
    private GameObject _handLineZ;

    private Toggle toggleX;
    private Toggle toggleY;
    private Toggle toggleZ;

    #endregion

    #region Instance Fields

    public enum Controller
    {
        DefaultPDController, NormalPDController, AntagonisticController
    }

    [Header("General Settings")]
    public Controller controllerType;
    public Transform kinematicLimb;

    [Header("Default PD Controller - Settings")]
    public bool activateDefaultPD;
    public float spring;
    public float damper;

    [Header("Normal PD Controller - Settings")]
    public bool activateNormalPD;
    public bool applyNormalTorque;
    public bool globalMode = true;
    public float Kp;
    public float Ki;
    public float Kd;
    public bool deployDesiredRotation;

    [Header("Antagonistic Controller - Settings")]
    public bool activateAntagonisticPD;
    public bool applyAntTorque;
    public bool debugMode;
    public float multGrav = 1f;

    [Header("Antagonistic Controller - Settings - X")]
    public float pLX;
    public float pHX;
    public float iX;
    public float dX;
    public float slopeX; // Before, variable used for each controller separatelly - now just to retrieve from lower class
    public float interceptX;
    public float minSoftLimitX;
    public float maxSoftLimitX;
    public float minHardLimitX;
    public float maxHardLimitX;
    public bool drawLimitsX;
    private bool applyAntTorqueX;
    private float torqueAppliedX;

    [Header("Antagonistic Controller - Settings - Y")]
    public float pLY;
    public float pHY;
    public float iY;
    public float dY;
    public float slopeY; // Before, variable used for each controller separatelly - now just to retrieve from lower class
    public float interceptY;
    public float minSoftLimitY;
    public float maxSoftLimitY;
    public float minHardLimitY;
    public float maxHardLimitY;
    public bool drawLimitsY;
    private bool applyAntTorqueY;
    private float torqueAppliedY;

    [Header("Antagonistic Controller - Settings - Z")]
    public float pLZ;
    public float pHZ;
    public float iZ;
    public float dZ;
    public float slopeZ; // Before, variable used for each controller separatelly - now just to retrieve from lower class
    public float interceptZ;
    public float minSoftLimitZ;
    public float maxSoftLimitZ;
    public float minHardLimitZ;
    public float maxHardLimitZ;
    public bool drawLimitsZ;
    private bool applyAntTorqueZ;
    private float torqueAppliedZ;

    // Others
    private ConfigurableJoint _jointAnt;
    private Transform _currentTransform;
    private Rigidbody _objectRigidbody;

    // Orientations
    private Quaternion _initialLocalOrientation;
    private Quaternion _initialGlobalOrientation;
    private Quaternion _currentLocalOrientation;
    private Quaternion _currentGlobalOrientation;
    private Quaternion _kinematicLocalOrientation;
    private Quaternion _kinematicGlobalOrientation;

    // External Forces
    private Vector3 distance3D;
    private Vector3 gravityAcc;
    private Vector3 gravityTorqueVector;
    private Vector3 gravityTorqueVectorLocal;

    // For Normal PD Controller
    private Quaternion newRotationLocal;
    private Quaternion newRotationGlobal;

    // For Antagonistic Controller
    private Quaternion currentLocalRotationMinX;
    private Quaternion currentLocalRotationMaxX;
    private Quaternion kinematicLocalRotationMinX;
    private Quaternion kinematicLocalRotationMaxX;

    private Quaternion currentLocalRotationMinY;
    private Quaternion currentLocalRotationMaxY;
    private Quaternion kinematicLocalRotationMinY;
    private Quaternion kinematicLocalRotationMaxY;

    private Quaternion currentLocalRotationMinZ;
    private Quaternion currentLocalRotationMaxZ;
    private Quaternion kinematicLocalRotationMinZ;
    private Quaternion kinematicLocalRotationMaxZ;

    #endregion

    #region Instance Properties

    public Quaternion DesiredLocalRotation { get; set; }

    #endregion

    #region Unity Methods

    private void Awake()
    {
        this._currentTransform = transform;
        this._objectRigidbody = GetComponent<Rigidbody>();
        this._jointAnt = GetComponent<ConfigurableJoint>();
        this._handGraph = GameObject.Find("WindowGraphHand").GetComponent<WindowGraph>();

        // Initial Orientations - They do not update!
        this._initialLocalOrientation = transform.localRotation;
        this._initialGlobalOrientation = transform.rotation;
    }

    private void Start()
    {
        // Get Toggle UI
        toggleX = GameObject.Find("ToggleX").GetComponent<Toggle>();
        toggleY = GameObject.Find("ToggleY").GetComponent<Toggle>();
        toggleZ = GameObject.Find("ToggleZ").GetComponent<Toggle>();

        toggleX.onValueChanged.AddListener(delegate
        {
            ToggleXValueChanged(toggleX);
        });

        toggleY.onValueChanged.AddListener(delegate
        {
            ToggleYValueChanged(toggleY);
        });

        toggleZ.onValueChanged.AddListener(delegate
        {
            ToggleZValueChanged(toggleZ);
        });

        // Window Graph - Hand
        _handPointX = _handGraph.CreateCircle(new Vector2(0, 0), Color.red, "handPointX");
        _handLineX = _handGraph.CreateLine(new Vector2(0, 0), new Vector2(0, 0), Color.red, "handLineX");
        _handPointY = _handGraph.CreateCircle(new Vector2(0, 0), Color.green, "handPointY");
        _handLineY = _handGraph.CreateLine(new Vector2(0, 0), new Vector2(0, 0), Color.green, "handLineY");
        _handPointZ = _handGraph.CreateCircle(new Vector2(0, 0), Color.blue, "handPointZ");
        _handLineZ = _handGraph.CreateLine(new Vector2(0, 0), new Vector2(0, 0), Color.blue, "handLineZ");
    }

    private void Update()
    {
        // Set hard limit to the limit in the joints
        var lowAngularXJoint = _jointAnt.lowAngularXLimit;
        lowAngularXJoint.limit = minHardLimitX;
        _jointAnt.lowAngularXLimit = lowAngularXJoint;

        var highAngularXJoint = _jointAnt.highAngularXLimit;
        highAngularXJoint.limit = maxHardLimitX;
        _jointAnt.highAngularXLimit = highAngularXJoint;

        var angularYJoint = _jointAnt.angularYLimit;
        angularYJoint.limit = maxHardLimitY;
        _jointAnt.angularYLimit = angularYJoint;

        var angularZJoint = _jointAnt.angularZLimit;
        angularZJoint.limit = maxHardLimitZ;
        _jointAnt.angularZLimit = angularZJoint;

        if(drawLimitsX)
        {
            // TODO - Should we multiply for transform.localRotation after the AngleAxis? Does the range varies actually if we move the hand?
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(minHardLimitX, transform.right) * transform.parent.up * 0.8f, Color.black);
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxHardLimitX, transform.right) * transform.parent.up * 0.8f, Color.black);

            Debug.DrawRay(transform.position, Quaternion.AngleAxis(minSoftLimitX, transform.right) * transform.parent.up * 0.4f, Color.red);
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxSoftLimitX, transform.right) * transform.parent.up * 0.4f, Color.green);
        }

        if (drawLimitsY)
        {
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(minHardLimitY, transform.up) * transform.parent.up * 0.8f, Color.black);
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxHardLimitY, transform.up) * transform.parent.up * 0.8f, Color.black);

            Debug.DrawRay(transform.position, Quaternion.AngleAxis(minSoftLimitY, transform.up) * transform.parent.up * 0.4f, Color.red);
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxSoftLimitY, transform.up) * transform.parent.up * 0.4f, Color.green);
        }

        if (drawLimitsZ)
        {
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(minHardLimitZ, transform.forward) * transform.parent.up * 0.8f, Color.black);
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxHardLimitZ, transform.forward) * transform.parent.up * 0.8f, Color.black);

            Debug.DrawRay(transform.position, Quaternion.AngleAxis(minSoftLimitZ, transform.forward) * transform.parent.up * 0.4f, Color.red);
            Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxSoftLimitZ, transform.forward) * transform.parent.up * 0.4f, Color.green);
        }
    }

    private void FixedUpdate()
    {
        if (DesiredLocalRotation == null || this._currentTransform == null || this._objectRigidbody == null || this.kinematicLimb == null)
        {
            return;
        }

        #region Update Gains and Isoline

        // Normal PD Controller
        this._normalPDController.KP = this.Kp;
        this._normalPDController.KI = this.Ki;
        this._normalPDController.KD = this.Kd;

        // Antagonistic PD Controller (Old)
        /*
        this._antagonisticControllerX.KPL = this.pLX;
        this._antagonisticControllerX.KPH = this.pHX;
        this._antagonisticControllerX.KI = this.iX;
        this._antagonisticControllerX.KD = this.dX;
        this._antagonisticControllerY.KPL = this.pLY;
        this._antagonisticControllerY.KPH = this.pHY;
        this._antagonisticControllerY.KI = this.iY;
        this._antagonisticControllerY.KD = this.dY;
        this._antagonisticControllerZ.KPL = this.pLZ;
        this._antagonisticControllerZ.KPH = this.pHZ;
        this._antagonisticControllerZ.KI = this.iZ;
        this._antagonisticControllerZ.KD = this.dZ;
        */

        // Antagonistic PD Controller with Middle Array
        this._antagonisticControllerXYZ.KPLX = this.pLX;
        this._antagonisticControllerXYZ.KIX = this.iX;
        this._antagonisticControllerXYZ.KDX = this.dX;

        this._antagonisticControllerXYZ.KPLY = this.pLY;
        this._antagonisticControllerXYZ.KIY = this.iY;
        this._antagonisticControllerXYZ.KDY = this.dY;

        this._antagonisticControllerXYZ.KPLZ = this.pLZ;
        this._antagonisticControllerXYZ.KIZ = this.iZ;
        this._antagonisticControllerXYZ.KDZ = this.dZ;

        // To get them back once they are calculated in the lower class and display them in the inspector
        this.pHX = this._antagonisticControllerXYZ.KPHX;
        this.pHY = this._antagonisticControllerXYZ.KPHY;
        this.pHZ = this._antagonisticControllerXYZ.KPHZ;

        // Getting slopes and intercepts
        this.slopeX = this._antagonisticControllerXYZ.SlopeX;
        this.slopeY = this._antagonisticControllerXYZ.SlopeY;
        this.slopeZ = this._antagonisticControllerXYZ.SlopeZ;
        this.interceptX = this._antagonisticControllerXYZ.interceptX;
        this.interceptY = this._antagonisticControllerXYZ.interceptY;
        this.interceptZ = this._antagonisticControllerXYZ.interceptZ;

        #endregion

        #region Plot

        // Window Graph Update - Hand
        _handGraph.MoveCircle(_handPointX, new Vector2(pLX, pHX));
        if (slopeX > 1f)
            _handGraph.MoveLine(_handLineX, Vector2.zero, new Vector2((_handGraph.Rect.sizeDelta.x / slopeX), _handGraph.Rect.sizeDelta.y));
        else if (slopeX < 1f)
            _handGraph.MoveLine(_handLineX, Vector2.zero, new Vector2(_handGraph.Rect.sizeDelta.x, _handGraph.Rect.sizeDelta.y * slopeX));

        _handGraph.MoveCircle(_handPointY, new Vector2(pLY, pHY));
        if (slopeY > 1f)
            _handGraph.MoveLine(_handLineY, Vector2.zero, new Vector2((_handGraph.Rect.sizeDelta.x / slopeY), _handGraph.Rect.sizeDelta.y));
        else if (slopeY < 1f)
            _handGraph.MoveLine(_handLineY, Vector2.zero, new Vector2(_handGraph.Rect.sizeDelta.x, _handGraph.Rect.sizeDelta.y * slopeY));

        _handGraph.MoveCircle(_handPointZ, new Vector2(pLZ, pHZ));
        if (slopeZ > 1f)
            _handGraph.MoveLine(_handLineZ, Vector2.zero, new Vector2((_handGraph.Rect.sizeDelta.x / slopeZ), _handGraph.Rect.sizeDelta.y));
        else if (slopeZ < 1f)
            _handGraph.MoveLine(_handLineZ, Vector2.zero, new Vector2(_handGraph.Rect.sizeDelta.x, _handGraph.Rect.sizeDelta.y * slopeZ));

        #endregion  

        #region Getting Orientations

        // Get kinematic orientations to be followed (Kinematic Model)
        _kinematicLocalOrientation = kinematicLimb.transform.localRotation;
        _kinematicGlobalOrientation = kinematicLimb.transform.rotation;

        // Get current orientations to be moved (Ragdoll Model)
        _currentLocalOrientation = transform.localRotation;
        _currentGlobalOrientation = transform.rotation;

        #endregion

        #region External Forces

        // Calculate forces relative to the RB - Distance from root to the COM
        distance3D = _objectRigidbody.worldCenterOfMass - transform.position;

        // 1. Gravity force and generated torque
        gravityAcc = Physics.gravity;
        gravityTorqueVector = Vector3.Cross(distance3D, _objectRigidbody.mass * gravityAcc); // Remember: wrt. global coord. system
        gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // Remember: wrt. local coord. system
        //Debug.DrawRay(transform.position, distance3D, Color.blue);
        //Debug.DrawRay(transform.position, gravityTorqueVectorLocal, Color.red);
        //Debug.DrawRay(_objectRigidbody.worldCenterOfMass, gravityAcc, Color.green);

        /* TODO - This should be improved, also maybe adding other external forces? */

        #endregion

        #region Default PD Controller

        /*
         * 1. The first method (DefaultPDController) already sets the target at the configurable joint. 
         * Otherwise, turn off DefaultPDController and set joint back to normal
         */

        if ((controllerType == Controller.DefaultPDController) && (activateDefaultPD))
        {
            // [Local]
            DesiredLocalRotation = ConfigurableJointExtensions.GetTargetRotationLocal(_jointAnt, _kinematicLocalOrientation, _initialLocalOrientation, _currentLocalOrientation, transform);
            ConfigurableJointExtensions.SetTargetRotationLocal(_jointAnt, _kinematicLocalOrientation, _initialLocalOrientation);

            _jointAnt.rotationDriveMode = RotationDriveMode.XYAndZ;
            var angularXDrive = _jointAnt.angularXDrive;
            angularXDrive.positionSpring = spring;
            angularXDrive.positionDamper = damper;
            angularXDrive.maximumForce = Mathf.Infinity;
            _jointAnt.angularXDrive = angularXDrive;
            var angularYZDrive = _jointAnt.angularYZDrive;
            angularYZDrive.positionSpring = spring;
            angularYZDrive.positionDamper = damper;
            angularYZDrive.maximumForce = Mathf.Infinity;
            _jointAnt.angularYZDrive = angularYZDrive;
        }
        else
        {
            // [Local]
            DesiredLocalRotation = ConfigurableJointExtensions.GetTargetRotationLocal(_jointAnt, _kinematicLocalOrientation, _initialLocalOrientation, _currentLocalOrientation, transform);

            _jointAnt.targetRotation = Quaternion.identity;
            _jointAnt.rotationDriveMode = RotationDriveMode.XYAndZ;
            var angularXDrive = _jointAnt.angularXDrive;
            angularXDrive.positionSpring = 0f;
            angularXDrive.positionDamper = 0f;
            angularXDrive.maximumForce = Mathf.Infinity;
            _jointAnt.angularXDrive = angularXDrive;
            var angularYZDrive = _jointAnt.angularYZDrive;
            angularYZDrive.positionSpring = 0f;
            angularYZDrive.positionDamper = 0f;
            angularYZDrive.maximumForce = Mathf.Infinity;
            _jointAnt.angularYZDrive = angularYZDrive;
        }

        #endregion

        #region Normal PD Controller

        /* 
         * 2. The second method (NormalPDController) uses my own implementation for a PD controller using Forward pass with Euler's integration, and using Axis-Angle representation.
         */

        if ((controllerType == Controller.NormalPDController) && (activateNormalPD))
        {

            // Hotfix related to the Inertia issue
            _objectRigidbody.angularDrag = 10f;

            Vector3 requiredTorque = ComputeRequiredTorque(_currentLocalOrientation,
                                                           _currentGlobalOrientation,
                                                           _kinematicLocalOrientation,
                                                           _kinematicGlobalOrientation,
                                                           DesiredLocalRotation,
                                                           this._objectRigidbody.angularVelocity,
                                                           gravityTorqueVectorLocal,
                                                           Time.fixedDeltaTime);

            Debug.Log("[INFO: " + this.gameObject.name + "] Normal PD Controller (Angle-axis) requiredTorque: " + requiredTorque);

            if (applyNormalTorque)
            {
                if (globalMode)
                    this._objectRigidbody.AddTorque(requiredTorque, ForceMode.Force); // Option (B) for Improved Torque [Global]
                else
                    this._objectRigidbody.AddRelativeTorque(requiredTorque, ForceMode.Force); // Option (B) for Improved Torque [Local]

                //this._objectRigidbody.AddRelativeTorque(requiredAngularAcceleration, ForceMode.Acceleration); // Option (A) for Torque [Local]
            }
        }

        #endregion

        #region Antagonistic Controller

        /* 
         * 3. The third method (AntagonisticController) uses my own implementation for a Antagonistic Controllers using Axis-Angle representation.
         */

        if ((controllerType == Controller.AntagonisticController) && (activateAntagonisticPD))
        {
            // Hotfix related to the Inertia issue
            _objectRigidbody.angularDrag = 49.9f;

            Vector3 requiredAntagonisticTorque = _antagonisticControllerXYZ.ComputeRequiredAntagonisticTorque(minSoftLimitX, maxSoftLimitX,
                                                                                                              minSoftLimitY, maxSoftLimitY,
                                                                                                              minSoftLimitZ, maxSoftLimitZ,
                                                                                                              _currentLocalOrientation,
                                                                                                              _currentGlobalOrientation,
                                                                                                              _kinematicLocalOrientation,
                                                                                                              _kinematicGlobalOrientation,
                                                                                                              DesiredLocalRotation,
                                                                                                              this._objectRigidbody.angularVelocity,
                                                                                                              gravityTorqueVectorLocal * multGrav,
                                                                                                              Time.fixedDeltaTime,
                                                                                                              debugMode,
                                                                                                              _currentTransform);
            /*
            Vector3 requiredAntagonisticTorqueX = ComputeRequiredAntagonisticTorqueX(_currentLocalOrientation,
                                                                                     _currentGlobalOrientation,
                                                                                     _kinematicLocalOrientation,
                                                                                     _kinematicGlobalOrientation,
                                                                                     DesiredLocalRotation,
                                                                                     this._objectRigidbody.angularVelocity,
                                                                                     gravityTorqueVectorLocal,
                                                                                     Time.fixedDeltaTime);

            Vector3 requiredAntagonisticTorqueY = ComputeRequiredAntagonisticTorqueY(_currentLocalOrientation,
                                                                                     _currentGlobalOrientation,
                                                                                     _kinematicLocalOrientation,
                                                                                     _kinematicGlobalOrientation,
                                                                                     DesiredLocalRotation,
                                                                                     this._objectRigidbody.angularVelocity,
                                                                                     gravityTorqueVectorLocal,
                                                                                     Time.fixedDeltaTime);

            Vector3 requiredAntagonisticTorqueZ = ComputeRequiredAntagonisticTorqueZ(_currentLocalOrientation,
                                                                                     _currentGlobalOrientation,
                                                                                     _kinematicLocalOrientation,
                                                                                     _kinematicGlobalOrientation,
                                                                                     DesiredLocalRotation,
                                                                                     this._objectRigidbody.angularVelocity,
                                                                                     gravityTorqueVectorLocal,
                                                                                     Time.fixedDeltaTime);
            */


            if (debugMode)
            {
                Debug.Log("[INFO: " + this.gameObject.name + "] NEW Antagonistic PD Controller (Angle-axis) requiredAntagonisticTorque: " + requiredAntagonisticTorque);

                /*
                Debug.Log("[INFO: " + this.gameObject.name + "] Antagonistic PD Controller (Angle-axis) requiredAntagonisticTorqueX: " + requiredAntagonisticTorqueX);
                Debug.Log("[INFO: " + this.gameObject.name + "] Antagonistic PD Controller (Angle-axis) requiredAntagonisticTorqueY: " + requiredAntagonisticTorqueY);
                Debug.Log("[INFO: " + this.gameObject.name + "] Antagonistic PD Controller (Angle-axis) requiredAntagonisticTorqueZ: " + requiredAntagonisticTorqueZ);
                */
            }

            if(applyAntTorque)
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorque, ForceMode.Force);
            }
            
            /*
            if (applyAntTorqueX)
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorqueX, ForceMode.Force);
            }

            if (applyAntTorqueY)
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorqueY, ForceMode.Force);
            }

            if (applyAntTorqueZ)
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorqueZ, ForceMode.Force);
            }
            */
        }

        #endregion
    }

    #endregion

    #region Old Instance Methods

    /*
    /// <summary>
    /// Compute torque for X-axis using Antagonistic Controller.
    /// </summary>
    /// <param name="currentLocalOrientation"></param>
    /// <param name="currentGlobalOrientation"></param>
    /// <param name="kinematicLocalOrientation"></param>
    /// <param name="kinematicGlobalOrientation"></param>
    /// <param name="desiredLocalRotation"></param>
    /// <param name="angularVelocity"></param>
    /// <param name="gravityTorqueVectorLocal"></param>
    /// <param name="fixedDeltaTime"></param>
    /// <returns></returns>
    private Vector3 ComputeRequiredAntagonisticTorqueX(Quaternion currentLocalOrientation, Quaternion currentGlobalOrientation,
                                                       Quaternion kinematicLocalOrientation, Quaternion kinematicGlobalOrientation,
                                                       Quaternion desiredLocalRotation, Vector3 angularVelocity, Vector3 gravityTorqueVectorLocal,
                                                       float fixedDeltaTime)
    {

        #region Orientations

        // Component-wise projection of Min/Max Euler Axis - It should not be needed
        Quaternion minAngleQuaternionX = Quaternion.Euler(new Vector3(minSoftLimitX, 0f, 0f));
        Quaternion maxAngleQuaternionX = Quaternion.Euler(new Vector3(maxSoftLimitX, 0f, 0f));

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionX = getRotationComponentAboutAxis(currentLocalOrientation, Vector3.right);
        Quaternion kinematicLocalOrientationQuaternionX = getRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.right);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionXAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionXAxis = Vector3.zero;
        currentLocalOrientationQuaternionX.ToAngleAxis(out currentLocalOrientationQuaternionXAngle, out currentLocalOrientationQuaternionXAxis);
        currentLocalOrientationQuaternionX.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionXAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionXAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionX.ToAngleAxis(out kinematicLocalOrientationQuaternionXAngle, out kinematicLocalOrientationQuaternionXAxis);
        kinematicLocalOrientationQuaternionX.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionXAngleCorrected = (currentLocalOrientationQuaternionXAngle > 180) ? currentLocalOrientationQuaternionXAngle - 360 : currentLocalOrientationQuaternionXAngle;
        float kinematicLocalOrientationQuaternionXAngleCorrected = (kinematicLocalOrientationQuaternionXAngle > 180) ? kinematicLocalOrientationQuaternionXAngle - 360 : kinematicLocalOrientationQuaternionXAngle;

        // Print axis and angle
        Debug.Log("[INFO: " + this.gameObject.name + "] Current Orientation X Axis: " + currentLocalOrientationQuaternionXAxis + " | Current Orientation X Angle: " + currentLocalOrientationQuaternionXAngleCorrected);
        Debug.Log("[INFO: " + this.gameObject.name + "] Kinematic Orientation X Axis: " + kinematicLocalOrientationQuaternionXAxis + " | Current Orientation X Angle: " + kinematicLocalOrientationQuaternionXAngleCorrected);

        #endregion

        #region Current Rotations - Not needed

        // Create Rotation from X-projected Current Orientation to minAngleQuaternionX and convert to Angle-Axis
        //currentLocalRotationMinX = minAngleQuaternionX * Quaternion.Inverse(currentLocalOrientation); // You shouldn't use the entire Quaternion
        currentLocalRotationMinX = minAngleQuaternionX * Quaternion.Inverse(currentLocalOrientationQuaternionX); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (currentLocalRotationMinX.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            currentLocalRotationMinX.x = -currentLocalRotationMinX.x;
            currentLocalRotationMinX.y = -currentLocalRotationMinX.y;
            currentLocalRotationMinX.z = -currentLocalRotationMinX.z;
            currentLocalRotationMinX.w = -currentLocalRotationMinX.w;
        }
        float currentLocalRotationMinXAngle = 0.0f;
        Vector3 currentLocalRotationMinXAxis = Vector3.zero;
        currentLocalRotationMinX.ToAngleAxis(out currentLocalRotationMinXAngle, out currentLocalRotationMinXAxis);
        currentLocalRotationMinXAxis.Normalize();

        // Create Rotation from X-projected Current Orientation to maxAngleQuaternionX and convert to Angle-Axis
        //currentLocalRotationMaxX = maxAngleQuaternionX * Quaternion.Inverse(currentLocalOrientation); // You shouldn't use the entire Quaternion
        currentLocalRotationMaxX = maxAngleQuaternionX * Quaternion.Inverse(currentLocalOrientationQuaternionX); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (currentLocalRotationMaxX.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            currentLocalRotationMaxX.x = -currentLocalRotationMaxX.x;
            currentLocalRotationMaxX.y = -currentLocalRotationMaxX.y;
            currentLocalRotationMaxX.z = -currentLocalRotationMaxX.z;
            currentLocalRotationMaxX.w = -currentLocalRotationMaxX.w;
        }
        float currentLocalRotationMaxXAngle = 0.0f;
        Vector3 currentLocalRotationMaxXAxis = Vector3.zero;
        currentLocalRotationMaxX.ToAngleAxis(out currentLocalRotationMaxXAngle, out currentLocalRotationMaxXAxis);
        currentLocalRotationMaxXAxis.Normalize();

        #endregion

        #region Kinematic Rotations - Not needed

        // Create Rotation from X-projected Kinematic Orientation to minAngleQuaternionX and convert to Angle-Axis

        //kinematicLocalRotationMinX = minAngleQuaternionX * Quaternion.Inverse(kinematicLocalOrientation); // You shouldn't use the entire Quaternion
        kinematicLocalRotationMinX = minAngleQuaternionX * Quaternion.Inverse(kinematicLocalOrientationQuaternionX); // Using the projection
        
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinematicLocalRotationMinX.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinematicLocalRotationMinX.x = -kinematicLocalRotationMinX.x;
            kinematicLocalRotationMinX.y = -kinematicLocalRotationMinX.y;
            kinematicLocalRotationMinX.z = -kinematicLocalRotationMinX.z;
            kinematicLocalRotationMinX.w = -kinematicLocalRotationMinX.w;
        }
        float kinematicLocalRotationMinXAngle = 0.0f;
        Vector3 kinematicLocalRotationMinXAxis = Vector3.zero;
        kinematicLocalRotationMinX.ToAngleAxis(out kinematicLocalRotationMinXAngle, out kinematicLocalRotationMinXAxis);
        kinematicLocalRotationMinXAxis.Normalize();

        // Create Rotation from X-projected Kinematic Orientation to maxAngleQuaternionX and convert to Angle-Axis

        //kinematicLocalRotationMaxX = maxAngleQuaternionX * Quaternion.Inverse(kinematicLocalOrientation); // You shouldn't use the entire Quaternion
        kinematicLocalRotationMaxX = maxAngleQuaternionX * Quaternion.Inverse(kinematicLocalOrientationQuaternionX); // Using the projection
        
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinematicLocalRotationMaxX.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinematicLocalRotationMaxX.x = -kinematicLocalRotationMaxX.x;
            kinematicLocalRotationMaxX.y = -kinematicLocalRotationMaxX.y;
            kinematicLocalRotationMaxX.z = -kinematicLocalRotationMaxX.z;
            kinematicLocalRotationMaxX.w = -kinematicLocalRotationMaxX.w;
        }
        float kinematicLocalRotationMaxXAngle = 0.0f;
        Vector3 kinematicLocalRotationMaxXAxis = Vector3.zero;
        kinematicLocalRotationMaxX.ToAngleAxis(out kinematicLocalRotationMaxXAngle, out kinematicLocalRotationMaxXAxis);
        kinematicLocalRotationMaxXAxis.Normalize();

        #endregion

        #region Estimate Errors from Rotation - Not needed

        float errorLocalRotationMinX = (currentLocalRotationMinXAngle * currentLocalRotationMinXAxis).x;
        float errorLocalRotationMaxX = (currentLocalRotationMaxXAngle * currentLocalRotationMaxXAxis).x;
        Debug.Log("[INFO: " + this.gameObject.name + "] errorLocalRotationMinX: " + errorLocalRotationMinX + " | errorLocalRotationMaxX: " + errorLocalRotationMaxX);

        #endregion

        #region Estimate Errors directly from Axis-Angle

        float currentLocalErrorMinX = minSoftLimitX - currentLocalOrientationQuaternionXAngleCorrected;
        float currentLocalErrorMaxX = maxSoftLimitX - currentLocalOrientationQuaternionXAngleCorrected;
        Debug.Log("[INFO: " + this.gameObject.name + "] currentLocalErrorMinX: " + currentLocalErrorMinX + " | currentLocalErrorMaxX: " + currentLocalErrorMaxX);

        float kinematicLocalErrorMinX = minSoftLimitX - kinematicLocalOrientationQuaternionXAngleCorrected;
        float kinematicLocalErrorMaxX = maxSoftLimitX - kinematicLocalOrientationQuaternionXAngleCorrected;
        Debug.Log("[INFO: " + this.gameObject.name + "] kinematicLocalErrorMinX: " + kinematicLocalErrorMinX + " | kinematicLocalErrorMaxX: " + kinematicLocalErrorMaxX);

        #endregion

        #region Isoline with Rotation errors - Not Needed

        interceptX = (0f) / (kinematicLocalRotationMaxXAngle * kinematicLocalRotationMaxXAxis).x;
        slopeX = ((kinematicLocalRotationMinXAngle * kinematicLocalRotationMinXAxis).x) / (-(kinematicLocalRotationMaxXAngle * kinematicLocalRotationMaxXAxis).x);
        pHX = pLX * slopeX + interceptX;
        this._antagonisticControllerX.KPL = pLX;
        this._antagonisticControllerX.KPH = pHX;
        torqueAppliedX = _antagonisticControllerX.GetOutput(errorLocalRotationMinX, errorLocalRotationMaxX, angularVelocity.magnitude, Time.fixedDeltaTime);

        #endregion

        #region Isoline with Angle errors

        interceptX = (0f) / kinematicLocalErrorMaxX;
        slopeX = (kinematicLocalErrorMinX) / (-(kinematicLocalErrorMaxX));
        pHX = pLX * slopeX + interceptX;
        this._antagonisticControllerX.KPL = pLX;
        this._antagonisticControllerX.KPH = pHX;
        torqueAppliedX = _antagonisticControllerX.GetOutput(currentLocalErrorMinX, currentLocalErrorMaxX, angularVelocity.magnitude, Time.fixedDeltaTime);

        #endregion

        return new Vector3(torqueAppliedX, 0f, 0f);
    }

    /// <summary>
    /// Compute torque for Y-axis using Antagonistic Controller.
    /// </summary>
    /// <param name="currentLocalOrientation"></param>
    /// <param name="currentGlobalOrientation"></param>
    /// <param name="kinematicLocalOrientation"></param>
    /// <param name="kinematicGlobalOrientation"></param>
    /// <param name="desiredLocalRotation"></param>
    /// <param name="angularVelocity"></param>
    /// <param name="gravityTorqueVectorLocal"></param>
    /// <param name="fixedDeltaTime"></param>
    /// <returns></returns>
    private Vector3 ComputeRequiredAntagonisticTorqueY(Quaternion currentLocalOrientation, Quaternion currentGlobalOrientation,
                                                       Quaternion kinematicLocalOrientation, Quaternion kinematicGlobalOrientation,
                                                       Quaternion desiredLocalRotation, Vector3 angularVelocity, Vector3 gravityTorqueVectorLocal,
                                                       float fixedDeltaTime)
    {

        #region Orientations

        // Component-wise projection of Min/Max Euler Axis - It should not be needed
        Quaternion minAngleQuaternionY = Quaternion.Euler(new Vector3(0f, minSoftLimitY, 0f));
        Quaternion maxAngleQuaternionY = Quaternion.Euler(new Vector3(0f, maxSoftLimitY, 0f));

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionY = getRotationComponentAboutAxis(currentLocalOrientation, Vector3.up);
        Quaternion kinematicLocalOrientationQuaternionY = getRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.up);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionYAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionYAxis = Vector3.zero;
        currentLocalOrientationQuaternionY.ToAngleAxis(out currentLocalOrientationQuaternionYAngle, out currentLocalOrientationQuaternionYAxis);
        currentLocalOrientationQuaternionY.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionYAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionYAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionY.ToAngleAxis(out kinematicLocalOrientationQuaternionYAngle, out kinematicLocalOrientationQuaternionYAxis);
        kinematicLocalOrientationQuaternionY.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionYAngleCorrected = (currentLocalOrientationQuaternionYAngle > 180) ? currentLocalOrientationQuaternionYAngle - 360 : currentLocalOrientationQuaternionYAngle;
        float kinematicLocalOrientationQuaternionYAngleCorrected = (kinematicLocalOrientationQuaternionYAngle > 180) ? kinematicLocalOrientationQuaternionYAngle - 360 : kinematicLocalOrientationQuaternionYAngle;

        // Print axis and angle
        Debug.Log("[INFO: " + this.gameObject.name + "] Current Orientation Y Axis: " + currentLocalOrientationQuaternionYAxis + " | Current Orientation Y Angle: " + currentLocalOrientationQuaternionYAngleCorrected);
        Debug.Log("[INFO: " + this.gameObject.name + "] Kinematic Orientation Y Axis: " + kinematicLocalOrientationQuaternionYAxis + " | Current Orientation Y Angle: " + kinematicLocalOrientationQuaternionYAngleCorrected);

        #endregion

        #region Current Rotations - Not needed

        // Create Rotation from Y-projected Current Orientation to minAngleQuaternionY and convert to Angle-Axis
        //currentLocalRotationMinY = minAngleQuaternionY * Quaternion.Inverse(currentLocalOrientation); // You shouldn't use the entire Quaternion
        currentLocalRotationMinY = minAngleQuaternionY * Quaternion.Inverse(currentLocalOrientationQuaternionY); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (currentLocalRotationMinY.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            currentLocalRotationMinY.x = -currentLocalRotationMinY.x;
            currentLocalRotationMinY.y = -currentLocalRotationMinY.y;
            currentLocalRotationMinY.z = -currentLocalRotationMinY.z;
            currentLocalRotationMinY.w = -currentLocalRotationMinY.w;
        }
        float currentLocalRotationMinYAngle = 0.0f;
        Vector3 currentLocalRotationMinYAxis = Vector3.zero;
        currentLocalRotationMinY.ToAngleAxis(out currentLocalRotationMinYAngle, out currentLocalRotationMinYAxis);
        currentLocalRotationMinYAxis.Normalize();

        // Create Rotation from Y-projected Current Orientation to maxAngleQuaternionY and convert to Angle-Axis
        //currentLocalRotationMaxY = maxAngleQuaternionY * Quaternion.Inverse(currentLocalOrientation); // You shouldn't use the entire Quaternion
        currentLocalRotationMaxY = maxAngleQuaternionY * Quaternion.Inverse(currentLocalOrientationQuaternionY); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (currentLocalRotationMaxY.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            currentLocalRotationMaxY.x = -currentLocalRotationMaxY.x;
            currentLocalRotationMaxY.y = -currentLocalRotationMaxY.y;
            currentLocalRotationMaxY.z = -currentLocalRotationMaxY.z;
            currentLocalRotationMaxY.w = -currentLocalRotationMaxY.w;
        }
        float currentLocalRotationMaxYAngle = 0.0f;
        Vector3 currentLocalRotationMaxYAxis = Vector3.zero;
        currentLocalRotationMaxY.ToAngleAxis(out currentLocalRotationMaxYAngle, out currentLocalRotationMaxYAxis);
        currentLocalRotationMaxYAxis.Normalize();
        
        #endregion

        #region Kinematic Rotations - Not needed

        // Create Rotation from Y-projected Kinematic Orientation to minAngleQuaternionX and convert to Angle-Axis

        //kinematicLocalRotationMinY = minAngleQuaternionY * Quaternion.Inverse(kinematicLocalOrientation); // You shouldn't use the entire Quaternion
        kinematicLocalRotationMinY = minAngleQuaternionY * Quaternion.Inverse(kinematicLocalOrientationQuaternionY); // Using the projection
        
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinematicLocalRotationMinY.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinematicLocalRotationMinY.x = -kinematicLocalRotationMinY.x;
            kinematicLocalRotationMinY.y = -kinematicLocalRotationMinY.y;
            kinematicLocalRotationMinY.z = -kinematicLocalRotationMinY.z;
            kinematicLocalRotationMinY.w = -kinematicLocalRotationMinY.w;
        }
        float kinematicLocalRotationMinYAngle = 0.0f;
        Vector3 kinematicLocalRotationMinYAxis = Vector3.zero;
        kinematicLocalRotationMinY.ToAngleAxis(out kinematicLocalRotationMinYAngle, out kinematicLocalRotationMinYAxis);
        kinematicLocalRotationMinYAxis.Normalize();

        // Create Rotation from Y-projected Kinematic Orientation to maxAngleQuaternionY and convert to Angle-Axis

        //kinematicLocalRotationMaxY = maxAngleQuaternionY * Quaternion.Inverse(kinematicLocalOrientation); // You shouldn't use the entire Quaternion
        kinematicLocalRotationMaxY = maxAngleQuaternionY * Quaternion.Inverse(kinematicLocalOrientationQuaternionY); // Using the projection
        
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinematicLocalRotationMaxY.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinematicLocalRotationMaxY.x = -kinematicLocalRotationMaxY.x;
            kinematicLocalRotationMaxY.y = -kinematicLocalRotationMaxY.y;
            kinematicLocalRotationMaxY.z = -kinematicLocalRotationMaxY.z;
            kinematicLocalRotationMaxY.w = -kinematicLocalRotationMaxY.w;
        }
        float kinematicLocalRotationMaxYAngle = 0.0f;
        Vector3 kinematicLocalRotationMaxYAxis = Vector3.zero;
        kinematicLocalRotationMaxY.ToAngleAxis(out kinematicLocalRotationMaxYAngle, out kinematicLocalRotationMaxYAxis);
        kinematicLocalRotationMaxYAxis.Normalize();

        #endregion

        #region Estimate Errors from Rotation - Not needed

        float errorLocalRotationMinY = (currentLocalRotationMinYAngle * currentLocalRotationMinYAxis).y;
        float errorLocalRotationMaxY = (currentLocalRotationMaxYAngle * currentLocalRotationMaxYAxis).y;
        Debug.Log("[INFO: " + this.gameObject.name + "] errorLocalRotationMinY: " + errorLocalRotationMinY + " | errorLocalRotationMaxY: " + errorLocalRotationMaxY);

        #endregion

        #region Estimate Errors directly from Axis-Angle

        float currentLocalErrorMinY = minSoftLimitY - currentLocalOrientationQuaternionYAngleCorrected;
        float currentLocalErrorMaxY = maxSoftLimitY - currentLocalOrientationQuaternionYAngleCorrected;
        Debug.Log("[INFO: " + this.gameObject.name + "] currentLocalErrorMinY: " + currentLocalErrorMinY + " | currentLocalErrorMaxY: " + currentLocalErrorMaxY);

        float kinematicLocalErrorMinY = minSoftLimitY - kinematicLocalOrientationQuaternionYAngleCorrected;
        float kinematicLocalErrorMaxY = maxSoftLimitY - kinematicLocalOrientationQuaternionYAngleCorrected;
        Debug.Log("[INFO: " + this.gameObject.name + "] kinematicLocalErrorMinY: " + kinematicLocalErrorMinY + " | kinematicLocalErrorMaxY: " + kinematicLocalErrorMaxY);

        #endregion

        #region Isoline with Rotation errors - Not Needed

        interceptY = (0f) / (kinematicLocalRotationMaxYAngle * kinematicLocalRotationMaxYAxis).y;
        slopeY = ((kinematicLocalRotationMinYAngle * kinematicLocalRotationMinYAxis).y) / (-(kinematicLocalRotationMaxYAngle * kinematicLocalRotationMaxYAxis).y);
        pHY = pLY * slopeY + interceptY;
        this._antagonisticControllerY.KPL = pLY;
        this._antagonisticControllerY.KPH = pHY;
        torqueAppliedY = _antagonisticControllerY.GetOutput(errorLocalRotationMinY, errorLocalRotationMaxY, angularVelocity.magnitude, Time.fixedDeltaTime);

        #endregion

        #region Isoline with Angle errors

        interceptY = (0f) / kinematicLocalErrorMaxY;
        slopeY = (kinematicLocalErrorMinY) / (-(kinematicLocalErrorMaxY));
        pHY = pLY * slopeY + interceptY;
        this._antagonisticControllerY.KPL = pLY;
        this._antagonisticControllerY.KPH = pHY;
        torqueAppliedY = _antagonisticControllerY.GetOutput(currentLocalErrorMinY, currentLocalErrorMaxY, angularVelocity.magnitude, Time.fixedDeltaTime);

        #endregion

        return new Vector3(0f, torqueAppliedY, 0f);
    }

    /// <summary>
    /// Compute torque for Z-axis using Antagonistic Controller.
    /// </summary>
    /// <param name="currentLocalOrientation"></param>
    /// <param name="currentGlobalOrientation"></param>
    /// <param name="kinematicLocalOrientation"></param>
    /// <param name="kinematicGlobalOrientation"></param>
    /// <param name="desiredLocalRotation"></param>
    /// <param name="angularVelocity"></param>
    /// <param name="gravityTorqueVectorLocal"></param>
    /// <param name="fixedDeltaTime"></param>
    /// <returns></returns>
    private Vector3 ComputeRequiredAntagonisticTorqueZ(Quaternion currentLocalOrientation, Quaternion currentGlobalOrientation,
                                                       Quaternion kinematicLocalOrientation, Quaternion kinematicGlobalOrientation,
                                                       Quaternion desiredLocalRotation, Vector3 angularVelocity, Vector3 gravityTorqueVectorLocal,
                                                       float fixedDeltaTime)
    {

        #region Orientations

        // Component-wise projection of Min/Max Euler Axis - It should not be needed
        Quaternion minAngleQuaternionZ = Quaternion.Euler(new Vector3(0f, 0f, minSoftLimitZ));
        Quaternion maxAngleQuaternionZ = Quaternion.Euler(new Vector3(0f, 0f, maxSoftLimitZ));

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionZ = getRotationComponentAboutAxis(currentLocalOrientation, Vector3.forward);
        Quaternion kinematicLocalOrientationQuaternionZ = getRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.forward);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionZAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionZAxis = Vector3.zero;
        currentLocalOrientationQuaternionZ.ToAngleAxis(out currentLocalOrientationQuaternionZAngle, out currentLocalOrientationQuaternionZAxis);
        currentLocalOrientationQuaternionZ.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionZAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionZAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionZ.ToAngleAxis(out kinematicLocalOrientationQuaternionZAngle, out kinematicLocalOrientationQuaternionZAxis);
        kinematicLocalOrientationQuaternionZ.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionZAngleCorrected = (currentLocalOrientationQuaternionZAngle > 180) ? currentLocalOrientationQuaternionZAngle - 360 : currentLocalOrientationQuaternionZAngle;
        float kinematicLocalOrientationQuaternionZAngleCorrected = (kinematicLocalOrientationQuaternionZAngle > 180) ? kinematicLocalOrientationQuaternionZAngle - 360 : kinematicLocalOrientationQuaternionZAngle;

        // Print axis and angle
        Debug.Log("[INFO: " + this.gameObject.name + "] Current Orientation Z Axis: " + currentLocalOrientationQuaternionZAxis + " | Current Orientation Z Angle: " + currentLocalOrientationQuaternionZAngleCorrected);
        Debug.Log("[INFO: " + this.gameObject.name + "] Kinematic Orientation Z Axis: " + kinematicLocalOrientationQuaternionZAxis + " | Current Orientation Z Angle: " + kinematicLocalOrientationQuaternionZAngleCorrected);

        #endregion

        #region Current Rotations - Not needed

        // Create Rotation from Z-projected Current Orientation to minAngleQuaternionZ and convert to Angle-Axis
        //currentLocalRotationMinZ = minAngleQuaternionZ * Quaternion.Inverse(currentLocalOrientation); // You shouldn't use the entire Quaternion
        currentLocalRotationMinZ = minAngleQuaternionZ * Quaternion.Inverse(currentLocalOrientationQuaternionZ); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (currentLocalRotationMinZ.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            currentLocalRotationMinZ.x = -currentLocalRotationMinZ.x;
            currentLocalRotationMinZ.y = -currentLocalRotationMinZ.y;
            currentLocalRotationMinZ.z = -currentLocalRotationMinZ.z;
            currentLocalRotationMinZ.w = -currentLocalRotationMinZ.w;
        }
        float currentLocalRotationMinZAngle = 0.0f;
        Vector3 currentLocalRotationMinZAxis = Vector3.zero;
        currentLocalRotationMinZ.ToAngleAxis(out currentLocalRotationMinZAngle, out currentLocalRotationMinZAxis);
        currentLocalRotationMinZAxis.Normalize();

        // Create Rotation from Y-projected Current Orientation to maxAngleQuaternionY and convert to Angle-Axis
        //currentLocalRotationMaxY = maxAngleQuaternionY * Quaternion.Inverse(currentLocalOrientation); // You shouldn't use the entire Quaternion
        currentLocalRotationMaxZ = maxAngleQuaternionZ * Quaternion.Inverse(currentLocalOrientationQuaternionZ); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (currentLocalRotationMaxZ.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            currentLocalRotationMaxZ.x = -currentLocalRotationMaxZ.x;
            currentLocalRotationMaxZ.y = -currentLocalRotationMaxZ.y;
            currentLocalRotationMaxZ.z = -currentLocalRotationMaxZ.z;
            currentLocalRotationMaxZ.w = -currentLocalRotationMaxZ.w;
        }
        float currentLocalRotationMaxZAngle = 0.0f;
        Vector3 currentLocalRotationMaxZAxis = Vector3.zero;
        currentLocalRotationMaxZ.ToAngleAxis(out currentLocalRotationMaxZAngle, out currentLocalRotationMaxZAxis);
        currentLocalRotationMaxZAxis.Normalize();

        #endregion

        #region Kinematic Rotations - Not needed

        // Create Rotation from Z-projected Kinematic Orientation to minAngleQuaternionZ and convert to Angle-Axis

        //kinematicLocalRotationMinZ = minAngleQuaternionZ * Quaternion.Inverse(kinematicLocalOrientation); // You shouldn't use the entire Quaternion
        kinematicLocalRotationMinZ = minAngleQuaternionZ * Quaternion.Inverse(kinematicLocalOrientationQuaternionZ); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinematicLocalRotationMinZ.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinematicLocalRotationMinZ.x = -kinematicLocalRotationMinZ.x;
            kinematicLocalRotationMinZ.y = -kinematicLocalRotationMinZ.y;
            kinematicLocalRotationMinZ.z = -kinematicLocalRotationMinZ.z;
            kinematicLocalRotationMinZ.w = -kinematicLocalRotationMinZ.w;
        }
        float kinematicLocalRotationMinZAngle = 0.0f;
        Vector3 kinematicLocalRotationMinZAxis = Vector3.zero;
        kinematicLocalRotationMinZ.ToAngleAxis(out kinematicLocalRotationMinZAngle, out kinematicLocalRotationMinZAxis);
        kinematicLocalRotationMinZAxis.Normalize();

        // Create Rotation from Z-projected Kinematic Orientation to maxAngleQuaternionZ and convert to Angle-Axis

        //kinematicLocalRotationMaxZ = maxAngleQuaternionZ * Quaternion.Inverse(kinematicLocalOrientation); // You shouldn't use the entire Quaternion
        kinematicLocalRotationMaxZ = maxAngleQuaternionZ * Quaternion.Inverse(kinematicLocalOrientationQuaternionZ); // Using the projection

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinematicLocalRotationMaxZ.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinematicLocalRotationMaxZ.x = -kinematicLocalRotationMaxZ.x;
            kinematicLocalRotationMaxZ.y = -kinematicLocalRotationMaxZ.y;
            kinematicLocalRotationMaxZ.z = -kinematicLocalRotationMaxZ.z;
            kinematicLocalRotationMaxZ.w = -kinematicLocalRotationMaxZ.w;
        }
        float kinematicLocalRotationMaxZAngle = 0.0f;
        Vector3 kinematicLocalRotationMaxZAxis = Vector3.zero;
        kinematicLocalRotationMaxY.ToAngleAxis(out kinematicLocalRotationMaxZAngle, out kinematicLocalRotationMaxZAxis);
        kinematicLocalRotationMaxZAxis.Normalize();

        #endregion

        #region Estimate Errors from Rotation - Not needed

        float errorLocalRotationMinZ = (currentLocalRotationMinZAngle * currentLocalRotationMinZAxis).z;
        float errorLocalRotationMaxZ = (currentLocalRotationMaxZAngle * currentLocalRotationMaxZAxis).z;
        Debug.Log("[INFO: " + this.gameObject.name + "] errorLocalRotationMinZ: " + errorLocalRotationMinZ + " | errorLocalRotationMaxZ: " + errorLocalRotationMaxZ);

        #endregion

        #region Estimate Errors directly from Axis-Angle

        float currentLocalErrorMinZ = minSoftLimitZ - currentLocalOrientationQuaternionZAngleCorrected;
        float currentLocalErrorMaxZ = maxSoftLimitZ - currentLocalOrientationQuaternionZAngleCorrected;
        Debug.Log("[INFO: " + this.gameObject.name + "] currentLocalErrorMinZ: " + currentLocalErrorMinZ + " | currentLocalErrorMaxZ: " + currentLocalErrorMaxZ);

        float kinematicLocalErrorMinZ = minSoftLimitZ - kinematicLocalOrientationQuaternionZAngleCorrected;
        float kinematicLocalErrorMaxZ = maxSoftLimitZ - kinematicLocalOrientationQuaternionZAngleCorrected;
        Debug.Log("[INFO: " + this.gameObject.name + "] kinematicLocalErrorMinZ: " + kinematicLocalErrorMinZ + " | kinematicLocalErrorMaxZ: " + kinematicLocalErrorMaxZ);

        #endregion

        #region Isoline with Rotation errors - Not Needed

        interceptZ = (0f) / (kinematicLocalRotationMaxZAngle * kinematicLocalRotationMaxZAxis).z;
        slopeZ = ((kinematicLocalRotationMinZAngle * kinematicLocalRotationMinZAxis).z) / (-(kinematicLocalRotationMaxZAngle * kinematicLocalRotationMaxZAxis).z);
        pHZ = pLZ * slopeZ + interceptZ;
        this._antagonisticControllerZ.KPL = pLZ;
        this._antagonisticControllerZ.KPH = pHZ;
        torqueAppliedZ = _antagonisticControllerZ.GetOutput(errorLocalRotationMinZ, errorLocalRotationMaxZ, angularVelocity.magnitude, Time.fixedDeltaTime);

        #endregion

        #region Isoline with Angle errors

        interceptZ = (0f) / kinematicLocalErrorMaxZ;
        slopeZ = (kinematicLocalErrorMinZ) / (-(kinematicLocalErrorMaxZ));
        pHZ = pLZ * slopeZ + interceptZ;
        this._antagonisticControllerZ.KPL = pLZ;
        this._antagonisticControllerZ.KPH = pHZ;
        torqueAppliedZ = _antagonisticControllerZ.GetOutput(currentLocalErrorMinZ, currentLocalErrorMaxZ, angularVelocity.magnitude, Time.fixedDeltaTime);

        #endregion

        return new Vector3(0f, 0f, torqueAppliedZ);
    }

    */

    /// <summary>
    /// Compute torque using Normal PD Controller.
    /// </summary>
    /// <param name="currentLocalOrientation"></param>
    /// <param name="currentGlobalOrientation"></param>
    /// <param name="kinematicLocalOrientation"></param>
    /// <param name="kinematicGlobalOrientation"></param>
    /// <param name="desiredLocalRotation"></param>
    /// <param name="angularVelocity"></param>
    /// <param name="gravityTorqueVectorLocal"></param>
    /// <param name="fixedDeltaTime"></param>
    /// <returns></returns>
    private Vector3 ComputeRequiredTorque(Quaternion currentLocalOrientation, Quaternion currentGlobalOrientation, 
                                          Quaternion kinematicLocalOrientation, Quaternion kinematicGlobalOrientation,
                                          Quaternion desiredLocalRotation, Vector3 angularVelocity, Vector3 gravityTorqueVectorLocal, 
                                          float fixedDeltaTime)
    {
        #region Orientations and Rotations

        // Current Local Orientation in Angle-axis
        float currentAngle = 0.0f;
        Vector3 currentAxis = Vector3.zero;
        currentLocalOrientation.ToAngleAxis(out currentAngle, out currentAxis);

        // Target Local Orientation in Angle-axis
        float targetAngle = 0.0f;
        Vector3 targetAxis = Vector3.zero;
        kinematicLocalOrientation.ToAngleAxis(out targetAngle, out targetAxis);

        // Target Rotation in Angle-axis -> Not used, the quaternion is wrt. joint coordinate system, valid for integrated PD in the conf. joint only.
        float rotationAngle = 0.0f;
        Vector3 rotationAxis = Vector3.zero;
        desiredLocalRotation.ToAngleAxis(out rotationAngle, out rotationAxis); // <--- Wrong Quaternion, is wrt. joint coordinate system!

        #endregion

        #region Rotations

        // Create Rotation from Current Local Orientation to Kinematic Local Orientation and convert to Angle-Axis
        newRotationLocal = kinematicLocalOrientation * Quaternion.Inverse(currentLocalOrientation);

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationLocal.x = -newRotationLocal.x;
            newRotationLocal.y = -newRotationLocal.y;
            newRotationLocal.z = -newRotationLocal.z;
            newRotationLocal.w = -newRotationLocal.w;
        }
        float rotationNewAngleLocal = 0.0f;
        Vector3 rotationNewAxisLocal = Vector3.zero;
        newRotationLocal.ToAngleAxis(out rotationNewAngleLocal, out rotationNewAxisLocal);
        rotationNewAxisLocal.Normalize();

        // Create Rotation from Current Global Orientation to Kinematic Global Orientation and convert to Angle-Axis
        newRotationGlobal = kinematicGlobalOrientation * Quaternion.Inverse(currentGlobalOrientation);

        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationGlobal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationGlobal.x = -newRotationGlobal.x;
            newRotationGlobal.y = -newRotationGlobal.y;
            newRotationGlobal.z = -newRotationGlobal.z;
            newRotationGlobal.w = -newRotationGlobal.w;
        }
        float rotationNewAngleGlobal = 0.0f;
        Vector3 rotationNewAxisGlobal = Vector3.zero;
        newRotationGlobal.ToAngleAxis(out rotationNewAngleGlobal, out rotationNewAxisGlobal);
        rotationNewAxisGlobal.Normalize();

        #endregion

        #region Deploy Rotations - Test

        if (deployDesiredRotation)
        {
            // 1. DesiredLocalRotation
            //transform.localRotation = desiredLocalRotation; // Wrong! -> The rotation is in the joint space, while local rotation in local space.

            // 2. Apply rotation quaternion [Local]
            //transform.localRotation = newRotationLocal * transform.localRotation; // Works perfectly - Still some jittering which I don't know where it comes from

            // 3. Apply rotation quaternion [Global]
            //transform.rotation = newRotationGlobal * transform.rotation; // Does not work entirely, are global rotations - Still some jittering which I don't know where it comes from
        }

        #endregion

        #region Controller Errors

        // Estimate Angle Error Local
        float newRotationErrorLocal = rotationNewAngleLocal;
        //Debug.Log("[INFO: " + this.gameObject.name + "] newRotationErrorLocal: " + newRotationErrorLocal);

        // Rotation Axis Local
        //Debug.Log("[INFO: " + this.gameObject.name + "] rotationNewAxisLocal: " + rotationNewAxisLocal);
        //Debug.DrawRay(this.transform.position, rotationNewAxisLocal, Color.blue);

        // Estimate Angle Error Global
        float newRotationErrorGlobal = rotationNewAngleGlobal;
        //Debug.Log("[INFO: " + this.gameObject.name + "] newRotationErrorGlobal: " + newRotationErrorGlobal);

        // Rotation Axis Global
        //Debug.Log("[INFO: " + this.gameObject.name + "] rotationNewAxisGlobal: " + rotationNewAxisGlobal);
        //Debug.DrawRay(this.transform.position, rotationNewAxisGlobal, Color.blue);

        #endregion

        #region Torque Estimation

        /*       1. Normal Torque Estimation (A)     */
        /* ========================================= */

        // Normal Torque [Local]
        float torqueLocal = _normalPDController.GetOutput(newRotationErrorLocal, angularVelocity.magnitude, Time.fixedDeltaTime);
        //Debug.Log("[INFO] torqueLocal * rotationNewAxis: " + (torqueLocal * rotationNewAxisLocal));

        /*     2. Improved Torque Estimation (B)     */
        /* ========================================= */

        // Improved Torque [Global] and [Local]
        Vector3 torqueImprovedGlobal = _normalPDController.GetOutputAxisAngle(newRotationErrorGlobal, rotationNewAxisGlobal, angularVelocity, Time.fixedDeltaTime);
        Vector3 torqueImprovedLocal = _normalPDController.GetOutputAxisAngle(newRotationErrorLocal, rotationNewAxisLocal, angularVelocity, Time.fixedDeltaTime);
        //Debug.Log("[INFO] torqueImprovedGlobal: " + torqueImprovedGlobal);
        //Debug.Log("[INFO] torqueImprovedLocal: " + torqueImprovedLocal);

        #endregion

        #region Inertia

        // Convert rotation of inertia tensor to global
        Quaternion rotInertia2World = _objectRigidbody.inertiaTensorRotation * transform.rotation;

        // Global
        torqueImprovedGlobal = Quaternion.Inverse(rotInertia2World) * torqueImprovedGlobal;
        torqueImprovedGlobal.Scale(_objectRigidbody.inertiaTensor);
        torqueImprovedGlobal = rotInertia2World * torqueImprovedGlobal;
        //Debug.Log("[INFO] torqueImprovedGlobal: " + torqueImprovedGlobal);

        // Local -> TODO - It is probably wrong
        //torqueImprovedLocal = Quaternion.Inverse(rotInertia2World) * torqueImprovedLocal;
        //torqueImprovedLocal.Scale(_objectRigidbody.inertiaTensor);
        torqueImprovedLocal = _objectRigidbody.inertiaTensorRotation * torqueImprovedLocal;
        // Debug.Log("[INFO] torqueImprovedLocal: " + torqueImprovedLocal);

        #endregion

        /*       1. Normal Torque Estimation (A)     */
        /* ========================================= */

        //return torqueLocal * rotationNewAxisLocal; //

        /*     2. Improved Torque Estimation (B)     */
        /* ========================================= */

        if (globalMode)
            return torqueImprovedGlobal; // Working fine
        else
            return torqueImprovedLocal; // Not entirely working
    }

    #endregion

    #region Instance Methods

    // Swing-Twist Decomposition - TODO 
    private Quaternion getRotationComponentAboutAxis(Quaternion rotation, Vector3 direction)
    {
        Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);

        float dotProd = Vector3.Dot(rotationAxis, direction);

        // Shortcut calculation of `projection` requires `direction` to be normalized
        Vector3 projection = new Vector3(direction.x * dotProd, direction.y * dotProd, direction.z * dotProd);

        Quaternion twist = new Quaternion(
                projection.x, projection.y, projection.z, rotation.w).normalized;
        if (dotProd < 0.0)
        {
            // Ensure `twist` points towards `direction`
            twist.x = -twist.x;
            twist.y = -twist.y;
            twist.z = -twist.z;
            twist.w = -twist.w;
            // Rotation angle `twist.angle()` is now reliable
        }
        return twist;
    }

    #endregion

    #region Listeners

    void ToggleXValueChanged(Toggle toggle)
    {
        if(toggle.isOn)
            _handGraph.SetTransparency(_handPointX, _handLineX, Color.red, 1f, 0.5f);
        else
            _handGraph.SetTransparency(_handPointX, _handLineX, Color.red, 0f, 0f);
    }

    void ToggleYValueChanged(Toggle toggle)
    {
        if (toggle.isOn)
            _handGraph.SetTransparency(_handPointY, _handLineY, Color.green, 1f, 0.5f);
        else
            _handGraph.SetTransparency(_handPointY, _handLineY, Color.green, 0f, 0f);
    }

    void ToggleZValueChanged(Toggle toggle)
    {
        if (toggle.isOn)
            _handGraph.SetTransparency(_handPointZ, _handLineZ, Color.blue, 1f, 0.5f);
        else
            _handGraph.SetTransparency(_handPointZ, _handLineZ, Color.blue, 0f, 0f);
    }

    #endregion
}

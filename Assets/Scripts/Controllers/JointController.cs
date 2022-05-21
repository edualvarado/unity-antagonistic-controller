/****************************************************
 * File: JointController.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
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
    private readonly JointControllerImitation _antagonisticControllerXYZ = new JointControllerImitation(1f, 0.0f, 0.0f, 0.01f,
                                                                                                        1f, 0.0f, 0.0f, 0.01f,
                                                                                                        1f, 0.0f, 0.0f, 0.01f);
    // For Normal PD Controller
    private Quaternion newRotationLocal;
    private Quaternion newRotationGlobal;

    // Orientations
    private Quaternion _initialLocalOrientation;
    private Quaternion _initialGlobalOrientation;
    private Quaternion _currentLocalOrientation;
    private Quaternion _currentGlobalOrientation;
    private Quaternion _kinematicLocalOrientation;
    private Quaternion _kinematicGlobalOrientation;

    // Others
    private ConfigurableJoint _jointAnt;
    private Transform _currentTransform;
    private Transform _kinematicTransform;
    private Rigidbody _objectRigidbody;

    // Window Graph - Right Hand
    private WindowGraph _rightHandGraph;
    private RectTransform _rightHandGraphContainer;
    private GameObject _rightHandPointX;
    private GameObject _rightHandLineX;
    private GameObject _rightHandLineXCurrent;
    private GameObject _rightHandPointY;
    private GameObject _rightHandLineY;
    private GameObject _rightHandLineYCurrent;
    private GameObject _rightHandPointZ;
    private GameObject _rightHandLineZ;
    private GameObject _rightHandLineZCurrent;

    private Toggle rightHandToggleX;
    private Toggle rightHandToggleY;
    private Toggle rightHandToggleZ;

    // Window Graph - Right Fore Arm
    private WindowGraph _rightForeArmGraph;
    private RectTransform _rightForeArmGraphContainer;
    private GameObject _rightForeArmPointX;
    private GameObject _rightForeArmLineX;
    private GameObject _rightForeArmLineXCurrent;
    private GameObject _rightForeArmPointY;
    private GameObject _rightForeArmLineY;
    private GameObject _rightForeArmLineYCurrent;
    private GameObject _rightForeArmPointZ;
    private GameObject _rightForeArmLineZ;
    private GameObject _rightForeArmLineZCurrent;

    private Toggle rightForeArmToggleX;
    private Toggle rightForeArmToggleY;
    private Toggle rightForeArmToggleZ;

    // Window Graph - Right Arm
    private WindowGraph _rightArmGraph;
    private RectTransform _rightArmGraphContainer;
    private GameObject _rightArmPointX;
    private GameObject _rightArmLineX;
    private GameObject _rightArmLineXCurrent;
    private GameObject _rightArmPointY;
    private GameObject _rightArmLineY;
    private GameObject _rightArmLineYCurrent;
    private GameObject _rightArmPointZ;
    private GameObject _rightArmLineZ;
    private GameObject _rightArmLineZCurrent;

    private Toggle rightArmToggleX;
    private Toggle rightArmToggleY;
    private Toggle rightArmToggleZ;

    #endregion

    #region Instance Fields

    // TEST
    public float DELTATIME = 0.02f;
    //public float t = 0.0f;
    //public bool springReachedLeft = false;
    //public bool springReachedRight = false;

    public enum Controller
    {
        DefaultPDController, NormalPDController, AntagonisticController
    }

    [Header("General Settings")]
    public Controller controllerType;
    public GameObject kinematicLimb;
    public SafetyRegionLeft safetyRegionLeft;
    public SafetyRegionRight safetyRegionRight;

    [Header("Ragdoll Limbs for External Forces Calculation")]
    public GameObject physicalHand;
    public GameObject physicalForeArm;
    public GameObject physicalArm;

    [Header("Default PD Controller - Settings")]
    public bool activateDefaultPD;
    public float spring;
    public float damper;

    [Header("Normal PD Controller - Settings")]
    public bool activateNormalPD;
    public bool applyNormalTorque;
    public Vector3 requiredTorque;
    public bool globalMode = true;
    public float Kp;
    public float Ki;
    public float Kd;
    public bool deployDesiredRotation;
    public bool debugModeNormal;
    public bool drawModeNormal;

    [Header("Antagonistic Controller - Settings")]
    public bool activateAntagonisticPD;
    //public bool applyAntTorque;
    public bool applyAntTorqueX;
    public bool applyAntTorqueY;
    public bool applyAntTorqueZ;
    public Vector3 requiredAntagonisticLocalTorque;
    public bool debugModeAntagonistic;
    public bool drawModeAntagonistic;

    [Header("Antagonistic Controller - Plot")]
    [UPyPlot.UPyPlotController.UPyProbe]
    public float antTorqueX;
    [UPyPlot.UPyPlotController.UPyProbe]
    public float antTorqueY;
    [UPyPlot.UPyPlotController.UPyProbe]
    public float antTorqueZ;

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
    public float slopeXCurrent;
    public float interceptXCurrent;

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
    public float slopeYCurrent;
    public float interceptYCurrent;

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
    public float slopeZCurrent;
    public float interceptZCurrent;

    [Header("External Forces")]
    public Vector3 distance3D;
    public Vector3 gravityAcc;
    public Vector3 gravityTorqueVector;
    public Vector3 gravityTorqueVectorLocal;
    public bool drawTorques;

    #endregion

    #region Instance Properties

    public Quaternion DesiredLocalRotation { get; set; }

    #endregion

    #region Unity Methods

    private void Awake()
    {

        // Retrieve components
        this._currentTransform = this.GetComponent<Transform>(); // Used in Antagonistic
        this._kinematicTransform = kinematicLimb.GetComponent<Transform>();
        this._objectRigidbody = this.GetComponent<Rigidbody>();
        this._jointAnt = this.GetComponent<ConfigurableJoint>();

        // Initial Orientations - They do not update! (used for Default Controller)
        this._initialLocalOrientation = transform.localRotation;
        this._initialGlobalOrientation = transform.rotation;

        // Window Graphs
        this._rightHandGraph = GameObject.Find("WindowGraphRightHand").GetComponent<WindowGraph>();
        this._rightHandGraphContainer = GameObject.Find("GraphContainerRightHand").GetComponent<RectTransform>();
        this._rightForeArmGraph = GameObject.Find("WindowGraphRightForeArm").GetComponent<WindowGraph>();
        this._rightForeArmGraphContainer = GameObject.Find("GraphContainerRightForeArm").GetComponent<RectTransform>();
        this._rightArmGraph = GameObject.Find("WindowGraphRightArm").GetComponent<WindowGraph>();
        this._rightArmGraphContainer = GameObject.Find("GraphContainerRightArm").GetComponent<RectTransform>();
    }

    private void Start()
    {
        #region UI

        // Get Toggle UI
        rightHandToggleX = GameObject.Find("RightHandToggleX").GetComponent<Toggle>();
        rightHandToggleY = GameObject.Find("RightHandToggleY").GetComponent<Toggle>();
        rightHandToggleZ = GameObject.Find("RightHandToggleZ").GetComponent<Toggle>();
        rightForeArmToggleX = GameObject.Find("RightForeArmToggleX").GetComponent<Toggle>();
        rightForeArmToggleY = GameObject.Find("RightForeArmToggleY").GetComponent<Toggle>();
        rightForeArmToggleZ = GameObject.Find("RightForeArmToggleZ").GetComponent<Toggle>();
        rightArmToggleX = GameObject.Find("RightArmToggleX").GetComponent<Toggle>();
        rightArmToggleY = GameObject.Find("RightArmToggleY").GetComponent<Toggle>();
        rightArmToggleZ = GameObject.Find("RightArmToggleZ").GetComponent<Toggle>();

        rightHandToggleX.onValueChanged.AddListener(delegate
        {
            RightHandToggleXValueChanged(rightHandToggleX);
        });

        rightHandToggleY.onValueChanged.AddListener(delegate
        {
            RightHandToggleYValueChanged(rightHandToggleY);
        });

        rightHandToggleZ.onValueChanged.AddListener(delegate
        {
            RightHandToggleZValueChanged(rightHandToggleZ);
        });

        rightForeArmToggleX.onValueChanged.AddListener(delegate
        {
            RightForeArmToggleXValueChanged(rightForeArmToggleX);
        });

        rightForeArmToggleY.onValueChanged.AddListener(delegate
        {
            RightForeArmToggleYValueChanged(rightForeArmToggleY);
        });

        rightForeArmToggleZ.onValueChanged.AddListener(delegate
        {
            RightForeArmToggleZValueChanged(rightForeArmToggleZ);
        });

        rightArmToggleX.onValueChanged.AddListener(delegate
        {
            RightArmToggleXValueChanged(rightArmToggleX);
        });

        rightArmToggleY.onValueChanged.AddListener(delegate
        {
            RightArmToggleYValueChanged(rightArmToggleY);
        });

        rightArmToggleZ.onValueChanged.AddListener(delegate
        {
            RightArmToggleZValueChanged(rightArmToggleZ);
        });

        // Window Graph - Right Hand
        if (this.gameObject.CompareTag("RightHand"))
        {
            _rightHandPointX = _rightHandGraph.CreateCircle(_rightHandGraphContainer, new Vector2(0, 0), Color.red, "rightHandPointX");
            _rightHandLineX = _rightHandGraph.CreateLine(_rightHandGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.red, "rightHandLineX");
            _rightHandLineXCurrent = _rightHandGraph.CreateLine(_rightHandGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightHandLineXCurrent");
            _rightHandPointY = _rightHandGraph.CreateCircle(_rightHandGraphContainer, new Vector2(0, 0), Color.green, "rightHandPointY");
            _rightHandLineY = _rightHandGraph.CreateLine(_rightHandGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.green, "rightHandLineY");
            _rightHandLineYCurrent = _rightHandGraph.CreateLine(_rightHandGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightHandLineYCurrent");
            _rightHandPointZ = _rightHandGraph.CreateCircle(_rightHandGraphContainer, new Vector2(0, 0), Color.blue, "rightHandPointZ");
            _rightHandLineZ = _rightHandGraph.CreateLine(_rightHandGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.blue, "rightHandLineZ");
            _rightHandLineZCurrent = _rightHandGraph.CreateLine(_rightHandGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightHandLineZCurrent");
        }

        // Window Graph - Right Fore Arm
        if (this.gameObject.CompareTag("RightForeArm"))
        {
            _rightForeArmPointX = _rightForeArmGraph.CreateCircle(_rightForeArmGraphContainer, new Vector2(0, 0), Color.red, "rightForeArmPointX");
            _rightForeArmLineX = _rightForeArmGraph.CreateLine(_rightForeArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.red, "rightForeArmLineX");
            _rightForeArmLineXCurrent = _rightForeArmGraph.CreateLine(_rightForeArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightForeArmLineXCurrent");

            /* No DOF */
            /*
            _rightForeArmPointY = _rightForeArmGraph.CreateCircle(_rightForeArmGraphContainer, new Vector2(0, 0), Color.green, "rightForeArmPointY");
            _rightForeArmLineY = _rightForeArmGraph.CreateLine(_rightForeArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.green, "rightForeArmLineY");
            _rightForeArmLineYCurrent = _rightForeArmGraph.CreateLine(_rightForeArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightForeArmLineYCurrent");
            _rightForeArmPointZ = _rightForeArmGraph.CreateCircle(_rightForeArmGraphContainer, new Vector2(0, 0), Color.blue, "rightForeArmPointZ");
            _rightForeArmLineZ = _rightForeArmGraph.CreateLine(_rightForeArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.blue, "rightForeArmLineZ");
            _rightForeArmLineZCurrent = _rightForeArmGraph.CreateLine(_rightForeArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightForeArmLineZCurrent");
            */
        }

        // Window Graph - Right Arm
        if (this.gameObject.CompareTag("RightArm"))
        {
            _rightArmPointX = _rightArmGraph.CreateCircle(_rightArmGraphContainer, new Vector2(0, 0), Color.red, "rightArmPointX");
            _rightArmLineX = _rightArmGraph.CreateLine(_rightArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.red, "rightArmLineX");
            _rightArmLineXCurrent = _rightArmGraph.CreateLine(_rightArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightArmLineXCurrent");
            _rightArmPointY = _rightArmGraph.CreateCircle(_rightArmGraphContainer, new Vector2(0, 0), Color.green, "rightArmPointY");
            _rightArmLineY = _rightArmGraph.CreateLine(_rightArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.green, "rightArmLineY");
            _rightArmLineYCurrent = _rightArmGraph.CreateLine(_rightArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightArmLineYCurrent");
            _rightArmPointZ = _rightArmGraph.CreateCircle(_rightArmGraphContainer, new Vector2(0, 0), Color.blue, "rightArmPointZ");
            _rightArmLineZ = _rightArmGraph.CreateLine(_rightArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.blue, "rightArmLineZ");
            _rightArmLineZCurrent = _rightArmGraph.CreateLine(_rightArmGraphContainer, new Vector2(0, 0), new Vector2(0, 0), Color.black, "rightArmLineZCurrent");
        }

        #endregion
    }

    private void Update()
    {
        // TEST
        //Debug.Log("[UPDATE] FixedDeltaTime: " + Time.fixedDeltaTime.ToString("F4"));
        //Debug.Log("[UPDATE] DeltaTime: " + Time.deltaTime.ToString("F4"));

        #region Setting Limits

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

        #endregion

        #region Update Gains and Isoline

        // Normal PD Controller
        this._normalPDController.KP = this.Kp;
        this._normalPDController.KI = this.Ki;
        this._normalPDController.KD = this.Kd;

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
        this.interceptX = this._antagonisticControllerXYZ.InterceptX;
        this.interceptY = this._antagonisticControllerXYZ.InterceptY;
        this.interceptZ = this._antagonisticControllerXYZ.InterceptZ;

        // Getting slopes and intercepts from current orientation
        this.slopeXCurrent = this._antagonisticControllerXYZ.SlopeXCurrent;
        this.slopeYCurrent = this._antagonisticControllerXYZ.SlopeYCurrent;
        this.slopeZCurrent = this._antagonisticControllerXYZ.SlopeZCurrent;
        this.interceptXCurrent = this._antagonisticControllerXYZ.InterceptXCurrent;
        this.interceptYCurrent = this._antagonisticControllerXYZ.InterceptYCurrent;
        this.interceptZCurrent = this._antagonisticControllerXYZ.InterceptZCurrent;

        #endregion

        #region Plots

        // Window Graph Update - Right Hand
        if (this.gameObject.CompareTag("RightHand"))
        {
            // Moving point
            //_rightHandGraph.MoveCircle(_rightHandPointX, new Vector2(pLX, pHX));
            // Max value to reach: 80f
            if (slopeX > 1f)
            {
                if((pHX / 80f) * (_rightHandGraphContainer.sizeDelta.y) >= _rightHandGraphContainer.sizeDelta.y)
                {
                    _rightHandGraph.MoveCircle(_rightHandPointX, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeX), Mathf.Clamp((pHX / 80f) * (_rightHandGraphContainer.sizeDelta.y), 0, _rightHandGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightHandGraph.MoveCircle(_rightHandPointX, new Vector2((pLX / 80f) * _rightHandGraphContainer.sizeDelta.x, Mathf.Clamp((pHX / 80f) * (_rightHandGraphContainer.sizeDelta.y), 0, _rightHandGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeX < 1f)
            {
                _rightHandGraph.MoveCircle(_rightHandPointX, new Vector2((pLX / 80f) * _rightHandGraphContainer.sizeDelta.x, (pHX / 80f) * (_rightHandGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeX > 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineX, Vector2.zero, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeX), _rightHandGraphContainer.sizeDelta.y));
            }
            else if (slopeX < 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineX, Vector2.zero, new Vector2(_rightHandGraphContainer.sizeDelta.x, _rightHandGraphContainer.sizeDelta.y * slopeX));
            }
            if (slopeXCurrent > 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineXCurrent, Vector2.zero, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeXCurrent), _rightHandGraphContainer.sizeDelta.y));
            }
            else if (slopeXCurrent < 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineXCurrent, Vector2.zero, new Vector2(_rightHandGraphContainer.sizeDelta.x, _rightHandGraphContainer.sizeDelta.y * slopeXCurrent));
            }

            // Moving point
            //_rightHandGraph.MoveCircle(_rightHandPointY, new Vector2(pLY, pHY));
            // Max value to reach: 80f
            if (slopeY > 1f)
            {
                if ((pHY / 80f) * (_rightHandGraphContainer.sizeDelta.y) >= _rightHandGraphContainer.sizeDelta.y)
                {
                    _rightHandGraph.MoveCircle(_rightHandPointY, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeY), Mathf.Clamp((pHY / 80f) * (_rightHandGraphContainer.sizeDelta.y), 0, _rightHandGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightHandGraph.MoveCircle(_rightHandPointY, new Vector2((pLY / 80f) * _rightHandGraphContainer.sizeDelta.x, Mathf.Clamp((pHY / 80f) * (_rightHandGraphContainer.sizeDelta.y), 0, _rightHandGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeY < 1f)
            {
                _rightHandGraph.MoveCircle(_rightHandPointY, new Vector2((pLY / 80f) * _rightHandGraphContainer.sizeDelta.x, (pHY / 80f) * (_rightHandGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeY > 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineY, Vector2.zero, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeY), _rightHandGraphContainer.sizeDelta.y));
            }
            else if (slopeY < 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineY, Vector2.zero, new Vector2(_rightHandGraphContainer.sizeDelta.x, _rightHandGraphContainer.sizeDelta.y * slopeY));
            }
            if (slopeYCurrent > 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineYCurrent, Vector2.zero, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeYCurrent), _rightHandGraphContainer.sizeDelta.y));
            }
            else if (slopeYCurrent < 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineYCurrent, Vector2.zero, new Vector2(_rightHandGraphContainer.sizeDelta.x, _rightHandGraphContainer.sizeDelta.y * slopeYCurrent));
            }

            // Moving point
            //_rightHandGraph.MoveCircle(_rightHandPointZ, new Vector2(pLZ, pHZ));
            // Max value to reach: 80f
            if (slopeZ > 1f)
            {
                if ((pHZ / 80f) * (_rightHandGraphContainer.sizeDelta.y) >= _rightHandGraphContainer.sizeDelta.y)
                {
                    _rightHandGraph.MoveCircle(_rightHandPointZ, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeZ), Mathf.Clamp((pHZ / 80f) * (_rightHandGraphContainer.sizeDelta.y), 0, _rightHandGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightHandGraph.MoveCircle(_rightHandPointZ, new Vector2((pLZ / 80f) * _rightHandGraphContainer.sizeDelta.x, Mathf.Clamp((pHZ / 80f) * (_rightHandGraphContainer.sizeDelta.y), 0, _rightHandGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeZ < 1f)
            {
                _rightHandGraph.MoveCircle(_rightHandPointZ, new Vector2((pLZ / 80f) * _rightHandGraphContainer.sizeDelta.x, (pHZ / 80f) * (_rightHandGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeZ > 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineZ, Vector2.zero, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeZ), _rightHandGraphContainer.sizeDelta.y));
            }
            else if (slopeZ < 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineZ, Vector2.zero, new Vector2(_rightHandGraphContainer.sizeDelta.x, _rightHandGraphContainer.sizeDelta.y * slopeZ));
            }
            if (slopeZCurrent > 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineZCurrent, Vector2.zero, new Vector2((_rightHandGraphContainer.sizeDelta.x / slopeZCurrent), _rightHandGraphContainer.sizeDelta.y));
            }
            else if (slopeZCurrent < 1f)
            {
                _rightHandGraph.MoveLine(_rightHandLineZCurrent, Vector2.zero, new Vector2(_rightHandGraphContainer.sizeDelta.x, _rightHandGraphContainer.sizeDelta.y * slopeZCurrent));
            }
        }

        // Window Graph Update - Right Fore Arm
        if (this.gameObject.CompareTag("RightForeArm"))
        {
            // Moving point
            //_rightForeArmGraph.MoveCircle(_rightForeArmPointX, new Vector2(pLX, pHX));
            // Max value to reach: 800f
            if (slopeX > 1f)
            {
                if ((pHX / 800f) * (_rightForeArmGraphContainer.sizeDelta.y) >= _rightForeArmGraphContainer.sizeDelta.y)
                {
                    _rightForeArmGraph.MoveCircle(_rightForeArmPointX, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeX), Mathf.Clamp((pHX / 800f) * (_rightForeArmGraphContainer.sizeDelta.y), 0, _rightForeArmGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightForeArmGraph.MoveCircle(_rightForeArmPointX, new Vector2((pLX / 800f) * _rightForeArmGraphContainer.sizeDelta.x, Mathf.Clamp((pHX / 800f) * (_rightForeArmGraphContainer.sizeDelta.y), 0, _rightForeArmGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeX < 1f)
            {
                _rightForeArmGraph.MoveCircle(_rightForeArmPointX, new Vector2((pLX / 800f) * _rightForeArmGraphContainer.sizeDelta.x, (pHX / 800f) * (_rightForeArmGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeX > 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineX, Vector2.zero, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeX), _rightForeArmGraphContainer.sizeDelta.y));
            }
            else if (slopeX < 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineX, Vector2.zero, new Vector2(_rightForeArmGraphContainer.sizeDelta.x, _rightForeArmGraphContainer.sizeDelta.y * slopeX));
            }
            if (slopeXCurrent > 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineXCurrent, Vector2.zero, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeXCurrent), _rightForeArmGraphContainer.sizeDelta.y));
            }
            else if (slopeXCurrent < 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineXCurrent, Vector2.zero, new Vector2(_rightForeArmGraphContainer.sizeDelta.x, _rightForeArmGraphContainer.sizeDelta.y * slopeXCurrent));
            }

            /* No DOF */
            /*

            // Moving point
            //_rightForeArmGraph.MoveCircle(_rightForeArmPointY, new Vector2(pLY, pHY));
            // Max value to reach: 40f
            if (slopeY > 1f)
            {
                if ((pHY / 40f) * (_rightForeArmGraphContainer.sizeDelta.y) >= _rightForeArmGraphContainer.sizeDelta.y)
                {
                    _rightForeArmGraph.MoveCircle(_rightForeArmPointY, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeY), Mathf.Clamp((pHY / 40f) * (_rightForeArmGraphContainer.sizeDelta.y), 0, _rightForeArmGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightForeArmGraph.MoveCircle(_rightForeArmPointY, new Vector2((pLY / 40f) * _rightForeArmGraphContainer.sizeDelta.x, Mathf.Clamp((pHY / 40f) * (_rightForeArmGraphContainer.sizeDelta.y), 0, _rightForeArmGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeY < 1f)
            {
                _rightForeArmGraph.MoveCircle(_rightForeArmPointY, new Vector2((pLY / 40f) * _rightForeArmGraphContainer.sizeDelta.x, (pHY / 40f) * (_rightForeArmGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeY > 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineY, Vector2.zero, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeY), _rightForeArmGraphContainer.sizeDelta.y));
            }
            else if (slopeY < 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineY, Vector2.zero, new Vector2(_rightForeArmGraphContainer.sizeDelta.x, _rightForeArmGraphContainer.sizeDelta.y * slopeY));
            }
            if (slopeYCurrent > 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineYCurrent, Vector2.zero, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeYCurrent), _rightForeArmGraphContainer.sizeDelta.y));
            }
            else if (slopeYCurrent < 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineYCurrent, Vector2.zero, new Vector2(_rightForeArmGraphContainer.sizeDelta.x, _rightForeArmGraphContainer.sizeDelta.y * slopeYCurrent));
            }

            // Moving point
            //_rightForeArmGraph.MoveCircle(_rightForeArmPointZ, new Vector2(pLZ, pHZ));
            // Max value to reach: 40f
            if (slopeZ > 1f)
            {
                if ((pHZ / 800f) * (_rightForeArmGraphContainer.sizeDelta.y) >= _rightForeArmGraphContainer.sizeDelta.y)
                {
                    _rightForeArmGraph.MoveCircle(_rightForeArmPointZ, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeZ), Mathf.Clamp((pHZ / 800f) * (_rightForeArmGraphContainer.sizeDelta.y), 0, _rightForeArmGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightForeArmGraph.MoveCircle(_rightForeArmPointZ, new Vector2((pLZ / 800f) * _rightForeArmGraphContainer.sizeDelta.x, Mathf.Clamp((pHZ / 800f) * (_rightForeArmGraphContainer.sizeDelta.y), 0, _rightForeArmGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeZ < 1f)
            {
                _rightForeArmGraph.MoveCircle(_rightForeArmPointZ, new Vector2((pLZ / 800f) * _rightForeArmGraphContainer.sizeDelta.x, (pHZ / 800f) * (_rightForeArmGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeZ > 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineZ, Vector2.zero, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeZ), _rightForeArmGraphContainer.sizeDelta.y));
            }
            else if (slopeZ < 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineZ, Vector2.zero, new Vector2(_rightForeArmGraphContainer.sizeDelta.x, _rightForeArmGraphContainer.sizeDelta.y * slopeZ));
            }
            if (slopeZCurrent > 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineZCurrent, Vector2.zero, new Vector2((_rightForeArmGraphContainer.sizeDelta.x / slopeZCurrent), _rightForeArmGraphContainer.sizeDelta.y));
            }
            else if (slopeZCurrent < 1f)
            {
                _rightForeArmGraph.MoveLine(_rightForeArmLineZCurrent, Vector2.zero, new Vector2(_rightForeArmGraphContainer.sizeDelta.x, _rightForeArmGraphContainer.sizeDelta.y * slopeZCurrent));
            }

            */
        }

        // Window Graph Update - Right Arm
        if (this.gameObject.CompareTag("RightArm"))
        {
            // Moving point
            //_rightArmGraph.MoveCircle(_rightArmPointX, new Vector2(pLX, pHX));
            // Max value to reach: 160f
            if (slopeX > 1f)
            {
                if ((pHX / 160f) * (_rightArmGraphContainer.sizeDelta.y) >= _rightArmGraphContainer.sizeDelta.y)
                {
                    _rightArmGraph.MoveCircle(_rightArmPointX, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeX), Mathf.Clamp((pHX / 160f) * (_rightArmGraphContainer.sizeDelta.y), 0, _rightArmGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightArmGraph.MoveCircle(_rightArmPointX, new Vector2((pLX / 160f) * _rightArmGraphContainer.sizeDelta.x, Mathf.Clamp((pHX / 160f) * (_rightArmGraphContainer.sizeDelta.y), 0, _rightArmGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeX < 1f)
            {
                _rightArmGraph.MoveCircle(_rightArmPointX, new Vector2((pLX / 160f) * _rightArmGraphContainer.sizeDelta.x, (pHX / 160f) * (_rightArmGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeX > 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineX, Vector2.zero, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeX), _rightArmGraphContainer.sizeDelta.y));
            }
            else if (slopeX < 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineX, Vector2.zero, new Vector2(_rightArmGraphContainer.sizeDelta.x, _rightArmGraphContainer.sizeDelta.y * slopeX));
            }
            if (slopeXCurrent > 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineXCurrent, Vector2.zero, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeXCurrent), _rightArmGraphContainer.sizeDelta.y));
            }
            else if (slopeXCurrent < 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineXCurrent, Vector2.zero, new Vector2(_rightArmGraphContainer.sizeDelta.x, _rightArmGraphContainer.sizeDelta.y * slopeXCurrent));
            }

            // Moving point
            //_rightArmGraph.MoveCircle(_rightArmPointY, new Vector2(pLY, pHY));
            // Max value to reach: 160f
            if (slopeY > 1f)
            {
                if ((pHY / 160f) * (_rightArmGraphContainer.sizeDelta.y) >= _rightArmGraphContainer.sizeDelta.y)
                {
                    _rightArmGraph.MoveCircle(_rightArmPointY, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeY), Mathf.Clamp((pHY / 160f) * (_rightArmGraphContainer.sizeDelta.y), 0, _rightArmGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightArmGraph.MoveCircle(_rightArmPointY, new Vector2((pLY / 160f) * _rightArmGraphContainer.sizeDelta.x, Mathf.Clamp((pHY / 160f) * (_rightArmGraphContainer.sizeDelta.y), 0, _rightArmGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeY < 1f)
            {
                _rightArmGraph.MoveCircle(_rightArmPointY, new Vector2((pLY / 160f) * _rightArmGraphContainer.sizeDelta.x, (pHY / 160f) * (_rightArmGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeY > 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineY, Vector2.zero, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeY), _rightArmGraphContainer.sizeDelta.y));
            }
            else if (slopeY < 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineY, Vector2.zero, new Vector2(_rightArmGraphContainer.sizeDelta.x, _rightArmGraphContainer.sizeDelta.y * slopeY));
            }
            if (slopeYCurrent > 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineYCurrent, Vector2.zero, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeYCurrent), _rightArmGraphContainer.sizeDelta.y));
            }
            else if (slopeYCurrent < 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineYCurrent, Vector2.zero, new Vector2(_rightArmGraphContainer.sizeDelta.x, _rightArmGraphContainer.sizeDelta.y * slopeYCurrent));
            }

            // Moving point
            //_rightArmGraph.MoveCircle(_rightArmPointZ, new Vector2(pLZ, pHZ));
            // Max value to reach: 1600f
            if (slopeZ > 1f)
            {
                if ((pHZ / 1600f) * (_rightArmGraphContainer.sizeDelta.y) >= _rightArmGraphContainer.sizeDelta.y)
                {
                    _rightArmGraph.MoveCircle(_rightArmPointZ, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeZ), Mathf.Clamp((pHZ / 1600f) * (_rightArmGraphContainer.sizeDelta.y), 0, _rightArmGraphContainer.sizeDelta.y)));
                }
                else
                {
                    _rightArmGraph.MoveCircle(_rightArmPointZ, new Vector2((pLZ / 1600f) * _rightArmGraphContainer.sizeDelta.x, Mathf.Clamp((pHZ / 1600f) * (_rightArmGraphContainer.sizeDelta.y), 0, _rightArmGraphContainer.sizeDelta.y)));
                }
            }
            else if (slopeZ < 1f)
            {
                _rightArmGraph.MoveCircle(_rightArmPointZ, new Vector2((pLZ / 1600f) * _rightArmGraphContainer.sizeDelta.x, (pHZ / 1600f) * (_rightArmGraphContainer.sizeDelta.y)));
            }

            // Moving line
            if (slopeZ > 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineZ, Vector2.zero, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeZ), _rightArmGraphContainer.sizeDelta.y));
            }
            else if (slopeZ < 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineZ, Vector2.zero, new Vector2(_rightArmGraphContainer.sizeDelta.x, _rightArmGraphContainer.sizeDelta.y * slopeZ));
            }
            if (slopeZCurrent > 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineZCurrent, Vector2.zero, new Vector2((_rightArmGraphContainer.sizeDelta.x / slopeZCurrent), _rightArmGraphContainer.sizeDelta.y));
            }
            else if (slopeZCurrent < 1f)
            {
                _rightArmGraph.MoveLine(_rightArmLineZCurrent, Vector2.zero, new Vector2(_rightArmGraphContainer.sizeDelta.x, _rightArmGraphContainer.sizeDelta.y * slopeZCurrent));
            }
        }

        #endregion
    }

    private void FixedUpdate()
    {
        // TEST
        //Debug.Log("[FIXED UPDATE] FixedDeltaTime: " + Time.fixedDeltaTime.ToString("F4"));
        //Debug.Log("[FIXED UPDATE] DeltaTime: " + Time.deltaTime.ToString("F4"));

        if (DesiredLocalRotation == null || this._currentTransform == null || this._objectRigidbody == null || this.kinematicLimb == null)
        {
            return;
        }

        #region Getting Orientations

        // Get kinematic orientations to be followed (Kinematic Model)
        _kinematicLocalOrientation = _kinematicTransform.localRotation;
        _kinematicGlobalOrientation = _kinematicTransform.rotation;

        // Get current orientations to be moved (Ragdoll Model)
        _currentLocalOrientation = _currentTransform.localRotation;
        _currentGlobalOrientation = _currentTransform.rotation;

        #endregion

        #region External Forces

        // Calculate forces relative to the RB - Distance from root to the COM

        // 1. Gravity force and generated torque
        gravityAcc = Physics.gravity;

        if (this.CompareTag("RightHand"))
        {
            distance3D = _objectRigidbody.worldCenterOfMass - transform.position;
            gravityTorqueVector = Vector3.Cross(distance3D, _objectRigidbody.mass * gravityAcc); // wrt. global coord. system
            gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // wrt. local coord. system

            if (drawTorques)
            {
                Debug.DrawRay(transform.position, distance3D, Color.cyan);
                Debug.DrawRay(transform.position, transform.TransformDirection(gravityTorqueVectorLocal), Color.yellow); // Drawing later would draw the ang. acceleration
            }

            gravityTorqueVectorLocal.Scale(new Vector3(1 / _objectRigidbody.inertiaTensor.x, 1 / _objectRigidbody.inertiaTensor.y, 1 / _objectRigidbody.inertiaTensor.z));

            //Debug.Log("[INFO] gravityTorqueVectorLocal: " + gravityTorqueVectorLocal);
        }
        else if (this.CompareTag("RightForeArm"))
        {
            distance3D = ((_objectRigidbody.worldCenterOfMass - transform.position) + (physicalHand.GetComponent<Rigidbody>().worldCenterOfMass - transform.position)) / 2;
            gravityTorqueVector = Vector3.Cross(distance3D, (_objectRigidbody.mass + physicalHand.GetComponent<Rigidbody>().mass) * gravityAcc); // wrt. global coord. system
            gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // wrt. local coord. system

            if (drawTorques)
            {
                Debug.DrawRay(transform.position, distance3D, Color.cyan);
                Debug.DrawRay(transform.position, transform.TransformDirection(gravityTorqueVectorLocal), Color.yellow); // Drawing later would draw the ang. acceleration
            }

            gravityTorqueVectorLocal.Scale(new Vector3(1 / _objectRigidbody.inertiaTensor.x, 1 / _objectRigidbody.inertiaTensor.y, 1 / _objectRigidbody.inertiaTensor.z));

            //Debug.Log("[INFO] gravityTorqueVectorLocal: " + gravityTorqueVectorLocal);
        }
        else if (this.CompareTag("RightArm"))
        {
            distance3D = ((_objectRigidbody.worldCenterOfMass - transform.position) + (physicalForeArm.GetComponent<Rigidbody>().worldCenterOfMass - transform.position) + (physicalHand.GetComponent<Rigidbody>().worldCenterOfMass - transform.position)) / 3;
            gravityTorqueVector = Vector3.Cross(distance3D, (_objectRigidbody.mass + physicalForeArm.GetComponent<Rigidbody>().mass  + physicalHand.GetComponent<Rigidbody>().mass) * gravityAcc); // wrt. global coord. system
            gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // wrt. local coord. system

            if (drawTorques)
            {
                Debug.DrawRay(transform.position, distance3D, Color.cyan);
                Debug.DrawRay(transform.position, transform.TransformDirection(gravityTorqueVectorLocal), Color.yellow); // Drawing later would draw the ang. acceleration
            }

            gravityTorqueVectorLocal.Scale(new Vector3(1 / _objectRigidbody.inertiaTensor.x, 1 / _objectRigidbody.inertiaTensor.y, 1 / _objectRigidbody.inertiaTensor.z));

            //Debug.Log("[INFO] gravityTorqueVectorLocal: " + gravityTorqueVectorLocal);
        }
        else if (this.CompareTag("LeftHand"))
        {
            distance3D = _objectRigidbody.worldCenterOfMass - transform.position;
            gravityTorqueVector = Vector3.Cross(distance3D, _objectRigidbody.mass * gravityAcc); // wrt. global coord. system
            gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // wrt. local coord. system

            if (drawTorques)
            {
                Debug.DrawRay(transform.position, distance3D, Color.cyan);
                Debug.DrawRay(transform.position, transform.TransformDirection(gravityTorqueVectorLocal), Color.yellow); // Drawing later would draw the ang. acceleration
            }

            gravityTorqueVectorLocal.Scale(new Vector3(1 / _objectRigidbody.inertiaTensor.x, 1 / _objectRigidbody.inertiaTensor.y, 1 / _objectRigidbody.inertiaTensor.z));

            //Debug.Log("[INFO] gravityTorqueVectorLocal: " + gravityTorqueVectorLocal);
        }
        else if (this.CompareTag("LeftForeArm"))
        {
            distance3D = ((_objectRigidbody.worldCenterOfMass - transform.position) + (physicalHand.GetComponent<Rigidbody>().worldCenterOfMass - transform.position)) / 2;
            gravityTorqueVector = Vector3.Cross(distance3D, (_objectRigidbody.mass + physicalHand.GetComponent<Rigidbody>().mass) * gravityAcc); // wrt. global coord. system
            gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // wrt. local coord. system

            if (drawTorques)
            {
                Debug.DrawRay(transform.position, distance3D, Color.cyan);
                Debug.DrawRay(transform.position, transform.TransformDirection(gravityTorqueVectorLocal), Color.yellow); // Drawing later would draw the ang. acceleration
            }

            gravityTorqueVectorLocal.Scale(new Vector3(1 / _objectRigidbody.inertiaTensor.x, 1 / _objectRigidbody.inertiaTensor.y, 1 / _objectRigidbody.inertiaTensor.z));

            //Debug.Log("[INFO] gravityTorqueVectorLocal: " + gravityTorqueVectorLocal);
        }
        else if (this.CompareTag("LeftArm"))
        {
            distance3D = ((_objectRigidbody.worldCenterOfMass - transform.position) + (physicalForeArm.GetComponent<Rigidbody>().worldCenterOfMass - transform.position) + (physicalHand.GetComponent<Rigidbody>().worldCenterOfMass - transform.position)) / 3;
            gravityTorqueVector = Vector3.Cross(distance3D, (_objectRigidbody.mass + physicalForeArm.GetComponent<Rigidbody>().mass + physicalHand.GetComponent<Rigidbody>().mass) * gravityAcc); // wrt. global coord. system
            gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // wrt. local coord. system

            if (drawTorques)
            {
                Debug.DrawRay(transform.position, distance3D, Color.cyan);
                Debug.DrawRay(transform.position, transform.TransformDirection(gravityTorqueVectorLocal), Color.yellow); // Drawing later would draw the ang. acceleration
            }

            gravityTorqueVectorLocal.Scale(new Vector3(1 / _objectRigidbody.inertiaTensor.x, 1 / _objectRigidbody.inertiaTensor.y, 1 / _objectRigidbody.inertiaTensor.z));

            //Debug.Log("[INFO] gravityTorqueVectorLocal: " + gravityTorqueVectorLocal);
        }

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
            requiredTorque = ComputeRequiredTorque(_currentLocalOrientation, _currentGlobalOrientation,
                                                   _kinematicLocalOrientation, _kinematicGlobalOrientation,
                                                   DesiredLocalRotation,
                                                   this._objectRigidbody, gravityTorqueVectorLocal,
                                                   DELTATIME,
                                                   debugModeNormal, drawModeNormal);


            if (debugModeNormal)
            {
                if (globalMode)
                    Debug.Log("[INFO: " + this.gameObject.name + "] Final Normal PD Controller (Angle-axis) requiredTorque [Global]: " + requiredTorque);
                else
                    Debug.Log("[INFO: " + this.gameObject.name + "] Final Normal PD Controller (Angle-axis) requiredTorque [Local]: " + requiredTorque);
            }

            if (applyNormalTorque)
            {
                if (globalMode)
                    this._objectRigidbody.AddTorque(requiredTorque, ForceMode.Force); // Option (B) for Improved Torque [Global]
                else
                    this._objectRigidbody.AddRelativeTorque(requiredTorque, ForceMode.Force); // Option (B) for Improved Torque [Local]

                //this._objectRigidbody.AddRelativeTorque(requiredAngularAcceleration, ForceMode.Acceleration); // Option (A) for Angular Acceleration [Local]
            }
        }

        #endregion

        #region Antagonistic Controller

        /* 
         * 3. The third method (AntagonisticController) uses my own implementation for a Antagonistic Controllers using Axis-Angle representation.
         */

        if ((controllerType == Controller.AntagonisticController) && (activateAntagonisticPD))
        {
            requiredAntagonisticLocalTorque = _antagonisticControllerXYZ.ComputeRequiredAntagonisticTorque(minSoftLimitX, maxSoftLimitX, minSoftLimitY, maxSoftLimitY, minSoftLimitZ, maxSoftLimitZ,
                                                                                                           minHardLimitX, maxHardLimitX, minHardLimitY, maxHardLimitY, minHardLimitZ, maxHardLimitZ,
                                                                                                           _currentTransform,
                                                                                                           _currentLocalOrientation, 
                                                                                                           _kinematicLocalOrientation,
                                                                                                           this._objectRigidbody, gravityTorqueVectorLocal,
                                                                                                           DELTATIME,
                                                                                                           debugModeAntagonistic, drawModeAntagonistic,
                                                                                                           this.gameObject);

            if (debugModeAntagonistic)
            {
                Debug.Log("[INFO: " + this.gameObject.name + "] Antagonistic PD Controller (Angle-axis) requiredAntagonisticTorque: " + requiredAntagonisticLocalTorque);
            }

            // PLOT
            antTorqueX = requiredAntagonisticLocalTorque.x;
            antTorqueY = requiredAntagonisticLocalTorque.y;
            antTorqueZ = requiredAntagonisticLocalTorque.z;

            //if (applyAntTorque)
            //{
            //    this._objectRigidbody.AddRelativeTorque(requiredAntagonisticLocalTorque, ForceMode.Force); // [Local]
            //}

            if (applyAntTorqueX)
            {
                this._objectRigidbody.AddRelativeTorque(new Vector3(requiredAntagonisticLocalTorque.x, 0f, 0f), ForceMode.Force); // [Local]
            }
            if (applyAntTorqueY)
            {
                this._objectRigidbody.AddRelativeTorque(new Vector3(0f, requiredAntagonisticLocalTorque.y, 0f), ForceMode.Force); // [Local]
            }
            if (applyAntTorqueZ)
            {
                this._objectRigidbody.AddRelativeTorque(new Vector3(0f, 0f, requiredAntagonisticLocalTorque.z), ForceMode.Force); // [Local]
            }
        }

        #endregion

        #region Spring Gain

        //SetSpringGain();

        #endregion
    }

    #endregion

    #region Instance Methods

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
                                          Quaternion desiredLocalRotation, 
                                          Rigidbody _objectRigidbody, Vector3 gravityTorqueVectorLocal,
                                          float fixedDeltaTime, 
                                          bool debugModeNormal, bool drawModeNormal)
    {
        #region Orientations and Rotations

        /*

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

        */

        #endregion

        #region Rotations

        // Create Rotation from Current Local Orientation to Kinematic Local Orientation and convert to Angle-Axis
        newRotationLocal = Quaternion.Inverse(currentLocalOrientation) * kinematicLocalOrientation; // Works
        //newRotationLocal = kinematicLocalOrientation * Quaternion.Inverse(currentLocalOrientation); // Creates jitter above limits

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
        newRotationGlobal = kinematicGlobalOrientation * Quaternion.Inverse(currentGlobalOrientation); // Works

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

        if (debugModeNormal)
        {
            if (!globalMode)
            {
                Debug.Log("[INFO: " + this.gameObject.name + "] rotationNewAngleLocal: " + rotationNewAngleLocal);
                Debug.Log("[INFO: " + this.gameObject.name + "] rotationNewAxisLocal: " + rotationNewAxisLocal);  
            }
        }

        // Estimate Angle Error Global
        float newRotationErrorGlobal = rotationNewAngleGlobal;

        if (debugModeNormal)
        {
            if (globalMode)
            {
                Debug.Log("[INFO: " + this.gameObject.name + "] rotationNewAngleGlobal: " + rotationNewAngleGlobal);
                Debug.Log("[INFO: " + this.gameObject.name + "] rotationNewAxisGlobal: " + rotationNewAxisGlobal);  
            }
        }

        if (drawModeNormal)
        {
            if(globalMode)
                Debug.DrawRay(this.transform.position, rotationNewAxisGlobal, Color.blue);
            else
                Debug.DrawRay(this.transform.position, transform.TransformDirection(rotationNewAxisLocal), Color.blue);
        }

        #endregion

        #region Torque Estimation

        /*       1. Normal Torque Estimation (A)     */
        /* ========================================= */

        // Normal Torque [Local]
        //float torqueLocal = _normalPDController.GetOutput(newRotationErrorLocal, _objectRigidbody.angularVelocity.magnitude, fixedDeltaTime);
        //Debug.Log("[INFO] torqueLocal * rotationNewAxis: " + (torqueLocal * rotationNewAxisLocal));

        /*     2. Improved Torque Estimation (B)     */
        /* ========================================= */

        // Improved Torque [Global] and [Local]
        Vector3 torqueImprovedGlobal = _normalPDController.GetOutputAxisAngle(newRotationErrorGlobal, rotationNewAxisGlobal, _objectRigidbody.angularVelocity, fixedDeltaTime);
        Vector3 torqueImprovedLocal = _normalPDController.GetOutputAxisAngle(newRotationErrorLocal, rotationNewAxisLocal, _objectRigidbody.angularVelocity, fixedDeltaTime);

        if (debugModeNormal)
        {
            if(globalMode)
                Debug.Log("[INFO] torqueImprovedGlobal (acceleration): " + torqueImprovedGlobal);
            else
                Debug.Log("[INFO] torqueImprovedLocal (acceleration): " + torqueImprovedLocal); 
        }

        #endregion

        #region Inertia

        // Global -> WORKS (If doing it in world, we need to bring the rotation inertia to world. Then, inverse->no-inverse and have global.

        // Convert rotation of inertia tensor to global
        Quaternion rotInertia2World = _objectRigidbody.inertiaTensorRotation * transform.rotation;

        torqueImprovedGlobal = Quaternion.Inverse(rotInertia2World) * torqueImprovedGlobal;
        torqueImprovedGlobal.Scale(_objectRigidbody.inertiaTensor);
        torqueImprovedGlobal = rotInertia2World * torqueImprovedGlobal;

        // Local -> WORKS - inertiaTensorRotation already in local. Then, no-inverse->inverse and have local.
        torqueImprovedLocal = _objectRigidbody.inertiaTensorRotation * torqueImprovedLocal;
        torqueImprovedLocal.Scale(_objectRigidbody.inertiaTensor);
        torqueImprovedLocal = Quaternion.Inverse(_objectRigidbody.inertiaTensorRotation) * torqueImprovedLocal;

        if (debugModeNormal)
        {
            if(globalMode)
                Debug.Log("[INFO] torqueImprovedGlobal (torque): " + torqueImprovedGlobal);
            else
                Debug.Log("[INFO] torqueImprovedLocal (torque): " + torqueImprovedLocal); 
        }

        if(drawModeNormal)
        {
            if (globalMode)
                Debug.DrawRay(this.transform.position, torqueImprovedGlobal, Color.yellow);
            else
                Debug.DrawRay(this.transform.position, transform.TransformDirection(torqueImprovedLocal), Color.yellow);
        }

        #endregion

        /*       1. Normal Torque Estimation (A)     */
        /* ========================================= */

        //return torqueLocal * rotationNewAxisLocal; //

        /*     2. Improved Torque Estimation (B)     */
        /* ========================================= */

        if (globalMode)
            return torqueImprovedGlobal; // Working fine
        else
            return torqueImprovedLocal; // Working fine
    }

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

    void RightHandToggleXValueChanged(Toggle toggle)
    {
        if(this.gameObject.CompareTag("RightHand"))
        {
            if (toggle.isOn)
            {
                _rightHandGraph.SetTransparencyLine(_rightHandLineX, Color.red, 0.5f);
                _rightHandGraph.SetTransparencyPoint(_rightHandPointX, Color.red, 1f);

                _rightHandGraph.SetTransparencyLine(_rightHandLineXCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightHandGraph.SetTransparencyLine(_rightHandLineX, Color.red, 0f);
                _rightHandGraph.SetTransparencyPoint(_rightHandPointX, Color.red, 0f);

                _rightHandGraph.SetTransparencyLine(_rightHandLineXCurrent, Color.black, 0f);
            }
        }
    }

    void RightHandToggleYValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightHand"))
        {
            if (toggle.isOn)
            {
                _rightHandGraph.SetTransparencyLine(_rightHandLineY, Color.green, 0.5f);
                _rightHandGraph.SetTransparencyPoint(_rightHandPointY, Color.green, 1f);

                _rightHandGraph.SetTransparencyLine(_rightHandLineYCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightHandGraph.SetTransparencyLine(_rightHandLineY, Color.green, 0f);
                _rightHandGraph.SetTransparencyPoint(_rightHandPointY, Color.green, 0f);

                _rightHandGraph.SetTransparencyLine(_rightHandLineYCurrent, Color.black, 0f);
            } 
        }
    }

    void RightHandToggleZValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightHand"))
        {
            if (toggle.isOn)
            {
                _rightHandGraph.SetTransparencyLine(_rightHandLineZ, Color.blue, 0.5f);
                _rightHandGraph.SetTransparencyPoint(_rightHandPointZ, Color.blue, 1f);

                _rightHandGraph.SetTransparencyLine(_rightHandLineZCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightHandGraph.SetTransparencyLine(_rightHandLineZ, Color.blue, 0f);
                _rightHandGraph.SetTransparencyPoint(_rightHandPointZ, Color.blue, 0f);

                _rightHandGraph.SetTransparencyLine(_rightHandLineZCurrent, Color.black, 0f);
            } 
        }
    }

    void RightForeArmToggleXValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightForeArm"))
        {
            if (toggle.isOn)
            {
                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineX, Color.red, 0.5f);
                _rightForeArmGraph.SetTransparencyPoint(_rightForeArmPointX, Color.red, 1f);

                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineXCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineX, Color.red, 0f);
                _rightForeArmGraph.SetTransparencyPoint(_rightForeArmPointX, Color.red, 0f);

                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineXCurrent, Color.black, 0f);
            } 
        }
    }

    void RightForeArmToggleYValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightForeArm"))
        {
            if (toggle.isOn)
            {
                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineY, Color.green, 0.5f);
                _rightForeArmGraph.SetTransparencyPoint(_rightForeArmPointY, Color.green, 1f);

                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineYCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineY, Color.green, 0f);
                _rightForeArmGraph.SetTransparencyPoint(_rightForeArmPointY, Color.green, 0f);

                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineYCurrent, Color.black, 0f);
            } 
        }
    }

    void RightForeArmToggleZValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightForeArm"))
        {
            if (toggle.isOn)
            {
                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineZ, Color.blue, 0.5f);
                _rightForeArmGraph.SetTransparencyPoint(_rightForeArmPointZ, Color.blue, 1f);

                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineZCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineZ, Color.blue, 0f);
                _rightForeArmGraph.SetTransparencyPoint(_rightForeArmPointZ, Color.blue, 0f);

                _rightForeArmGraph.SetTransparencyLine(_rightForeArmLineZCurrent, Color.black, 0f);
            } 
        }
    }

    void RightArmToggleXValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightArm"))
        {
            if (toggle.isOn)
            {
                _rightArmGraph.SetTransparencyLine(_rightArmLineX, Color.red, 0.5f);
                _rightArmGraph.SetTransparencyPoint(_rightArmPointX, Color.red, 1f);

                _rightArmGraph.SetTransparencyLine(_rightArmLineXCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightArmGraph.SetTransparencyLine(_rightArmLineX, Color.red, 0f);
                _rightArmGraph.SetTransparencyPoint(_rightArmPointX, Color.red, 0f);

                _rightArmGraph.SetTransparencyLine(_rightArmLineXCurrent, Color.black, 0f);
            }
        }
    }

    void RightArmToggleYValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightArm"))
        {
            if (toggle.isOn)
            {
                _rightArmGraph.SetTransparencyLine(_rightArmLineY, Color.green, 0.5f);
                _rightArmGraph.SetTransparencyPoint(_rightArmPointY, Color.green, 1f);

                _rightArmGraph.SetTransparencyLine(_rightArmLineYCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightArmGraph.SetTransparencyLine(_rightArmLineY, Color.green, 0f);
                _rightArmGraph.SetTransparencyPoint(_rightArmPointY, Color.green, 0f);

                _rightArmGraph.SetTransparencyLine(_rightArmLineYCurrent, Color.black, 0f);
            }
        }
    }

    void RightArmToggleZValueChanged(Toggle toggle)
    {
        if (this.gameObject.CompareTag("RightArm"))
        {
            if (toggle.isOn)
            {
                _rightArmGraph.SetTransparencyLine(_rightArmLineZ, Color.blue, 0.5f);
                _rightArmGraph.SetTransparencyPoint(_rightArmPointZ, Color.blue, 1f);

                _rightArmGraph.SetTransparencyLine(_rightArmLineZCurrent, Color.black, 0.5f);
            }
            else
            {
                _rightArmGraph.SetTransparencyLine(_rightArmLineZ, Color.blue, 0f);
                _rightArmGraph.SetTransparencyPoint(_rightArmPointZ, Color.blue, 0f);

                _rightArmGraph.SetTransparencyLine(_rightArmLineZCurrent, Color.black, 0f);
            }
        }
    }

    #endregion

    #region Hand Springs

    /*
    private void SetSpringGain()
    {
        if (this.gameObject.CompareTag("LeftHand"))
        {
            SpringJoint spring = GetComponent<SpringJoint>();

            if (safetyRegionLeft.hasLeftTargetReached)
            {
                //spring.spring = 10000f;
            }
            else if (!safetyRegionLeft.hasLeftTargetReached)
            {
                //spring.spring = 10000f;
            }
        }
        else if (this.gameObject.CompareTag("RightHand"))
        {
            SpringJoint spring = GetComponent<SpringJoint>();

            if (safetyRegionRight.hasRightTargetReached)
            {
                //spring.spring = 10000f;
            }
            else if (!safetyRegionRight.hasRightTargetReached)
            {
                //spring.spring = 10000f;
            }
        }
    }
    */   

    /*
    private void OnCollisionEnter(Collision collision)
    {
        if (this.gameObject.CompareTag("LeftHand"))
        {
            //StartCoroutine(IncreaseSpring());
            //Debug.Log("DEBUG LEFT ENTERED!");
        }
        else if (this.gameObject.CompareTag("RightHand"))
        {
            //StartCoroutine(IncreaseSpring());
            //Debug.Log("DEBUG RIGHT ENTERED!");
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (this.gameObject.CompareTag("LeftHand"))
        {
            //SpringJoint spring = GetComponent<SpringJoint>();
            //spring.spring = 0f;

            //Debug.Log("DEBUG LEFT EXITED!");
        }
        else if (this.gameObject.CompareTag("RightHand"))
        {
            //SpringJoint spring = GetComponent<SpringJoint>();
            //spring.spring = 0f;

            //Debug.Log("DEBUG RIGHT EXITED!");
        }
    }

    
    IEnumerator IncreaseSpring()
    {
        SpringJoint spring = GetComponent<SpringJoint>();

        float timeElapsed = 0;
        while (timeElapsed < 1f)
        {
            spring.spring = Mathf.Lerp(0f, 10000f, timeElapsed / 1f);
            timeElapsed += Time.deltaTime;
            yield return null;
        }
        spring.spring = 10000f;
    }

    IEnumerator DecreaseSpring()
    {
        SpringJoint spring = GetComponent<SpringJoint>();

        float timeElapsed = 0;
        while (timeElapsed < 1f)
        {
            spring.spring = Mathf.Lerp(10000f, 0f, timeElapsed / 1f);
            timeElapsed += Time.deltaTime;
            yield return null;
        }
        spring.spring = 0f;
    }
    */

    #endregion
}

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticJointQuaternion : MonoBehaviour
{

    #region Read-only & Static Fields

    // Test - Quaternion Substraction
    private readonly AntagonisticPDControllerQuaternion _antagonisticPDControllerQuaternion = new AntagonisticPDControllerQuaternion(0.05f, 0.0f, 0.0f, 0.01f);

    // Test - Normal PD Controller
    private readonly AntagonisticPDController _normalPDController = new AntagonisticPDController(1f, 0f, 0f, 0.1f);
    
    // Test - Antagonistic PD Controller
    private readonly AntagonisticPDController _antPDController = new AntagonisticPDController(1f, 0f, 0f, 0.1f);
    private readonly AntagonisticPDController _antPDControllerY = new AntagonisticPDController(1f, 0f, 0f, 0.1f);
    private readonly AntagonisticPDController _antPDControllerZ = new AntagonisticPDController(1f, 0f, 0f, 0.1f);

    #endregion

    #region Fields

    private Transform _currentTransform;
    private Rigidbody _objectRigidbody;
    private Quaternion _initialLocalOrientation;
    private Quaternion _initialGlobalOrientation;

    [Header("General Settings")]
    public bool AntagonisticPD;
    public Transform kinematicArm;

    [Header("Normal PD Controller - Settings")]
    public bool globalMode = false;
    public bool applyTorque;
    public Vector3 torqueTest;
    public float Kp;
    public float Ki;
    public float Kd;

    [Header("Antagonistic Controller - Settings - X")]
    public bool applyAntTorqueX;
    public float torqueAppliedX;
    public float pLX;
    public float pHX;
    public float iX;
    public float dX;
    public float minAngleX;
    public float maxAngleX;
    public float slopeX;
    public float interceptX;

    [Header("Antagonistic Controller - Settings - Y")]
    public bool applyAntTorqueY;
    public float torqueAppliedY;
    public float pLY;
    public float pHY;
    public float iY;
    public float dY;
    public float minAngleY;
    public float maxAngleY;
    public float slopeY;
    public float interceptY;

    [Header("Antagonistic Controller - Settings - Z")]
    public bool applyAntTorqueZ;
    public float torqueAppliedZ;
    public float pLZ;
    public float pHZ;
    public float iZ;
    public float dZ;
    public float minAngleZ;
    public float maxAngleZ;
    public float slopeZ;
    public float interceptZ;

    [Header("Debug")]
    public bool printX;
    public bool printY;
    public bool printZ;
    public bool deployDesiredRotation;
    public Vector3 eulerRot;

    [Header("Configurable joint PD Controller mode")]
    public bool useJointPD = false;

    private Quaternion _currentLocalOrientation;
    private Quaternion _currentGlobalOrientation;
    private Quaternion _kinematicLocalOrientation;
    private Quaternion _kinematicGlobalOrientation;

    private ConfigurableJoint _jointAnt;
    private Vector3 distance3D;
    private Vector3 gravityAcc;
    private Vector3 gravityTorqueVector;
    private Vector3 gravityTorqueVectorLocal;
    private Quaternion newRotationLocal;
    private Quaternion newRotationGlobal;

    private Quaternion newRotationMinXLocal;
    private Quaternion newRotationMaxXLocal;
    private Quaternion kinRotationMinXLocal;
    private Quaternion kinRotationMaxXLocal;

    private Quaternion newRotationMinYLocal;
    private Quaternion newRotationMaxYLocal;
    private Quaternion kinRotationMinYLocal;
    private Quaternion kinRotationMaxYLocal;

    private Quaternion newRotationMinZLocal;
    private Quaternion newRotationMaxZLocal;
    private Quaternion kinRotationMinZLocal;
    private Quaternion kinRotationMaxZLocal;
    #endregion

    #region Instance Properties

    public Quaternion DesiredLocalRotation { get; set; }

    #endregion

    // ---- DIVISOR

    #region Old Variables

    /*
    [Header("Joint")]
    public Vector3 currentAngle;
    public Quaternion initialAngleQuaternion;
    public Quaternion currentAngleQuaternion;
    public Quaternion kinematicAngleQuaternion;
    public Quaternion resultAngleQuaternion;
    public Transform kinematicArm;

    [Header("Antagonistic - X")]
    public bool applyTorqueX;
    public float torqueAppliedX;
    public float pLX;
    public float pHX;
    public float iX;
    public float dX;
    public float minAngleX;
    public float maxAngleX;
    public float slopeX;
    public float interceptX;
    public float eqAngleX;
    private AntagonisticPDController _AntPIDX;

    [Header("Antagonistic - Y")]
    public bool applyTorqueY;
    public float torqueAppliedY;
    public float pLY;
    public float pHY;
    public float iY;
    public float dY;
    public float minAngleY;
    public float maxAngleY;
    public float slopeY;
    public float interceptY;
    public float eqAngleY;
    private AntagonisticPDController _AntPIDY;

    [Header("Antagonistic - Z")]
    public bool applyTorqueZ;
    public float torqueAppliedZ;
    public float pLZ;
    public float pHZ;
    public float iZ;
    public float dZ;
    public float minAngleZ;
    public float maxAngleZ;
    public float slopeZ;
    public float interceptZ;
    public float eqAngleZ;
    private AntagonisticPDController _AntPIDZ;

    [Header("Debug")]
    public bool toogleRays = false;
    public bool printTorqueX = false;
    public bool printTorqueY = false;
    public bool printTorqueZ = false;
    public Rigidbody _rbAnt;
    public ConfigurableJoint _jointAnt;
    public Transform sphereAnt;

    [Header("Experimental")]
    public Vector3 distance3D;
    public bool applyExternalForce;
    public Vector3 externalForce;
    public float externalTorqueMagnitudeX;
    public Vector3 externalTorqueVector;
    public Vector3 externalTorqueVectorLocal;
    public bool applyGravity;
    public Vector3 gravityAcc;
    public float gravityTorqueMagnitudeX;
    public Vector3 gravityTorqueVector;
    public Vector3 gravityTorqueVectorLocal;
    */

    #endregion

    private void Awake()
    {
        // Only use to see if there is a transform
        this._currentTransform = transform;

        // Initial - They do not update!
        this._initialLocalOrientation = transform.localRotation;
        this._initialGlobalOrientation = transform.rotation;

        this._objectRigidbody = GetComponent<Rigidbody>();
        this._jointAnt = GetComponent<ConfigurableJoint>();
    }

    private void Update()
    {

    }

    private void FixedUpdate()
    {
        if (DesiredLocalRotation == null || this._currentTransform == null || this._objectRigidbody == null)
        {
            return;
        }

        #region Gains

        // Test - Quaternion Substraction
        //this._antagonisticPDControllerQuaternion.KPL = this.Kpl;
        //this._antagonisticPDControllerQuaternion.KPH = this.Kph;
        //this._antagonisticPDControllerQuaternion.KI = this.Ki;
        //this._antagonisticPDControllerQuaternion.KD = this.Kd;

        // Test - Normal PD Controller
        this._normalPDController.KPL = this.Kp;
        this._normalPDController.KPH = 0f; // this.Kph
        this._normalPDController.KI = this.Ki;
        this._normalPDController.KD = this.Kd;

        // Test - Antagonistic PD Controller
        this._antPDController.KPL = this.pLX;
        this._antPDController.KPH = this.pHX;
        this._antPDController.KI = this.iX;
        this._antPDController.KD = this.dX;

        #endregion

        // Get target orientation that we need to mimic (kinematic)
        _kinematicLocalOrientation = kinematicArm.transform.localRotation;
        _kinematicGlobalOrientation = kinematicArm.transform.rotation;

        // Get current orientation at each frame that we need to move (ragdoll)
        _currentLocalOrientation = transform.localRotation;
        _currentGlobalOrientation = transform.rotation;

        // The first method (normal PD) already sets the target at the configurable joint. Otherwise, just retrieve target quaternion and turn off normal PD.
        if (useJointPD)
        {
            // Local
            // In case we need it later, save it as well
            DesiredLocalRotation = ConfigurableJointExtensions.GetTargetRotationLocal(_jointAnt, _kinematicLocalOrientation, _initialLocalOrientation, _currentLocalOrientation, transform);
            ConfigurableJointExtensions.SetTargetRotationLocal(_jointAnt, _kinematicLocalOrientation, _initialLocalOrientation);

            // Bug, which I do not find: When activating useJointPD, and coming back to torque, it doesn't work. It looks like it changes something in the joint.
            _jointAnt.rotationDriveMode = RotationDriveMode.XYAndZ;
            var angularXDrive = _jointAnt.angularXDrive;
            angularXDrive.positionSpring = 1600f;
            angularXDrive.positionDamper = 11f;
            angularXDrive.maximumForce = Mathf.Infinity;
            _jointAnt.angularXDrive = angularXDrive;
            var angularYZDrive = _jointAnt.angularYZDrive;
            angularYZDrive.positionSpring = 1600f;
            angularYZDrive.positionDamper = 11f;
            angularYZDrive.maximumForce = Mathf.Infinity;
            _jointAnt.angularYZDrive = angularYZDrive;
        }
        else
        {
            // Local
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

        // Calculate forces relative to the Rigid Body
        // Distance from root to the RB COM
        distance3D = _objectRigidbody.worldCenterOfMass - transform.position;

        // 1. Gravity force and generated torque
        gravityAcc = Physics.gravity;
        gravityTorqueVector = Vector3.Cross(distance3D, _objectRigidbody.mass * gravityAcc); // Remember: wrt. global coord. system
        gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // Remember: wrt. local coord. system

        // ------
        
        if(!AntagonisticPD)
        {
            // Test - Normal PD Controller (Angle-axis)
            // ----------------------------------------

            Vector3 requiredTorque = ComputeRequiredTorque(_currentLocalOrientation,
                                                           _currentGlobalOrientation,
                                                           _kinematicLocalOrientation,
                                                           _kinematicGlobalOrientation,
                                                           DesiredLocalRotation,
                                                           this._objectRigidbody.angularVelocity,
                                                           gravityTorqueVectorLocal,
                                                           Time.fixedDeltaTime);

            Debug.Log("[" + this.gameObject.name + "] Normal PD Controller (Angle-axis) requiredTorque: " + requiredTorque);

            if ((!useJointPD) && (applyTorque))
            {
                // TODO: ForceMode.Acceleration or Force? Taking into account Inertia Matrix?

                // WRT GLOBAL -> Used for improved torque, that calculated in Global system
                if (globalMode)
                    this._objectRigidbody.AddTorque(requiredTorque, ForceMode.Force); // -> B (Global)
                else
                    this._objectRigidbody.AddRelativeTorque(requiredTorque, ForceMode.Force); // -> B (Local)

                // WRT LOCAL -> Used for first torque version
                //this._objectRigidbody.AddRelativeTorque(requiredAngularAcceleration, ForceMode.Acceleration); // -> A.

                // WRT TEST
                //this._objectRigidbody.AddRelativeTorque(torqueTest, ForceMode.Force);

                /*
                 * ForceMode.Force: Interprets the input as torque (measured in Newton-metres), and changes the angular velocity by the value of torque * DT / mass. 
                 * The effect depends on the simulation step length and the mass of the body.
                 * ForceMode.Acceleration: Interprets the parameter as angular acceleration (measured in degrees per second squared), and changes the angular velocity by the value of torque * DT. 
                 * The effect depends on the simulation step length but does not depend on the mass of the body.
                 * ForceMode.Impulse: Interprets the parameter as an angular momentum (measured in kilogram-meters-squared per second), and changes the angular velocity by the value of torque / mass. 
                 * The effect depends on the mass of the body but doesn't depend on the simulation step length.
                 * ForceMode.VelocityChange: Interprets the parameter as a direct angular velocity change (measured in degrees per second), and changes the angular velocity by the value of torque. 
                 * The effect doesn't depend on the mass of the body and the simulation step length.
                 * */
            }
        }
        else
        {
            // Test - Antagonistic PD Controller
            // ---------------------------------

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

            Debug.Log("[" + this.gameObject.name + "] requiredAntagonisticTorqueX: " + requiredAntagonisticTorqueX);
            Debug.Log("[" + this.gameObject.name + "] requiredAntagonisticTorqueY: " + requiredAntagonisticTorqueY);
            Debug.Log("[" + this.gameObject.name + "] requiredAntagonisticTorqueZ: " + requiredAntagonisticTorqueZ);

            if ((!useJointPD) && (applyAntTorqueX))
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorqueX, ForceMode.Force);
            }

            if ((!useJointPD) && (applyAntTorqueY))
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorqueY, ForceMode.Force);
            }

            if ((!useJointPD) && (applyAntTorqueZ))
            {
                this._objectRigidbody.AddRelativeTorque(requiredAntagonisticTorqueZ, ForceMode.Force);
            }
        }

        // ---

        #region Test - Quaternion Substraction

        // TEST: Previous version with Quaternion substraction

        /*
        // The PD controller takes the current orientation of an object, its desired orientation and the current angular velocity
        // and returns the required angular acceleration to rotate towards the desired orientation.        
        Vector3 requiredAngularAccelerationX = this._antagonisticPDControllerQuaternion.ComputeRequiredAngularAccelerationX(angleX, 0f, 0f,
                                                                                                     this._currentTransform.localRotation,
                                                                                                     DesiredLocalOrientation,
                                                                                                     this._objectRigidbody.angularVelocity,
                                                                                                     gravityTorqueVectorLocal,
                                                                                                     Time.fixedDeltaTime, 
                                                                                                     printX);


       
        Vector3 requiredAngularAccelerationY = this._antagonisticPDControllerQuaternion.ComputeRequiredAngularAccelerationY(0f, angleY, 0f,
                                                                                                     this._currentTransform.localRotation,
                                                                                                     DesiredLocalOrientation,
                                                                                                     this._objectRigidbody.angularVelocity,
                                                                                                     gravityTorqueVectorLocal,
                                                                                                     Time.fixedDeltaTime,
                                                                                                     printY);

        Vector3 requiredAngularAccelerationZ = this._antagonisticPDControllerQuaternion.ComputeRequiredAngularAccelerationZ(0f, 0f, angleZ,
                                                                                                     this._currentTransform.localRotation,
                                                                                                     DesiredLocalOrientation,
                                                                                                     this._objectRigidbody.angularVelocity,
                                                                                                     gravityTorqueVectorLocal,
                                                                                                     Time.fixedDeltaTime,
                                                                                                     printZ);
        
        // TODO: Changed Acceleation by Force. Use AddTorque() or AddRelativeTorque()?
        if (!useJointPD)
        {
            this._objectRigidbody.AddTorque(requiredAngularAccelerationX, ForceMode.Acceleration);
            this._objectRigidbody.AddTorque(requiredAngularAccelerationY, ForceMode.Force);
            this._objectRigidbody.AddTorque(requiredAngularAccelerationZ, ForceMode.Force);
        }
        */

        #endregion
    }

    /// <summary>
    /// 
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

        #region TEST - Axis-angle

        /* Estimation of orientations and rotations */
        /* ======================================== */

        Quaternion minAngleQuaternionX = Quaternion.Euler(new Vector3(minAngleX, 0f, 0f));
        Quaternion maxAngleQuaternionX = Quaternion.Euler(new Vector3(maxAngleX, 0f, 0f));

        // Target Local Rotation in Angle-axis
        newRotationMinXLocal = minAngleQuaternionX * Quaternion.Inverse(currentLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationMinXLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationMinXLocal.x = -newRotationMinXLocal.x;
            newRotationMinXLocal.y = -newRotationMinXLocal.y;
            newRotationMinXLocal.z = -newRotationMinXLocal.z;
            newRotationMinXLocal.w = -newRotationMinXLocal.w;
        }
        float rotationNewAngleMinXLocal = 0.0f;
        Vector3 rotationNewAxisMinXLocal = Vector3.zero;
        newRotationMinXLocal.ToAngleAxis(out rotationNewAngleMinXLocal, out rotationNewAxisMinXLocal);
        rotationNewAxisMinXLocal.Normalize();

        // Target Local Rotation in Angle-axis
        newRotationMaxXLocal = maxAngleQuaternionX * Quaternion.Inverse(currentLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationMaxXLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationMaxXLocal.x = -newRotationMaxXLocal.x;
            newRotationMaxXLocal.y = -newRotationMaxXLocal.y;
            newRotationMaxXLocal.z = -newRotationMaxXLocal.z;
            newRotationMaxXLocal.w = -newRotationMaxXLocal.w;
        }
        float rotationNewAngleMaxXLocal = 0.0f;
        Vector3 rotationNewAxisMaxXLocal = Vector3.zero;
        newRotationMaxXLocal.ToAngleAxis(out rotationNewAngleMaxXLocal, out rotationNewAxisMaxXLocal);
        rotationNewAxisMaxXLocal.Normalize();

        float errorMinXLocal = (rotationNewAngleMinXLocal * rotationNewAxisMinXLocal).x;
        float errorMaxXLocal = (rotationNewAngleMaxXLocal * rotationNewAxisMaxXLocal).x;
        Debug.Log("errorMinXLocal: " + errorMinXLocal); // Working
        Debug.Log("errorMaxXLocal: " + errorMaxXLocal); // Working

        // Test - It's working
        //pHX = pLX * slopeX + interceptX;
        //this._antPDController.KPL = pLX;
        //this._antPDController.KPH = pHX;
        //torqueAppliedX = _antPDController.GetOutput(errorMinXLocal, errorMaxXLocal, angularVelocity.magnitude, Time.fixedDeltaTime);

        // Now, make real slope/intercept

        // Target Local Rotation in Angle-axis
        kinRotationMinXLocal = minAngleQuaternionX * Quaternion.Inverse(kinematicLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinRotationMinXLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinRotationMinXLocal.x = -kinRotationMinXLocal.x;
            kinRotationMinXLocal.y = -kinRotationMinXLocal.y;
            kinRotationMinXLocal.z = -kinRotationMinXLocal.z;
            kinRotationMinXLocal.w = -kinRotationMinXLocal.w;
        }
        float rotationKinAngleMinXLocal = 0.0f;
        Vector3 rotationKinAxisMinXLocal = Vector3.zero;
        kinRotationMinXLocal.ToAngleAxis(out rotationKinAngleMinXLocal, out rotationKinAxisMinXLocal);
        rotationKinAxisMinXLocal.Normalize();

        // Target Local Rotation in Angle-axis
        kinRotationMaxXLocal = maxAngleQuaternionX * Quaternion.Inverse(kinematicLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinRotationMaxXLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinRotationMaxXLocal.x = -kinRotationMaxXLocal.x;
            kinRotationMaxXLocal.y = -kinRotationMaxXLocal.y;
            kinRotationMaxXLocal.z = -kinRotationMaxXLocal.z;
            kinRotationMaxXLocal.w = -kinRotationMaxXLocal.w;
        }
        float rotationKinAngleMaxXLocal = 0.0f;
        Vector3 rotationKinAxisMaxXLocal = Vector3.zero;
        kinRotationMaxXLocal.ToAngleAxis(out rotationKinAngleMaxXLocal, out rotationKinAxisMaxXLocal);
        rotationKinAxisMaxXLocal.Normalize();

        interceptX = 0f;
        slopeX = ((rotationKinAngleMinXLocal * rotationKinAxisMinXLocal).x) / (-(rotationKinAngleMaxXLocal * rotationKinAxisMaxXLocal).x);
        pHX = pLX * slopeX + interceptX;
        this._antPDController.KPL = pLX;
        this._antPDController.KPH = pHX;
        torqueAppliedX = _antPDController.GetOutput(errorMinXLocal, errorMaxXLocal, angularVelocity.magnitude, Time.fixedDeltaTime);

        // Seems to work! Review code, test for each axis, and use multiple 

        #endregion
   
        return new Vector3(torqueAppliedX, 0f, 0f);
    }

    /// <summary>
    /// 
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

        #region TEST - Axis-angle

        /* Estimation of orientations and rotations */
        /* ======================================== */

        Quaternion minAngleQuaternionY = Quaternion.Euler(new Vector3(0f, minAngleY, 0f));
        Quaternion maxAngleQuaternionY = Quaternion.Euler(new Vector3(0f, maxAngleY, 0f));

        // Target Local Rotation in Angle-axis
        newRotationMinYLocal = minAngleQuaternionY * Quaternion.Inverse(currentLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationMinYLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationMinYLocal.x = -newRotationMinYLocal.x;
            newRotationMinYLocal.y = -newRotationMinYLocal.y;
            newRotationMinYLocal.z = -newRotationMinYLocal.z;
            newRotationMinYLocal.w = -newRotationMinYLocal.w;
        }
        float rotationNewAngleMinYLocal = 0.0f;
        Vector3 rotationNewAxisMinYLocal = Vector3.zero;
        newRotationMinYLocal.ToAngleAxis(out rotationNewAngleMinYLocal, out rotationNewAxisMinYLocal);
        rotationNewAxisMinYLocal.Normalize();

        // Target Local Rotation in Angle-axis
        newRotationMaxYLocal = maxAngleQuaternionY * Quaternion.Inverse(currentLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationMaxYLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationMaxYLocal.x = -newRotationMaxYLocal.x;
            newRotationMaxYLocal.y = -newRotationMaxYLocal.y;
            newRotationMaxYLocal.z = -newRotationMaxYLocal.z;
            newRotationMaxYLocal.w = -newRotationMaxYLocal.w;
        }
        float rotationNewAngleMaxYLocal = 0.0f;
        Vector3 rotationNewAxisMaxYLocal = Vector3.zero;
        newRotationMaxYLocal.ToAngleAxis(out rotationNewAngleMaxYLocal, out rotationNewAxisMaxYLocal);
        rotationNewAxisMaxYLocal.Normalize();

        float errorMinYLocal = (rotationNewAngleMinYLocal * rotationNewAxisMinYLocal).y;
        float errorMaxYLocal = (rotationNewAngleMaxYLocal * rotationNewAxisMaxYLocal).y;
        Debug.Log("errorMinYLocal: " + errorMinYLocal); // Not working?
        Debug.Log("errorMaxYLocal: " + errorMaxYLocal); // Not working?

        // Test - It's working
        //pHX = pLX * slopeX + interceptX;
        //this._antPDController.KPL = pLX;
        //this._antPDController.KPH = pHX;
        //torqueAppliedX = _antPDController.GetOutput(errorMinXLocal, errorMaxXLocal, angularVelocity.magnitude, Time.fixedDeltaTime);

        // Now, make real slope/intercept

        // Target Local Rotation in Angle-axis
        kinRotationMinYLocal = minAngleQuaternionY * Quaternion.Inverse(kinematicLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinRotationMinYLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinRotationMinYLocal.x = -kinRotationMinYLocal.x;
            kinRotationMinYLocal.y = -kinRotationMinYLocal.y;
            kinRotationMinYLocal.z = -kinRotationMinYLocal.z;
            kinRotationMinYLocal.w = -kinRotationMinYLocal.w;
        }
        float rotationKinAngleMinYLocal = 0.0f;
        Vector3 rotationKinAxisMinYLocal = Vector3.zero;
        kinRotationMinYLocal.ToAngleAxis(out rotationKinAngleMinYLocal, out rotationKinAxisMinYLocal);
        rotationKinAxisMinYLocal.Normalize();

        // Target Local Rotation in Angle-axis
        kinRotationMaxYLocal = maxAngleQuaternionY * Quaternion.Inverse(kinematicLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinRotationMaxYLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinRotationMaxYLocal.x = -kinRotationMaxYLocal.x;
            kinRotationMaxYLocal.y = -kinRotationMaxYLocal.y;
            kinRotationMaxYLocal.z = -kinRotationMaxYLocal.z;
            kinRotationMaxYLocal.w = -kinRotationMaxYLocal.w;
        }
        float rotationKinAngleMaxYLocal = 0.0f;
        Vector3 rotationKinAxisMaxYLocal = Vector3.zero;
        kinRotationMaxYLocal.ToAngleAxis(out rotationKinAngleMaxYLocal, out rotationKinAxisMaxYLocal);
        rotationKinAxisMaxYLocal.Normalize();

        interceptY = 0f;
        slopeY = ((rotationKinAngleMinYLocal * rotationKinAxisMinYLocal).y) / (-(rotationKinAngleMaxYLocal * rotationKinAxisMaxYLocal).y);
        pHY = pLY * slopeY + interceptY;
        this._antPDControllerY.KPL = pLY;
        this._antPDControllerY.KPH = pHY;
        torqueAppliedY = _antPDControllerY.GetOutput(errorMinYLocal, errorMaxYLocal, angularVelocity.magnitude, Time.fixedDeltaTime);

        // Seems to work! Review code, test for each axis, and use multiple 

        #endregion

        return new Vector3(0f, torqueAppliedY, 0f);
    }

    /// <summary>
    /// 
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

        #region TEST - Axis-angle

        /* Estimation of orientations and rotations */
        /* ======================================== */

        Quaternion minAngleQuaternionZ = Quaternion.Euler(new Vector3(0f, 0f, minAngleZ));
        Quaternion maxAngleQuaternionZ = Quaternion.Euler(new Vector3(0f, 0f, maxAngleZ));

        // Target Local Rotation in Angle-axis
        newRotationMinZLocal = minAngleQuaternionZ * Quaternion.Inverse(currentLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationMinZLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationMinZLocal.x = -newRotationMinZLocal.x;
            newRotationMinZLocal.y = -newRotationMinZLocal.y;
            newRotationMinZLocal.z = -newRotationMinZLocal.z;
            newRotationMinZLocal.w = -newRotationMinZLocal.w;
        }
        float rotationNewAngleMinZLocal = 0.0f;
        Vector3 rotationNewAxisMinZLocal = Vector3.zero;
        newRotationMinZLocal.ToAngleAxis(out rotationNewAngleMinZLocal, out rotationNewAxisMinZLocal);
        rotationNewAxisMinZLocal.Normalize();

        // Target Local Rotation in Angle-axis
        newRotationMaxZLocal = maxAngleQuaternionZ * Quaternion.Inverse(currentLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (newRotationMaxZLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            newRotationMaxZLocal.x = -newRotationMaxZLocal.x;
            newRotationMaxZLocal.y = -newRotationMaxZLocal.y;
            newRotationMaxZLocal.z = -newRotationMaxZLocal.z;
            newRotationMaxZLocal.w = -newRotationMaxZLocal.w;
        }
        float rotationNewAngleMaxZLocal = 0.0f;
        Vector3 rotationNewAxisMaxZLocal = Vector3.zero;
        newRotationMaxZLocal.ToAngleAxis(out rotationNewAngleMaxZLocal, out rotationNewAxisMaxZLocal);
        rotationNewAxisMaxZLocal.Normalize();

        float errorMinZLocal = (rotationNewAngleMinZLocal * rotationNewAxisMinZLocal).z;
        float errorMaxZLocal = (rotationNewAngleMaxZLocal * rotationNewAxisMaxZLocal).z;
        Debug.Log("errorMinZLocal: " + errorMinZLocal); // Not working?
        Debug.Log("errorMaxZLocal: " + errorMaxZLocal); // Not working?

        // Test - It's working
        //pHX = pLX * slopeX + interceptX;
        //this._antPDController.KPL = pLX;
        //this._antPDController.KPH = pHX;
        //torqueAppliedX = _antPDController.GetOutput(errorMinXLocal, errorMaxXLocal, angularVelocity.magnitude, Time.fixedDeltaTime);

        // Now, make real slope/intercept

        // Target Local Rotation in Angle-axis
        kinRotationMinZLocal = minAngleQuaternionZ * Quaternion.Inverse(kinematicLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinRotationMinZLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinRotationMinZLocal.x = -kinRotationMinZLocal.x;
            kinRotationMinZLocal.y = -kinRotationMinZLocal.y;
            kinRotationMinZLocal.z = -kinRotationMinZLocal.z;
            kinRotationMinZLocal.w = -kinRotationMinZLocal.w;
        }
        float rotationKinAngleMinZLocal = 0.0f;
        Vector3 rotationKinAxisMinZLocal = Vector3.zero;
        kinRotationMinZLocal.ToAngleAxis(out rotationKinAngleMinZLocal, out rotationKinAxisMinZLocal);
        rotationKinAxisMinZLocal.Normalize();

        // Target Local Rotation in Angle-axis
        kinRotationMaxZLocal = maxAngleQuaternionZ * Quaternion.Inverse(kinematicLocalOrientation); // Works
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (kinRotationMaxZLocal.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            kinRotationMaxZLocal.x = -kinRotationMaxZLocal.x;
            kinRotationMaxZLocal.y = -kinRotationMaxZLocal.y;
            kinRotationMaxZLocal.z = -kinRotationMaxZLocal.z;
            kinRotationMaxZLocal.w = -kinRotationMaxZLocal.w;
        }
        float rotationKinAngleMaxZLocal = 0.0f;
        Vector3 rotationKinAxisMaxZLocal = Vector3.zero;
        kinRotationMaxZLocal.ToAngleAxis(out rotationKinAngleMaxZLocal, out rotationKinAxisMaxZLocal);
        rotationKinAxisMaxZLocal.Normalize();

        interceptZ = 0f;
        slopeZ = ((rotationKinAngleMinZLocal * rotationKinAxisMinZLocal).z) / (-(rotationKinAngleMaxZLocal * rotationKinAxisMaxZLocal).z);
        pHZ = pLZ * slopeZ + interceptZ;
        this._antPDControllerZ.KPL = pLZ;
        this._antPDControllerZ.KPH = pHZ;
        torqueAppliedZ = _antPDControllerZ.GetOutput(errorMinZLocal, errorMaxZLocal, angularVelocity.magnitude, Time.fixedDeltaTime);

        // Seems to work! Review code, test for each axis, and use multiple 

        #endregion

        return new Vector3(0f, 0f, torqueAppliedZ);
    }

    /// <summary>
    /// 
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

        /* Estimation of orientations and rotations */
        /* ======================================== */

        // Current Local Orientation in Angle-axis
        float currentAngle = 0.0f;
        Vector3 currentAxis = Vector3.zero;
        currentLocalOrientation.ToAngleAxis(out currentAngle, out currentAxis);

        // Target Local Orientation in Angle-axis
        float targetAngle = 0.0f;
        Vector3 targetAxis = Vector3.zero;
        kinematicLocalOrientation.ToAngleAxis(out targetAngle, out targetAxis);

        // ---

        // Target Rotation in Angle-axis -> Not used, the quaternion is wrt. joint coordinate system, valid for integrated PD in the conf. joint only.
        float rotationAngle = 0.0f;
        Vector3 rotationAxis = Vector3.zero;
        desiredLocalRotation.ToAngleAxis(out rotationAngle, out rotationAxis); // <--- Wrong Quaternion, is wrt. joint coordinate system!

        // ---

        // Target Local Rotation in Angle-axis
        newRotationLocal = kinematicLocalOrientation * Quaternion.Inverse(currentLocalOrientation); // Works
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

        // ---

        // Target Global Rotation in Angle-axis
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

        /*             Test Rotations               */
        /* ======================================== */

        if (deployDesiredRotation)
        {
            // 1. DesiredLocalRotation
            //transform.localRotation = desiredLocalRotation; // Wrong! -> The rotation is in the joint space, while local rotation in local space.

            // 2. Apply rotation quaternion (LOCAL)
            //transform.localRotation = newRotationLocal * transform.localRotation; // Works perfectly - Still some jittering which I don't know where it comes from

            // 3. Apply rotation quaternion (GLOBAL)
            //transform.rotation = newRotationGlobal * transform.rotation; // Does not work entirely, are global rotations - Still some jittering which I don't know where it comes from

        }

        /*                Controllers               */
        /* ======================================== */

        // Estimate Angle Error Local
        float newRotationErrorLocal = rotationNewAngleLocal;
        Debug.Log("[" + this.gameObject.name + "] newRotationErrorLocal: " + newRotationErrorLocal);

        // Rotation Axis Local
        Debug.Log("[" + this.gameObject.name + "] rotationNewAxisLocal: " + rotationNewAxisLocal);
        Debug.DrawRay(this.transform.position, rotationNewAxisLocal, Color.blue);

        // Estimate Angle Error Global
        float newRotationErrorGlobal = rotationNewAngleGlobal;
        Debug.Log("[" + this.gameObject.name + "] newRotationErrorGlobal: " + newRotationErrorGlobal);

        // Rotation Axis Global
        Debug.Log("[" + this.gameObject.name + "] rotationNewAxisGlobal: " + rotationNewAxisGlobal);
        Debug.DrawRay(this.transform.position, rotationNewAxisGlobal, Color.blue);

        /*       1. Normal Torque Estimation        */
        /* ======================================== */

        // Normal Torque (Local)
        float torqueLocal = _normalPDController.GetOutputPD(newRotationErrorLocal, angularVelocity.magnitude, Time.fixedDeltaTime);
        Debug.Log("[INFO] Torque Estimation (LOCAL) -> torqueLocal * rotationNewAxis: " + (torqueLocal * rotationNewAxisLocal));

        /*     2. Improved Torque Estimation        */
        /* ======================================== */

        // Improved Torque (Global and Local)
        Vector3 torqueImprovedGlobal = _normalPDController.GetOutputImprovedPD(newRotationErrorGlobal, rotationNewAxisGlobal, angularVelocity, Time.fixedDeltaTime);
        Vector3 torqueImprovedLocal = _normalPDController.GetOutputImprovedPD(newRotationErrorLocal, rotationNewAxisLocal, angularVelocity, Time.fixedDeltaTime);

        // Convert rotation of inertia tensor to global
        Quaternion rotInertia2World = _objectRigidbody.inertiaTensorRotation * transform.rotation;

        // ****
        torqueImprovedGlobal = Quaternion.Inverse(rotInertia2World) * torqueImprovedGlobal;
        torqueImprovedGlobal.Scale(_objectRigidbody.inertiaTensor);
        torqueImprovedGlobal = rotInertia2World * torqueImprovedGlobal;
        Debug.Log("[INFO] Torque Estimation (GLOBAL) -> torqueImprovedGlobal: " + (torqueImprovedGlobal));
        // ****

        // ****
        //torqueImprovedLocal = Quaternion.Inverse(rotInertia2World) * torqueImprovedLocal;
        //torqueImprovedLocal.Scale(_objectRigidbody.inertiaTensor);
        torqueImprovedLocal = _objectRigidbody.inertiaTensorRotation * torqueImprovedLocal;
        Debug.Log("[INFO] Torque Estimation (LOCAL) -> torqueImprovedLocal: " + (torqueImprovedLocal));
        // ****

        /*                  Returns                 */
        /* ======================================== */

        // 1. Torque Estimation (LOCAL)
        //return torqueLocal * rotationNewAxisLocal; // -> A.

        // 2. Torque Estimation (GLOBAL)
        if(globalMode)
            return torqueImprovedGlobal; // -> B (Global) - Working fine
        else
            return torqueImprovedLocal; // -> B (Local) - Not entirely working
    }

    #region AntagonisticJoint.cs

    /*
    // Start is called before the first frame update
    void Start()
    {
        _AntPIDX = new AntagonisticPDController(pLX, pHX, iX, dX);
        _AntPIDY = new AntagonisticPDController(pLY, pHY, iY, dY);
        _AntPIDZ = new AntagonisticPDController(pLZ, pHZ, iZ, dZ);
    }
    */

    /*
    // Update is called once per frame
    void Update()
    {
        if (toogleRays)
        {
            // Draw equilibrium angle (GREEN)
            Debug.DrawRay(transform.position,
                          Quaternion.AngleAxis(eqAngleZ, transform.parent.forward) * Quaternion.AngleAxis(eqAngleX, transform.parent.right) * transform.parent.up * 10f,
                          Color.green);

            // Draw up-axis of body limb (BLUE)
            Debug.DrawRay(transform.position, transform.up * 10f, Color.blue);

            // Draw limits of the antagonistic controller
            if (applyTorqueX)
            {
                Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxAngleX, transform.parent.right) * transform.parent.up * 10f, Color.red);
                Debug.DrawRay(transform.position, Quaternion.AngleAxis(minAngleX, transform.parent.right) * transform.parent.up * 10f, Color.red);
            }

            if (applyTorqueY)
            {
                Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxAngleY, transform.parent.forward) * transform.parent.up * 10f, Color.red);
                Debug.DrawRay(transform.position, Quaternion.AngleAxis(minAngleY, transform.parent.forward) * transform.parent.up * 10f, Color.red);
            }

            if (applyTorqueZ)
            {
                Debug.DrawRay(transform.position, Quaternion.AngleAxis(maxAngleZ, transform.parent.forward) * transform.parent.up * 10f, Color.red);
                Debug.DrawRay(transform.position, Quaternion.AngleAxis(minAngleZ, transform.parent.forward) * transform.parent.up * 10f, Color.red);
            }

            // Draw line conecting root with COM
            Debug.DrawLine(transform.position, _rbAnt.worldCenterOfMass, Color.yellow);

            // Draw external force
            //Debug.DrawRay(_rbAnt.worldCenterOfMass, _rbAnt.mass * gravityAcc, Color.yellow);
            //Debug.DrawRay(_rbAnt.worldCenterOfMass, externalForce, Color.magenta);

            // Draw torque produced by external
            //Debug.DrawRay(transform.position, gravityTorqueVector, Color.yellow);
            //Debug.DrawRay(transform.position, externalTorqueVector, Color.magenta);
        }
    }
    */

    /*
    private void FixedUpdate()
    {
        // Controller Gains
        _AntPIDX.KI = iX;
        _AntPIDX.KD = dX;
        _AntPIDY.KI = iY;
        _AntPIDY.KD = dY;
        _AntPIDZ.KI = iZ;
        _AntPIDZ.KD = dZ;

        // Enable/Unable Gravity
        _rbAnt.useGravity = applyGravity;

        //// TODO ////
        
        // Get current joint angle -> ERROR GIMBAL LOCK
        currentAngle = jointRotation(_jointAnt);
        Debug.Log("currentAngle: " + currentAngle);

        //currentAngleQuaternion = jointRotationQuaternion(_jointAnt);
        currentAngleQuaternion = transform.localRotation;
        Debug.Log("currentAngleQuaternion: " + currentAngleQuaternion);

        // Target Quaternion
        kinematicAngleQuaternion = kinematicArm.transform.localRotation;
        resultAngleQuaternion = ConfigurableJointExtensions.GetTargetRotationLocal(_jointAnt, kinematicAngleQuaternion, initialAngleQuaternion);

        // This last Q is the rotation to be achieved with the antagonistic controller.
        // Instead of setting the Q directly as a target rotation in the configurable joint, we use our Antagonitic controller.

        // DOING: Antagonistic-Quaternion Controller

        //////////////

        // Distance from root to the RB COM
        distance3D = _rbAnt.worldCenterOfMass - transform.position;

        // 1. Gravity force and generated torque
        gravityAcc = Physics.gravity;
        gravityTorqueMagnitudeX = Vector3.Magnitude(_rbAnt.mass * gravityAcc) * Vector3.Distance(transform.parent.position, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 - currentAngle.x) * Mathf.Deg2Rad); // Debug
        gravityTorqueVector = Vector3.Cross(distance3D, _rbAnt.mass * gravityAcc); // Remember: wrt. global coord. system
        gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // Remember: wrt. local coord. system

        // 2. External Forces and generated torque
        externalTorqueMagnitudeX = Vector3.Magnitude(externalForce) * Vector3.Distance(sphereAnt.position, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 - currentAngle.x) * Mathf.Deg2Rad); // Debug - TODO: Angle could be wrong
        externalTorqueVector = Vector3.Cross(distance3D, externalForce); // Remember: wrt. global coord. system
        externalTorqueVectorLocal = transform.InverseTransformDirection(externalTorqueVector); // Remember: wrt. local coord. system

        // Applying forces: Gravity is applied by Unity in the RB. For the external forces, we add them to the RB.
        if (applyExternalForce)
            _rbAnt.AddForce(externalForce, ForceMode.Force);

        //------//

        // Now, we create a controller for each axis. Each will contain an intercept and slope.
        // Here, we define the torques that we expect the controller to counteract.

        if (applyTorqueX)
        {
            // With "-" because Unity uses left-hand rule. I leave the debugs lines with left-hand.
            interceptX = (-gravityTorqueVectorLocal.x - externalTorqueVectorLocal.x) / (maxAngleX - eqAngleX);
            slopeX = (minAngleX - eqAngleX) / (eqAngleX - maxAngleX);

            // Isoline
            pHX = pLX * slopeX + interceptX;
            _AntPIDX.KPH = pHX;
            _AntPIDX.KPL = pLX;

            float angleLowErrorX = minAngleX - currentAngle.x;
            float angleHighErrorX = maxAngleX - currentAngle.x;

            if (printTorqueX)
            {
                Debug.Log("currentAngle.x: " + currentAngle.x);
                Debug.Log("angleLowErrorX: " + (angleLowErrorX));
                Debug.Log("angleHighErrorX: " + (angleHighErrorX));
            }

            torqueAppliedX = _AntPIDX.GetOutput(angleLowErrorX, angleHighErrorX, Time.fixedDeltaTime);
            if (printTorqueX)
            {
                Debug.Log("torqueAppliedX in Ant: " + torqueAppliedX);
                Debug.Log("gravityTorqueVectorLocal.x generated by gravity: " + gravityTorqueVectorLocal.x);
                Debug.Log("externalTorqueVectorLocal.x generated by extF: " + externalTorqueVectorLocal.x);
                Debug.Log("Torque difference: " + (gravityTorqueVectorLocal.x + externalTorqueVectorLocal.x + torqueAppliedX));
                Debug.Log("---------------");
            }
        }

        if (applyTorqueY)
        {
            interceptY = (-gravityTorqueVectorLocal.y - externalTorqueVectorLocal.y) / (maxAngleY - eqAngleY);
            slopeY = (minAngleY - eqAngleY) / (eqAngleY - maxAngleY);

            // Isoline
            pHY = pLY * slopeY + interceptY;
            _AntPIDY.KPH = pHY;
            _AntPIDY.KPL = pLY;

            float angleLowErrorY = minAngleY - currentAngle.y;
            float angleHighErrorY = maxAngleY - currentAngle.y;
            if (printTorqueY)
            {
                Debug.Log("currentAngle.y: " + currentAngle.y);
                Debug.Log("angleLowErrorY: " + angleLowErrorY);
                Debug.Log("angleHighErrorY: " + angleHighErrorY);
            }

            // CAUTION -> Had to switch High and Low errors...
            torqueAppliedY = _AntPIDY.GetOutput(angleHighErrorY, angleLowErrorY, Time.fixedDeltaTime);
            if (printTorqueY)
            {
                Debug.Log("torqueAppliedY in Ant: " + torqueAppliedY);
                Debug.Log("gravityTorqueVectorLocal.y generated by gravity: " + gravityTorqueVectorLocal.y);
                Debug.Log("externalTorqueVectorLocal.y generated by extF: " + externalTorqueVectorLocal.y);
                Debug.Log("Torque difference: " + (gravityTorqueVectorLocal.y + externalTorqueVectorLocal.y + torqueAppliedY));
                Debug.Log("---------------");
            }
        }

        if (applyTorqueZ)
        {
            interceptZ = (-gravityTorqueVectorLocal.z - externalTorqueVectorLocal.z) / (maxAngleZ - eqAngleZ);
            slopeZ = (minAngleZ - eqAngleZ) / (eqAngleZ - maxAngleZ);

            // Isoline
            pHZ = pLZ * slopeZ + interceptZ;
            _AntPIDZ.KPH = pHZ;
            _AntPIDZ.KPL = pLZ;

            float angleLowErrorZ = minAngleZ - currentAngle.z;
            float angleHighErrorZ = maxAngleZ - currentAngle.z;
            if (printTorqueZ)
            {
                Debug.Log("currentAngle.z: " + currentAngle.z);
                Debug.Log("angleLowErrorZ: " + angleLowErrorZ);
                Debug.Log("angleHighErrorZ: " + angleHighErrorZ);
            }

            torqueAppliedZ = _AntPIDZ.GetOutput(angleLowErrorZ, angleHighErrorZ, Time.fixedDeltaTime);
            if (printTorqueZ)
            {
                Debug.Log("torqueAppliedZ in Ant: " + torqueAppliedZ);
                Debug.Log("gravityTorqueVectorLocal.z generated by gravity: " + gravityTorqueVectorLocal.z);
                Debug.Log("externalTorqueVectorLocal.z generated by extF: " + externalTorqueVectorLocal.z);
                Debug.Log("Torque difference: " + (gravityTorqueVectorLocal.z + externalTorqueVectorLocal.z + torqueAppliedZ));
                Debug.Log("---------------");
            }

        }

        // Applying the relative torques
        if (applyTorqueX)
            _rbAnt.AddRelativeTorque(torqueAppliedX * Vector3.right);

        //if (applyTorqueY)
        //    _rbAnt.AddRelativeTorque(torqueAppliedY * Vector3.forward);

        if (applyTorqueZ)
            _rbAnt.AddRelativeTorque(torqueAppliedZ * Vector3.forward);
    }
    */

    public float to180(float v)
    {
        if (v > 270)
        {
            v = v - 360;
        }
        return v;
    }

    public float to90(float v)
    {
        if (v > 90)
        {
            v = 180 - v;
        }
        return v;
    }

    /*
    Vector3 jointRotation(ConfigurableJoint joint)
    {
        // First Debug option - TODO: FIX
        float xAngle = Vector3.Angle(joint.connectedBody.transform.forward, joint.GetComponent<Rigidbody>().transform.forward);
        float yAngle = Vector3.Angle(joint.connectedBody.transform.up, joint.GetComponent<Rigidbody>().transform.up);
        float zAngle = Vector3.SignedAngle(joint.connectedBody.transform.right, joint.GetComponent<Rigidbody>().transform.right, Vector3.right);

        var currentEulerAngles = new Vector3(xAngle, 0f, zAngle);

        return currentEulerAngles;
    }
    */

    /*
    Quaternion jointRotationQuaternion(ConfigurableJoint joint)
    {
        // TODO: This needs to be very improved!

        Quaternion localRotation = Quaternion.Inverse(this.transform.parent.rotation) * this.transform.rotation;
        //Debug.Log("localRotation: " + localRotation);

        //Quaternion jointBasis = Quaternion.LookRotation(joint.secondaryAxis, Vector3.Cross(joint.axis, joint.secondaryAxis));
        //Debug.Log("jointBasis: " + jointBasis.eulerAngles);
        //Quaternion jointBasisInverse = Quaternion.Inverse(jointBasis);
        //var rotation = (jointBasisInverse * Quaternion.Inverse(joint.connectedBody.rotation) * joint.GetComponent<Rigidbody>().transform.rotation * jointBasis).eulerAngles;
        //Debug.Log("rotation: " + rotation.x);

        //return new Vector3(to180(rotation.x), to180(rotation.z), to180(rotation.y));
        //return new Vector3(to180(localRotation.eulerAngles.x), to180(localRotation.eulerAngles.y), to180(localRotation.eulerAngles.z));

        return localRotation;
    }
    */

    #endregion
}

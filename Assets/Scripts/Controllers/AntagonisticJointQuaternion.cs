using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticJointQuaternion : MonoBehaviour
{

    #region Read-only & Static Fields

    private readonly AntagonisticPDControllerQuaternion _antagonisticPDControllerQuaternion = new AntagonisticPDControllerQuaternion(0.05f, 0.0f, 0.0f, 0.01f);

    #endregion

    #region Fields

    private Transform _currentTransform;
    private Rigidbody _objectRigidbody;
    private Quaternion _initialLocalRotation;

    public float Kpl;
    public float Kph;
    public float Ki;
    public float Kd;

    public float angleX, angleY, angleZ;

    #endregion

    #region Instance Properties

    public Quaternion DesiredLocalOrientation { get; set; }

    #endregion

    // ---- DIVISOR

    #region Old Variables

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

    #endregion

    private void Awake()
    {
        this._currentTransform = transform;
        this._initialLocalRotation = transform.localRotation;
        this._objectRigidbody = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        if (DesiredLocalOrientation == null || this._currentTransform == null || this._objectRigidbody == null)
        {
            return;
        }

        // Update gains in real-time
        this._antagonisticPDControllerQuaternion.KPL = this.Kpl;
        this._antagonisticPDControllerQuaternion.KPH = this.Kph;
        this._antagonisticPDControllerQuaternion.KI = this.Ki;
        this._antagonisticPDControllerQuaternion.KD = this.Kd;

        // TODO - START HERE - We need to take to the AntagonisticPDControllerQuaternion class, all the stuff that is here calculated.

        // Get rotation that we need to copy (kinematic)
        kinematicAngleQuaternion = kinematicArm.transform.localRotation;
        DesiredLocalOrientation = ConfigurableJointExtensions.GetTargetRotationLocal(_jointAnt, kinematicAngleQuaternion, _initialLocalRotation);

        // Calculate forces relative to the Rigid Body
        // Distance from root to the RB COM
        distance3D = _objectRigidbody.worldCenterOfMass - transform.position;

        // 1. Gravity force and generated torque
        gravityAcc = Physics.gravity;
        gravityTorqueVector = Vector3.Cross(distance3D, _objectRigidbody.mass * gravityAcc); // Remember: wrt. global coord. system
        gravityTorqueVectorLocal = transform.InverseTransformDirection(gravityTorqueVector); // Remember: wrt. local coord. system

        // The PID controller takes the current orientation of an object, its desired orientation and the current angular velocity
        // and returns the required angular acceleration to rotate towards the desired orientation.
        Vector3 requiredAngularAccelerationX = this._antagonisticPDControllerQuaternion.ComputeRequiredAngularAccelerationX(angleX, 0f, 0f,
                                                                                                     this._currentTransform.localRotation,
                                                                                                     DesiredLocalOrientation,
                                                                                                     this._objectRigidbody.angularVelocity,
                                                                                                     gravityTorqueVectorLocal,
                                                                                                     Time.fixedDeltaTime);

        /*
        Vector3 requiredAngularAccelerationY = this._antagonisticPDControllerQuaternion.ComputeRequiredAngularAccelerationY(0f, angleY, 0f,
                                                                                                     this._currentTransform.localRotation,
                                                                                                     DesiredLocalOrientation,
                                                                                                     this._objectRigidbody.angularVelocity,
                                                                                                     gravityTorqueVectorLocal,
                                                                                                     Time.fixedDeltaTime);

        Vector3 requiredAngularAccelerationZ = this._antagonisticPDControllerQuaternion.ComputeRequiredAngularAccelerationZ(0f, 0f, angleZ,
                                                                                                     this._currentTransform.localRotation,
                                                                                                     DesiredLocalOrientation,
                                                                                                     this._objectRigidbody.angularVelocity,
                                                                                                     gravityTorqueVectorLocal,
                                                                                                     Time.fixedDeltaTime);
        */

        // ! Changed Acceleation by Force
        this._objectRigidbody.AddTorque(requiredAngularAccelerationX, ForceMode.Force);
        
        
        //this._objectRigidbody.AddTorque(requiredAngularAccelerationY, ForceMode.Force);
        //this._objectRigidbody.AddTorque(requiredAngularAccelerationZ, ForceMode.Force);
    }

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

    /*
    public float to180(float v)
    {
        if (v > 180)
        {
            v = v - 360;
        }
        return v;
    }
    */

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
}

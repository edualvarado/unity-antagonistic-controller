using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticArm : MonoBehaviour
{
    [Header("Antagonistic - X")]
    public bool applyTorqueX;
    public float pLX;
    public float pHX;
    public float iX;
    public float dX;
    public float minAngleX;
    public float maxAngleX;
    public float slopeX;
    public float interceptX;
    public float eqAngleX;
    private AntagonisticController _AntPIDX;

    [Header("Antagonistic - Y")]
    public bool applyTorqueY;
    public float pLY;
    public float pHY;
    public float iY;
    public float dY;
    public float minAngleY;
    public float maxAngleY;
    public float slopeY;
    public float interceptY;
    public float eqAngleY;
    private AntagonisticController _AntPIDY;

    [Header("Debug")]
    public Vector3 currentAngle;
    public Rigidbody _rbAnt;
    public ConfigurableJoint _jointAnt;
    public Transform sphereAnt;

    [Header("Experimental")]
    public Vector3 distance3D;
    public bool applyExternalForce;
    public Vector3 externalForce;
    public float externalTorqueMagnitudeX;
    public Vector3 externalTorqueVector;
    public Vector3 gravityAcc;
    public float gravityTorqueMagnitudeX;
    public Vector3 gravityTorqueVector;

    // Start is called before the first frame update
    void Start()
    {
        _AntPIDX = new AntagonisticController(pLX, pHX, iX, dX);
        _AntPIDY = new AntagonisticController(pLY, pHY, iY, dY);
    }

    // Update is called once per frame
    void Update()
    {
        // Draw equilibrium angle
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(eqAngleX, eqAngleY, 0f)) * Vector3.forward * 10f, Color.green);

        // Draw up-axis of body limb
        Debug.DrawRay(sphereAnt.position, transform.up * 10f, Color.blue);

        // To make it simpler, rotate axis to gravity and draw
        //Debug.DrawRay(_rbAnt.worldCenterOfMass, Quaternion.Euler(90f - currentAngle.x, 0f, 0f) * transform.up * 10f, Color.black);
        //Quaternion rotToGrav = Quaternion.FromToRotation(transform.up, -Vector3.up);
        //Debug.DrawRay(_rbAnt.worldCenterOfMass, rotToGrav * transform.up * 10f, Color.black);

        // Draw limits of the antagonistic controller
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(maxAngleX, 0f, 0f)) * Vector3.forward * 10f, Color.red);
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(minAngleX, 0f, 0f)) * Vector3.forward * 10f, Color.red);
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(0f, maxAngleY, 0f)) * Vector3.forward * 10f, Color.red);
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(0f, minAngleY, 0f)) * Vector3.forward * 10f, Color.red);

        // Draw line conecting root with COM
        Debug.DrawLine(sphereAnt.position, _rbAnt.worldCenterOfMass, Color.yellow);

        // Draw external force
        Debug.DrawRay(_rbAnt.worldCenterOfMass, _rbAnt.mass * gravityAcc, Color.yellow);
        Debug.DrawRay(_rbAnt.worldCenterOfMass, externalForce, Color.magenta);

        // Draw torque produced by external
        Debug.DrawRay(sphereAnt.position, gravityTorqueVector, Color.yellow);
        Debug.DrawRay(sphereAnt.position, externalTorqueVector, Color.magenta);

    }

    private void FixedUpdate()
    {
        // Controller Gains
        _AntPIDX.KI = iX;
        _AntPIDX.KD = dX;
        _AntPIDY.KI = iY;
        _AntPIDY.KD = dY;

        // Get current joint angle
        currentAngle = jointRotation(_jointAnt);
        Debug.Log("currentAngle: " + currentAngle);

        // Distance from root to rb
        distance3D = _rbAnt.worldCenterOfMass - sphereAnt.position;

        // 1. Gravity force will be always in the system
        gravityAcc = Physics.gravity;
        gravityTorqueMagnitudeX = Vector3.Magnitude(_rbAnt.mass * gravityAcc) * Vector3.Distance(sphereAnt.position, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 - currentAngle.x) * Mathf.Deg2Rad);
        gravityTorqueVector = Vector3.Cross(distance3D, _rbAnt.mass * gravityAcc); // MAN, I was putting directly the acceleration!

        // 2. External Forces: For this config, create a torque vector based on an external Vector3 Force:
        // Angle is wrong in the magnitude!
        externalTorqueMagnitudeX = Vector3.Magnitude(externalForce) * Vector3.Distance(sphereAnt.position, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 - currentAngle.x) * Mathf.Deg2Rad);
        externalTorqueVector = Vector3.Cross(distance3D, externalForce);

        // Applying forces: Gravity is applied by Unity in the RB. For the external forces, we add them to the Rb.
        if (applyExternalForce)
            _rbAnt.AddForce(externalForce, ForceMode.Force);

        //------//

        // Now, we create a controller for each axis. Each will contain an intercept and slope.
        // Here, we define the torques that we expect the controller to counteract.

        Debug.Log("gravityTorqueVector.x: " + gravityTorqueVector.x);
        Debug.Log("the other: " + _rbAnt.mass * Physics.gravity.y * Vector3.Distance(sphereAnt.position, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 - currentAngle.x) * Mathf.Deg2Rad));

        // Put - because Unity uses left-hand rule. I leave the Debugs lines with left hand.
        interceptX = (- gravityTorqueVector.x - externalTorqueVector.x) / (maxAngleX - eqAngleX);
        slopeX = (minAngleX - eqAngleX) / (eqAngleX - maxAngleX);

        pHX = pLX * slopeX + interceptX;
        _AntPIDX.KPH = pHX;
        _AntPIDX.KPL = pLX;

        float angleLowErrorX = minAngleX - currentAngle.x;
        float angleHighErrorX = maxAngleX - currentAngle.x;
        //Debug.Log("currentAngle.x: " + currentAngle.x);
        //Debug.Log("angleLowErrorX: " + angleLowErrorX);
        //Debug.Log("angleHighErrorX: " + angleHighErrorX);

        float torqueAppliedX = _AntPIDX.GetOutput(angleLowErrorX, angleHighErrorX, Time.fixedDeltaTime);
        //Debug.Log("torqueApplied in Ant: " + torqueApplied);
        //Debug.Log("Angle Gravity Force in Ant: " + (90f + _joint.angle));
        //Debug.Log("Gravity in Ant: " + _rb.mass * Physics.gravity.y * Vector3.Distance(_joint.connectedAnchor, _rb.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad));
        //Debug.Log("---------------");
        //Debug.Log("Difference in Ant: " + (_rbAnt.mass * Physics.gravity.y * Vector3.Distance(_jointAnt.connectedAnchor, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad) - torqueApplied));

        if(applyTorqueX)
            _rbAnt.AddRelativeTorque(torqueAppliedX * Vector3.right);

    }

    public Quaternion getJointRotation(ConfigurableJoint joint)
    {
        //Debug.Log("joint.axis: " + joint.axis);
        //Debug.Log("joint.connectedBody.transform.rotation.eulerAngles: " + joint.connectedBody.transform.rotation.eulerAngles);
        return (Quaternion.FromToRotation(joint.axis, joint.connectedBody.transform.rotation.eulerAngles));
    }

    public float to180(float v)
    {
        if (v > 180)
        {
            v = v - 360;
        }
        return v;
    }
    Vector3 jointRotation(ConfigurableJoint joint)
    {
        Quaternion jointBasis = Quaternion.LookRotation(joint.secondaryAxis, Vector3.Cross(joint.axis, joint.secondaryAxis));
        Quaternion jointBasisInverse = Quaternion.Inverse(jointBasis);
        var rotation = (jointBasisInverse * Quaternion.Inverse(joint.connectedBody.rotation) * joint.GetComponent<Rigidbody>().transform.rotation * jointBasis).eulerAngles;
        return new Vector3(to180(rotation.x), to180(rotation.z), to180(rotation.y));
    }
}

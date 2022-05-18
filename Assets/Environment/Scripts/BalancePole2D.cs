using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BalancePole2D: MonoBehaviour
{
    [Range(0f, 1000f)]
    public float p, i, d;

    public float targetAngle;

    private PDController _PID;

    public Rigidbody _rb;
    public ConfigurableJoint _joint;
    public Transform root;

    public bool applyX;
    public bool applyZ;

    // Start is called before the first frame update
    void Start()
    {
        _PID = new PDController(p, i, d);
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawRay(root.position, Quaternion.Euler(new Vector3(targetAngle, 0f, 0f)) * Vector3.up * 10f, Color.green);

        Debug.DrawRay(root.position, transform.up * 10f, Color.blue);

        //Debug.DrawRay(_rb.worldCenterOfMass, Quaternion.Euler(- _joint.angle, 0f, 0f) * -transform.up * 10f, Color.black);

        Debug.DrawLine(root.position, _rb.worldCenterOfMass, Color.yellow);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        _PID.KP = p;
        _PID.KI = i;
        _PID.KD = d;

        float angleDiffX = 90f - Vector3.Angle(transform.up, root.transform.forward);
        float angleDiffZ = 90f - Vector3.Angle(transform.up, root.transform.right);
        //Debug.Log("angleDiffX " + angleDiffX);
        //Debug.Log("angleDiffZ " + angleDiffZ);
        //Debug.Log("--------------- ");

        float angleErrorX = targetAngle - angleDiffX;
        float angleErrorZ = targetAngle - angleDiffZ;
        //Debug.Log("angleErrorX: " + angleErrorX);
        //Debug.Log("angleErrorZ: " + angleErrorZ);
        //Debug.Log("--------------- ");

        //float angleError = targetAngle - _joint.angle;
        //Debug.Log("_joint.angle: " + _joint.angle);
        //Debug.Log("angleError: " + angleError);
        //Debug.Log("--------------- ");

        float torqueAppliedX = _PID.GetOutput(angleErrorX, _rb.angularVelocity.magnitude, Time.fixedDeltaTime);
        float torqueAppliedZ = _PID.GetOutput(angleErrorZ, _rb.angularVelocity.magnitude, Time.fixedDeltaTime);
        //Debug.Log("torqueAppliedX in PD: " + torqueAppliedX);
        //Debug.Log("torqueAppliedZ in PD: " + torqueAppliedZ);

        if(applyX)
            _rb.AddRelativeTorque(torqueAppliedX * Vector3.right);

        if(applyZ)
            _rb.AddRelativeTorque(torqueAppliedZ * -Vector3.forward);

        //float torqueApplied = _PID.GetOutput(angleError, Time.fixedDeltaTime);
        //Debug.Log("torqueApplied in PD: " + torqueApplied); 
        //Debug.Log("Angle Gravity Force in PD: " + (90f + _joint.angle));
        //Debug.Log("Gravity in PD: " +  _rb.mass * Physics.gravity.y * Vector3.Distance(_joint.connectedAnchor, _rb.worldCenterOfMass) * Mathf.Sin((90 + _joint.angle) * Mathf.Deg2Rad));
        //Debug.Log("---------------");
        //Debug.Log("Difference in PD: " + (_rbPD.mass * Physics.gravity.y * Vector3.Distance(_jointPD.connectedAnchor, _rbPD.worldCenterOfMass) * Mathf.Sin((90f + _jointPD.angle) * Mathf.Deg2Rad) - torqueApplied));

        //_rb.AddRelativeTorque(torqueApplied * Vector3.right);
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BalanceArm : MonoBehaviour
{
    [Range(0f,1000f)]
    public float p, i, d;

    public float targetAngle;

    private PDController _PID;

    public Rigidbody _rbPD;
    public HingeJoint _jointPD;
    public Transform spherePD;

    // Start is called before the first frame update
    void Start()
    {
        _PID = new PDController(p, i, d);
    }

    private void Update()
    {
        Debug.DrawRay(spherePD.position, Quaternion.Euler(new Vector3(targetAngle, 0f, 0f)) * Vector3.forward * 10f, Color.green);

        Debug.DrawRay(spherePD.position, transform.up * 10f, Color.blue);

        Debug.DrawRay(_rbPD.worldCenterOfMass, Quaternion.Euler(90f - _jointPD.angle, 0f, 0f) * transform.up * 10f, Color.black);

        Debug.DrawLine(spherePD.position, _rbPD.worldCenterOfMass, Color.yellow);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        _PID.KP = p;
        _PID.KI = i;
        _PID.KD = d;

        float angleError = targetAngle - _jointPD.angle;
        //Debug.Log("_joint.angle: " + _jointPD.angle);
        //Debug.Log("angleError: " + angleError);
        //Debug.Log("--------------- ");

        float torqueApplied = _PID.GetOutput(angleError, Time.fixedDeltaTime);
        //Debug.Log("torqueApplied in PD: " + torqueApplied); 
        //Debug.Log("Angle Gravity Force in PD: " + (90f + _joint.angle));
        //Debug.Log("Gravity in PD: " +  _rb.mass * Physics.gravity.y * Vector3.Distance(_joint.connectedAnchor, _rb.worldCenterOfMass) * Mathf.Sin((90 + _joint.angle) * Mathf.Deg2Rad));
        //Debug.Log("---------------");
        //Debug.Log("Difference in PD: " + (_rbPD.mass * Physics.gravity.y * Vector3.Distance(_jointPD.connectedAnchor, _rbPD.worldCenterOfMass) * Mathf.Sin((90f + _jointPD.angle) * Mathf.Deg2Rad) - torqueApplied));

        _rbPD.AddRelativeTorque(torqueApplied * Vector3.right);
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BalancePoleAntagonistic : MonoBehaviour
{
    public float pL, pH, i, d;

    public float minAngle;
    public float maxAngle;

    public float slope;
    public float intercept;
    public float eqAngle;

    private AntagonisticController _AntPID;

    public Rigidbody _rb;
    public HingeJoint _joint;
    public Transform root;

    // Start is called before the first frame update
    void Start()
    {
        _AntPID = new AntagonisticController(pL, pH, i, d);
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawRay(root.position, Quaternion.Euler(new Vector3(eqAngle, 0f, 0f)) * Vector3.up * 2f, Color.green);

        Debug.DrawRay(this.transform.position, transform.up * 1.5f, Color.blue);

        Debug.DrawRay(_rb.worldCenterOfMass, Quaternion.Euler(-_joint.angle, 0, 0f) * -_rb.transform.up * 2f, Color.black);

        Debug.DrawRay(root.position, Quaternion.Euler(new Vector3(maxAngle, 0f, 0f)) * Vector3.up * 2f, Color.red);
        Debug.DrawRay(root.position, Quaternion.Euler(new Vector3(minAngle, 0f, 0f)) * Vector3.up * 2f, Color.red);

        Debug.DrawLine(root.position, _rb.worldCenterOfMass, Color.yellow);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        _AntPID.KI = i;
        _AntPID.KD = d;

        intercept = (_rb.mass * Physics.gravity.y * Vector3.Distance(root.position, _rb.worldCenterOfMass) * Mathf.Sin((_joint.angle) * Mathf.Deg2Rad)) / (maxAngle - eqAngle);
        slope = (minAngle - eqAngle) / (eqAngle - maxAngle);

        pH = pL * slope + intercept;
        _AntPID.KPH = pH;
        _AntPID.KPL = pL;

        float angleLowError = minAngle - _joint.angle;
        float angleHighError = maxAngle - _joint.angle;
        //Debug.Log("_joint.angle: " + _joint.angle);
        //Debug.Log("angleLowError: " + angleLowError);
        //Debug.Log("angleHighError: " + angleHighError);

        float torqueApplied = _AntPID.GetOutput(angleLowError, angleHighError, _rb.angularVelocity.magnitude, Time.fixedDeltaTime);
        //Debug.Log("torqueApplied in Ant: " + torqueApplied);
        //Debug.Log("Angle Gravity Force in Ant: " + (_joint.angle));
        //Debug.Log("Gravity in Ant: " + _rb.mass * Physics.gravity.y * Vector3.Distance(_joint.connectedAnchor, _rb.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad));
        //Debug.Log("---------------");
        //Debug.Log("Difference in Ant: " + (_rbAnt.mass * Physics.gravity.y * Vector3.Distance(_jointAnt.connectedAnchor, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad) - torqueApplied));

        _rb.AddRelativeTorque(torqueApplied * Vector3.right);
    }
}

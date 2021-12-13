using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BalanceArmAntagonistic : MonoBehaviour
{
    public float pL, pH, i, d;

    public float minAngle;
    public float maxAngle;

    public float slope;
    public float intercept;
    public float eqAngle;

    private AntagonisticController _AntPID;

    public Rigidbody _rbAnt;
    public HingeJoint _jointAnt;
    public Transform sphereAnt;

    // Start is called before the first frame update
    void Start()
    {
        _AntPID = new AntagonisticController(pL, pH, i, d);
    }

    private void Update()
    {
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(eqAngle, 0f, 0f)) * Vector3.forward * 10f, Color.green);

        Debug.DrawRay(sphereAnt.position, transform.up * 10f, Color.blue);

        Debug.DrawRay(_rbAnt.worldCenterOfMass, Quaternion.Euler(90f - _jointAnt.angle, 0f, 0f) * transform.up * 10f, Color.black);

        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(maxAngle, 0f, 0f)) * Vector3.forward * 10f, Color.red);
        Debug.DrawRay(sphereAnt.position, Quaternion.Euler(new Vector3(minAngle, 0f, 0f)) * Vector3.forward * 10f, Color.red);

        Debug.DrawLine(sphereAnt.position, _rbAnt.worldCenterOfMass, Color.yellow);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        _AntPID.KI = i;
        _AntPID.KD = d;

        intercept = (_rbAnt.mass * Physics.gravity.y * Vector3.Distance(sphereAnt.position, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad)) / (maxAngle - eqAngle);
        slope = (minAngle - eqAngle) / (eqAngle - maxAngle);

        pH = pL * slope + intercept;
        _AntPID.KPH = pH;
        _AntPID.KPL = pL;

        float angleLowError = minAngle - _jointAnt.angle;
        float angleHighError = maxAngle - _jointAnt.angle;
        //Debug.Log("_joint.angle: " + _jointAnt.angle);
        //Debug.Log("angleLowError: " + angleLowError);
        //Debug.Log("angleHighError: " + angleHighError);

        float torqueApplied = _AntPID.GetOutput(angleLowError, angleHighError, Time.fixedDeltaTime);
        //Debug.Log("torqueApplied in Ant: " + torqueApplied);
        //Debug.Log("Angle Gravity Force in Ant: " + (90f + _joint.angle));
        //Debug.Log("Gravity in Ant: " + _rb.mass * Physics.gravity.y * Vector3.Distance(_joint.connectedAnchor, _rb.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad));
        //Debug.Log("---------------");
        //Debug.Log("Difference in Ant: " + (_rbAnt.mass * Physics.gravity.y * Vector3.Distance(_jointAnt.connectedAnchor, _rbAnt.worldCenterOfMass) * Mathf.Sin((90 + _jointAnt.angle) * Mathf.Deg2Rad) - torqueApplied));

        _rbAnt.AddRelativeTorque(torqueApplied * Vector3.right);
    }
}

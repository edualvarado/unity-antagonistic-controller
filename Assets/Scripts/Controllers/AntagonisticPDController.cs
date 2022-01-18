using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticPDController
{

    #region Instance Fields

    public float _kI, _kD, _kPL, _kPH;
    public float _PL, _PH, _P, _I, _D;
    public float _previousError;

    public Vector3 _PVector, _IVector, _DVector;

    #endregion

    #region Instance Properties

    public float KPL { get => _kPL; set => _kPL = value; }
    public float KPH { get => _kPH; set => _kPH = value; }
    public float KI { get => _kI; set => _kI = value; }
    public float KD { get => _kD; set => _kD = value; }

    #endregion

    #region Constructors

    public AntagonisticPDController(float pL, float pH, float i, float d)
    {
        _kPL = pL;
        _kPH = pH;
        _kI = i;
        _kD = d;
    }

    #endregion

    #region Instance Methods

    /// <summary>
    /// Computes the corrective output using antagonistic formulation. 
    /// We have two errors, lower and upper ones. The rest is the same than the usual formulation.
    /// </summary>
    /// <param name="currentLowError"></param>
    /// <param name="currentHighError"></param>
    /// <param name="dt"></param>
    /// <returns></returns>
    public float GetOutput(float currentLowError, float currentHighError, float delta, float dt)
    {
        _PL = currentLowError;
        _PH = currentHighError;
        
        _P = currentLowError;
        _I += _P * dt;
        //_D = (_P - _previousError) / dt;
        _D = (delta) / dt;

        _previousError = currentLowError;

        float output = _PL * _kPL + _PH * _kPH + _I * _kI + _D * _kD;

        return output;
    }

    /// <summary>
    /// TEST - Computes the corrective output using PD formulation. 
    /// We have two errors, lower and upper ones. The rest is the same than the usual formulation.
    /// </summary>
    /// <param name="currentLowError"></param>
    /// <param name="currentHighError"></param>
    /// <param name="dt"></param>
    /// <returns></returns>
    public float GetOutputPD(float error, float delta, float dt)
    {
        _P = error;
        _I += _P * dt;
        //_D = (delta) / dt;
        _D = delta; // Directly the angular velocity

        //Debug.Log("_P (error): " + _P);
        //Debug.Log("_D: " + _D);
        //Debug.Log("_kPL " + _kPL);
        //Debug.Log("_kD " + _kD);

        float output = _P * _kPL + _I * _kI + _D * _kD;

        return output;
    }

    public Vector3 GetOutputImprovedPD(float error, Vector3 axis, Vector3 delta, Rigidbody _rb, Transform _t, float dt)
    {

        // 1. Vector space from the beginning

        _PVector = (error * Mathf.Deg2Rad) * axis;
        _IVector += _PVector * dt;
        _DVector = delta;

        Vector3 output = _kPL * _PVector - _kD * _rb.angularVelocity;

        // Convert rotation of inertia tensor to global
        Quaternion rotInertia2World = _rb.inertiaTensorRotation * _t.rotation;

        output = Quaternion.Inverse(rotInertia2World) * output;
        output.Scale(_rb.inertiaTensor);
        output = rotInertia2World * output;

        // 2. All scalar, return output * axis

        _P = (error * Mathf.Deg2Rad);
        _I += _P * dt;
        _D = delta.magnitude;

        float outputScalar = _kPL * _P - _kD * _D;
        Vector3 output2 = outputScalar * axis;

        // Convert rotation of inertia tensor to global
        Quaternion rotInertia2World2 = _rb.inertiaTensorRotation * _t.rotation;

        // ****
        output2 = Quaternion.Inverse(rotInertia2World2) * output2;
        output2.Scale(_rb.inertiaTensor);
        output2 = rotInertia2World2 * output2;
        // ****

        Debug.Log("output: " + output + " - output2: " + output2); // Both are the same

        //return output;
        return output2;
    }

    #endregion
}

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
        // Working version of Antagonistic Controller

        _PL = currentLowError;
        _PH = currentHighError;
        
        _P = currentLowError;
        _I += _P * dt;
        _D = (_P - _previousError) / dt; // or _D = delta

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
        // Normal PD Controlling (float) - Must multiply after by axis

        _P = error;
        _I += _P * dt;
        _D = delta;

        float output = _P * _kPL + _I * _kI + _D * _kD;

        return output;
    }

    public Vector3 GetOutputImprovedPD(float error, Vector3 axis, Vector3 delta, float dt)
    {

        // Normal PD Controlling (Vector3) + Euler Integration

        // Euler Integration for Backward PD
        //float g = 1 / (1 + _kD * dt + _kPL * dt * dt);
        //float ksg = _kPL * g;
        //float kdg = (_kD + _kPL * dt) * g;

        // 1. Vector space from the beginning
        // -----------------------------------

        _PVector = (error * Mathf.Deg2Rad) * axis;
        _IVector += _PVector * dt;
        _DVector = delta;

        Vector3 result = _kPL * _PVector + _kD * _DVector;

        // 2. All scalar, return output * axis
        // -----------------------------------

        _P = (error * Mathf.Deg2Rad);
        _I += _P * dt;
        //_D = delta.magnitude;
        _D = (_P - _previousError) / dt; // or _D = delta

        _previousError = error * Mathf.Deg2Rad;

        float outputScalar = _kPL * _P + _kD * _D;
        //float outputScalar = ksg * _P + kdg * _D;

        Vector3 output = outputScalar * axis;

        // -----------------------------------

        Debug.Log("result: " + result + " - output: " + output); // Both are the same

        //return result;
        return output;
    }

    public Vector3 GetOutputAntagonisticPD(float currentLowError, float currentHighError, Vector3 axis, Vector3 delta, float dt)
    {

        // 1. All scalar, return output * axis
        // -----------------------------------

        _PL = currentLowError;
        _PH = currentHighError;
        
        _I += _P * dt;
        _D = delta.magnitude;

        float outputScalar = _PL * _kPL + _PH * _kPH + _I * _kI + _D * _kD;

        Vector3 output = outputScalar * axis;

        // -----------------------------------

        return output;
    }

    #endregion
}

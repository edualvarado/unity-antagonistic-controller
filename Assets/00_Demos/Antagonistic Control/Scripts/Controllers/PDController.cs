/****************************************************
 * File: PDController.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

/* Status: STABLE */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PDController
{
    #region Instance Fields

    public float _kI, _kD, _kP;
    public float _P, _I, _D;
    public float _previousError;

    public Vector3 _PVector, _IVector, _DVector;

    #endregion

    #region Instance Properties

    public float KP { get => _kP; set => _kP = value; }
    public float KI { get => _kI; set => _kI = value; }
    public float KD { get => _kD; set => _kD = value; }

    #endregion

    #region Constructors

    public PDController(float p, float i, float d)
    {
        _kP = p;
        _kI = i;
        _kD = d;
    }

    #endregion

    #region Instance Methods

    /// <summary>
    /// Estimate output given error using PD Controller.
    /// </summary>
    /// <param name="currentError"></param>
    /// <param name="delta"></param>
    /// <param name="dt"></param>
    /// <returns></returns>
    public float GetOutput(float currentError, float delta, float dt)
    {
        _P = currentError;
        _I += _P * dt;
        _D = delta;

        //_D = (_P - _previousError) / dt; // or _D = delta
        //_previousError = currentError;

        return _P * _kP + _I * _kI + _D * _kD;
    }

    /// <summary>
    /// Estimate output given error in Axis-Angle representation using Backward PD Controller (w/ Euler's Integration).
    /// </summary>
    /// <param name="error"></param>
    /// <param name="axis"></param>
    /// <param name="delta"></param>
    /// <param name="dt"></param>
    /// <returns></returns>
    public Vector3 GetOutputAxisAngle(float error, Vector3 axis, Vector3 delta, float dt)
    {
        // Euler Integration for Backward PD
        //float g = 1 / (1 + _kD * dt + _kPL * dt * dt);
        //float ksg = _kPL * g;
        //float kdg = (_kD + _kPL * dt) * g;

        // 1. In Vector3 space
        // -----------------------------------

        _PVector = (error * Mathf.Deg2Rad) * axis;
        _IVector += _PVector * dt;
        _DVector = delta;

        Vector3 output1 = _kP * _PVector + _kD * _DVector;

        // 2. In scalar space
        // -----------------------------------

        _P = (error * Mathf.Deg2Rad);
        _I += _P * dt;
        _D = delta.magnitude;

        //_D = (_P - _previousError) / dt; // or _D = delta.magnitude
        //_previousError = error * Mathf.Deg2Rad;

        float outputScalar = _kP * _P + _kD * _D;
        //float outputScalar = ksg * _P + kdg * _D; // Output w/ Euler Integration

        Vector3 output2 = outputScalar * axis;

        // -----------------------------------

        //Debug.Log("[INFO] output1: " + output1 + " | output2: " + output2); // Both are the same

        //return output1;
        return output2;
    }

    #endregion
}

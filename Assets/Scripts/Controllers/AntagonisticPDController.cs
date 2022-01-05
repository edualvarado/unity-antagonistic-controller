using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticPDController
{

    #region Instance Fields

    public float _kI, _kD, _kPL, _kPH;
    public float _PL, _PH, _P, _I, _D;
    public float _previousError;

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
    public float GetOutput(float currentLowError, float currentHighError, float dt)
    {
        _PL = currentLowError;
        _PH = currentHighError;
        
        _P = currentLowError;
        _I += _P * dt;
        _D = (_P - _previousError) / dt;

        _previousError = currentLowError;

        float output = _PL * _kPL + _PH * _kPH + _I * _kI + _D * _kD;

        return output;
    }

    #endregion
}

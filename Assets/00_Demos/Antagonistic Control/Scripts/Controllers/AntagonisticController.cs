/****************************************************
 * File: AntagonisticController.cs
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

public class AntagonisticController
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

    public AntagonisticController(float pL, float pH, float i, float d)
    {
        _kPL = pL;
        _kPH = pH;
        _kI = i;
        _kD = d;
    }

    #endregion

    #region Instance Methods

    /// <summary>
    /// Estimate output given low-/upper-error intervals using Antagonistic Controller.
    /// </summary>
    /// <param name="currentLowError"></param>
    /// <param name="currentHighError"></param>
    /// <param name="delta"></param>
    /// <param name="dt"></param>
    /// <returns></returns>
    public float GetOutput(float currentLowError, float currentHighError, float delta, float dt)
    {
        _PL = currentLowError;
        _PH = currentHighError;
        
        _P = currentLowError;
        _I += _P * dt;
        _D = delta;

        //_D = (_P - _previousError) / dt; // or _D = delta
        //_previousError = currentLowError;

        return _PL * _kPL + _PH * _kPH + _I * _kI + _D * _kD;
    }

    #endregion
}

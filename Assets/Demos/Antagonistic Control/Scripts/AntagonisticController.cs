using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntagonisticController : MonoBehaviour
{

    public float _kI, _kD, _kPL, _kPH;
    public float _PL, _PH, _P, _I, _D;
    public float _previousError;

    public float KPL { get => _kPL; set => _kPL = value; }
    public float KPH { get => _kPH; set => _kPH = value; }
    public float KI { get => _kI; set => _kI = value; }
    public float KD { get => _kD; set => _kD = value; }

    public AntagonisticController(float pL, float pH, float i, float d)
    {
        _kPL = pL;
        _kPH = pH;
        _kI = i;
        _kD = d;
    } 

    public float GetOutput(float currentLowError, float currentHighError, float dt)
    {
        _PL = currentLowError;
        _PH = currentHighError;
        
        _P = currentLowError;
        _I += _P * dt;
        _D = (_P - _previousError) / dt;

        _previousError = currentLowError;

        return _PL * _kPL + _PH * _kPH + _I * _kI + _D * _kD;
    }

}

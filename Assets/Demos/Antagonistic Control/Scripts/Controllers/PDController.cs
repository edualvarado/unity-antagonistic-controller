using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PDController
{
    public float _kI, _kD, _kP;
    public float _P, _I, _D;
    public float _previousError;

    public float KP { get => _kP; set => _kP = value; }
    public float KI { get => _kI; set => _kI = value; }
    public float KD { get => _kD; set => _kD = value; }

    public PDController(float p, float i, float d)
    {
        _kP = p;
        _kI = i;
        _kD = d;
    } 

    public float GetOutput(float currentError, float dt)
    {
        _P = currentError;
        _I += _P * dt;
        _D = (_P - _previousError) / dt;

        _previousError = currentError;

        return _P * _kP + _I * _kI + _D * _kD;
    }
}

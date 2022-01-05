using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class AntagonisticPDControllerQuaternion
{
    #region Read-only & Static Fields

    // Set of Antagonistic Controllers for each joint. Each will contain 4 controllers, one for each quaternion component.
    private readonly AntagonisticPDController[] _internalAntagonisticController;

    #endregion

    #region Constructors

    public AntagonisticPDControllerQuaternion(float pl, float ph, float ki, float kd)
    {
        if (pl < 0.0f)
            throw new ArgumentOutOfRangeException("pl", "pl must be a non-negative number.");

        if (ph < 0.0f)
            throw new ArgumentOutOfRangeException("ph", "ph must be a non-negative number.");

        if (ki < 0.0f)
            throw new ArgumentOutOfRangeException("ki", "ki must be a non-negative number.");

        if (kd < 0.0f)
            throw new ArgumentOutOfRangeException("kd", "kd must be a non-negative number.");

        this._internalAntagonisticController = new[]
        {
            new AntagonisticPDController(pl, ph, ki, kd),
            new AntagonisticPDController(pl, ph, ki, kd),
            new AntagonisticPDController(pl, ph, ki, kd),
            new AntagonisticPDController(pl, ph, ki, kd)
        };
    }

    #endregion

    #region Instance Properties

    public float KPL
    {
        get
        {
            return this._internalAntagonisticController[0].KPL;
        }
        set
        {
            this._internalAntagonisticController[0].KPL = value;
            this._internalAntagonisticController[1].KPL = value;
            this._internalAntagonisticController[2].KPL = value;
            this._internalAntagonisticController[3].KPL = value;
        }
    }

    public float KPH
    {
        get
        {
            return this._internalAntagonisticController[0].KPH;
        }
        set
        {
            this._internalAntagonisticController[0].KPH = value;
            this._internalAntagonisticController[1].KPH = value;
            this._internalAntagonisticController[2].KPH = value;
            this._internalAntagonisticController[3].KPH = value;
        }
    }

    public float KD
    {
        get
        {
            return this._internalAntagonisticController[0].KD;
        }
        set
        {
            this._internalAntagonisticController[0].KD = value;
            this._internalAntagonisticController[1].KD = value;
            this._internalAntagonisticController[2].KD = value;
            this._internalAntagonisticController[3].KD = value;
        }
    }

    public float KI
    {
        get
        {
            return this._internalAntagonisticController[0].KI;
        }
        set
        {
            this._internalAntagonisticController[0].KI = value;
            this._internalAntagonisticController[1].KI = value;
            this._internalAntagonisticController[2].KI = value;
            this._internalAntagonisticController[3].KI = value;
        }
    }

    #endregion

    #region Class Methods

    /// <summary>
    /// Multiplies a Matrix by a quaternion treating the quaternion as a Vector4.
    /// </summary>
    /// <param name="matrix">The matrix.</param>
    /// <param name="quaternion">The quaternion.</param>
    /// <returns>The resulting quaternion.</returns>
    public static Quaternion MultiplyAsVector(Matrix4x4 matrix, Quaternion quaternion)
    {
        var vector = new Vector4(quaternion.w, quaternion.x, quaternion.y, quaternion.z);

        Vector4 result = matrix * vector;

        return new Quaternion(result.y, result.z, result.w, result.x);
    }

    /// <summary>
    /// Transforms the specified Euler angle vector to a quaternion containing these Euler angles.
    /// </summary>
    /// <param name="eulerAngles">The Euler angles vector.</param>
    /// <returns>
    ///     A quaternion containing Euler angles.
    /// </returns>
    public static Quaternion ToEulerAngleQuaternion(Vector3 eulerAngles)
    {
        return new Quaternion(eulerAngles.x, eulerAngles.y, eulerAngles.z, 0);
    }

    #endregion

    #region Instance Methods

    /// <summary>
    ///     Computes the angular acceleration required to rotate from the current orientation to
    ///     a desired orientation based on the specified current angular velocity for the current frame.
    /// </summary>
    /// <param name="currentOrientation">The current orientation.</param>
    /// <param name="desiredOrientation">The desired orientation.</param>
    /// <param name="currentAngularVelocity">The current angular velocity.</param>
    /// <param name="deltaTime">The frame delta time.</param>
    /// <returns>
    ///     The angular acceleration required to rotate from the current orientation to the desired orientation.
    /// </returns>
    public Vector3 ComputeRequiredAngularAcceleration(Quaternion currentOrientation, Quaternion desiredOrientation, Vector3 currentAngularVelocity, float deltaTime)
    {
        /*
        Quaternion requiredRotation = QuaternionOpExtensions.RequiredRotation(currentOrientation, desiredOrientation);

        Debug.Log("REQUIRED ROTATION: " + requiredRotation);

        Quaternion error = Quaternion.identity.Subtract(requiredRotation);
        Quaternion angularVelocity = ToEulerAngleQuaternion(currentAngularVelocity);
        Quaternion delta = angularVelocity * requiredRotation;

        var orthogonalizeMatrix = new Matrix4x4()
        {
            m00 =
                                          -requiredRotation.x * -requiredRotation.x + -requiredRotation.y * -requiredRotation.y +
                                          -requiredRotation.z * -requiredRotation.z,
            m01 =
                                          -requiredRotation.x * requiredRotation.w + -requiredRotation.y * -requiredRotation.z +
                                          -requiredRotation.z * requiredRotation.y,
            m02 =
                                          -requiredRotation.x * requiredRotation.z + -requiredRotation.y * requiredRotation.w +
                                          -requiredRotation.z * -requiredRotation.x,
            m03 =
                                          -requiredRotation.x * -requiredRotation.y + -requiredRotation.y * requiredRotation.x +
                                          -requiredRotation.z * requiredRotation.w,
            m10 =
                                          requiredRotation.w * -requiredRotation.x + -requiredRotation.z * -requiredRotation.y +
                                          requiredRotation.y * -requiredRotation.z,
            m11 =
                                          requiredRotation.w * requiredRotation.w + -requiredRotation.z * -requiredRotation.z +
                                          requiredRotation.y * requiredRotation.y,
            m12 =
                                          requiredRotation.w * requiredRotation.z + -requiredRotation.z * requiredRotation.w +
                                          requiredRotation.y * -requiredRotation.x,
            m13 =
                                          requiredRotation.w * -requiredRotation.y + -requiredRotation.z * requiredRotation.x +
                                          requiredRotation.y * requiredRotation.w,
            m20 =
                                          requiredRotation.z * -requiredRotation.x + requiredRotation.w * -requiredRotation.y +
                                          -requiredRotation.x * -requiredRotation.z,
            m21 =
                                          requiredRotation.z * requiredRotation.w + requiredRotation.w * -requiredRotation.z +
                                          -requiredRotation.x * requiredRotation.y,
            m22 =
                                          requiredRotation.z * requiredRotation.z + requiredRotation.w * requiredRotation.w +
                                          -requiredRotation.x * -requiredRotation.x,
            m23 =
                                          requiredRotation.z * -requiredRotation.y + requiredRotation.w * requiredRotation.x +
                                          -requiredRotation.x * requiredRotation.w,
            m30 =
                                          -requiredRotation.y * -requiredRotation.x + requiredRotation.x * -requiredRotation.y +
                                          requiredRotation.w * -requiredRotation.z,
            m31 =
                                          -requiredRotation.y * requiredRotation.w + requiredRotation.x * -requiredRotation.z +
                                          requiredRotation.w * requiredRotation.y,
            m32 =
                                          -requiredRotation.y * requiredRotation.z + requiredRotation.x * requiredRotation.w +
                                          requiredRotation.w * -requiredRotation.x,
            m33 =
                                          -requiredRotation.y * -requiredRotation.y + requiredRotation.x * requiredRotation.x +
                                          requiredRotation.w * requiredRotation.w,
        };

        Quaternion neededAngularVelocity = ComputeOutput(error, delta, deltaTime);

        neededAngularVelocity = MultiplyAsVector(orthogonalizeMatrix, neededAngularVelocity);

        Quaternion doubleNegative = neededAngularVelocity.Multiply(-2.0f);
        Quaternion result = doubleNegative * Quaternion.Inverse(requiredRotation);

        return new Vector3(result.x, result.y, result.z);
        */

        return Vector3.zero;
    }

    /*
    private Quaternion ComputeOutput(Quaternion error, Quaternion delta, float deltaTime)
    {
        var output = new Quaternion
        {
            x = this._internalController[0].ComputeOutput(error.x, delta.x, deltaTime),
            y = this._internalController[1].ComputeOutput(error.y, delta.y, deltaTime),
            z = this._internalController[2].ComputeOutput(error.z, delta.z, deltaTime),
            w = this._internalController[3].ComputeOutput(error.w, delta.w, deltaTime)
        };

        return output;
    }
    */

    #endregion
}

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
    public Vector3 ComputeRequiredAngularAccelerationX(float angleX, float angleY, float angleZ, Quaternion currentOrientation, Quaternion desiredOrientation, Vector3 currentAngularVelocity, Vector3 extForces, float deltaTime, bool printX)
    {
        // 1. Limit angles for the antagonistic controller.
        // If I put here other Euler angle, it does not work!
        Vector3 minAngleEulerX = new Vector3(0f, 0f, 0f);
        Vector3 maxAngleEulerX = new Vector3(180f, 0f, 0f);
        Quaternion minAngleQuaternionX = Quaternion.Euler(minAngleEulerX);
        Quaternion maxAngleQuaternionX = Quaternion.Euler(maxAngleEulerX);

        if(printX)
        {
            Debug.Log("minAngleQuaternionX: " + minAngleQuaternionX);
            Debug.Log("maxAngleQuaternionX: " + maxAngleQuaternionX);
        }

        // 2. Retrieve Target Quaternion, which is already calculated by the upper class.
        //Quaternion requiredRotationX = desiredOrientation;
        //Quaternion requiredRotationX = new Quaternion(desiredOrientation.x, 0f, 0f, desiredOrientation.w);
        Quaternion requiredRotationX = Quaternion.Euler(new Vector3(angleX, 0f, 0f));
        
        if(printX)
        {
            Debug.Log("requiredRotationX: " + requiredRotationX);
        }

        // 3. Isoline and gains.
        float interceptX = (0f) / (minAngleQuaternionX.x - requiredRotationX.x);
        float slopeX = (minAngleQuaternionX.x - requiredRotationX.x) / (requiredRotationX.x - maxAngleQuaternionX.x);
        KPH = KPL * slopeX + interceptX;
        //float interceptX = (0f) / (minAngleEuler.x - angleX);
        //float slopeX = (minAngleEuler.x - angleX) / (angleX - maxAngleEuler.x);
        //KPH = KPL * slopeX + interceptX;

        if (printX)
        {
            Debug.Log("interceptX: " + interceptX);
            Debug.Log("slopeX: " + slopeX);
            Debug.Log("KPL: " + KPL + " - KPH: " + KPH);
        }

        // 4. Calculate low and upper errors.
        //Quaternion lowError = QuaternionOpExtensions.SubtractTwo(minAngleQuaternion, currentOrientation);
        //Quaternion highError = QuaternionOpExtensions.SubtractTwo(maxAngleQuaternion, currentOrientation);
        Quaternion lowError = QuaternionOpExtensions.SubtractTwo(minAngleQuaternionX, new Quaternion(currentOrientation.x, 0f, 0f, currentOrientation.w));
        Quaternion highError = QuaternionOpExtensions.SubtractTwo(maxAngleQuaternionX, new Quaternion(currentOrientation.x, 0f, 0f, currentOrientation.w));

        if (printX)
        {
            Debug.Log("lowError: " + minAngleQuaternionX + " - " + new Quaternion(currentOrientation.x, 0f, 0f, currentOrientation.w) + " = " + lowError);
            Debug.Log("highError: " + maxAngleQuaternionX + " - " + new Quaternion(currentOrientation.x, 0f, 0f, currentOrientation.w) + " = " + highError);
        }

        // 5. Angular Velocity.
        Quaternion angularVelocity = ToEulerAngleQuaternion(currentAngularVelocity);
        Quaternion delta = angularVelocity * requiredRotationX;

        var orthogonalizeMatrix = new Matrix4x4()
        {
            m00 =
                                          -requiredRotationX.x * -requiredRotationX.x + -requiredRotationX.y * -requiredRotationX.y +
                                          -requiredRotationX.z * -requiredRotationX.z,
            m01 =
                                          -requiredRotationX.x * requiredRotationX.w + -requiredRotationX.y * -requiredRotationX.z +
                                          -requiredRotationX.z * requiredRotationX.y,
            m02 =
                                          -requiredRotationX.x * requiredRotationX.z + -requiredRotationX.y * requiredRotationX.w +
                                          -requiredRotationX.z * -requiredRotationX.x,
            m03 =
                                          -requiredRotationX.x * -requiredRotationX.y + -requiredRotationX.y * requiredRotationX.x +
                                          -requiredRotationX.z * requiredRotationX.w,
            m10 =
                                          requiredRotationX.w * -requiredRotationX.x + -requiredRotationX.z * -requiredRotationX.y +
                                          requiredRotationX.y * -requiredRotationX.z,
            m11 =
                                          requiredRotationX.w * requiredRotationX.w + -requiredRotationX.z * -requiredRotationX.z +
                                          requiredRotationX.y * requiredRotationX.y,
            m12 =
                                          requiredRotationX.w * requiredRotationX.z + -requiredRotationX.z * requiredRotationX.w +
                                          requiredRotationX.y * -requiredRotationX.x,
            m13 =
                                          requiredRotationX.w * -requiredRotationX.y + -requiredRotationX.z * requiredRotationX.x +
                                          requiredRotationX.y * requiredRotationX.w,
            m20 =
                                          requiredRotationX.z * -requiredRotationX.x + requiredRotationX.w * -requiredRotationX.y +
                                          -requiredRotationX.x * -requiredRotationX.z,
            m21 =
                                          requiredRotationX.z * requiredRotationX.w + requiredRotationX.w * -requiredRotationX.z +
                                          -requiredRotationX.x * requiredRotationX.y,
            m22 =
                                          requiredRotationX.z * requiredRotationX.z + requiredRotationX.w * requiredRotationX.w +
                                          -requiredRotationX.x * -requiredRotationX.x,
            m23 =
                                          requiredRotationX.z * -requiredRotationX.y + requiredRotationX.w * requiredRotationX.x +
                                          -requiredRotationX.x * requiredRotationX.w,
            m30 =
                                          -requiredRotationX.y * -requiredRotationX.x + requiredRotationX.x * -requiredRotationX.y +
                                          requiredRotationX.w * -requiredRotationX.z,
            m31 =
                                          -requiredRotationX.y * requiredRotationX.w + requiredRotationX.x * -requiredRotationX.z +
                                          requiredRotationX.w * requiredRotationX.y,
            m32 =
                                          -requiredRotationX.y * requiredRotationX.z + requiredRotationX.x * requiredRotationX.w +
                                          requiredRotationX.w * -requiredRotationX.x,
            m33 =
                                          -requiredRotationX.y * -requiredRotationX.y + requiredRotationX.x * requiredRotationX.x +
                                          requiredRotationX.w * requiredRotationX.w,
        };

        // From here, review

        Quaternion neededAngularVelocity = GetOutput(lowError, highError, delta, deltaTime);
        neededAngularVelocity = MultiplyAsVector(orthogonalizeMatrix, neededAngularVelocity);
        Quaternion doubleNegative = neededAngularVelocity.Multiply(-2.0f);
        Quaternion resultX = doubleNegative * Quaternion.Inverse(requiredRotationX);

        if(printX)
        {
            Debug.Log("resultX: " + resultX);
            Debug.Log("---------------------------------");
        }

        return new Vector3(resultX.x, resultX.y, resultX.z);


        /*
        Quaternion requiredRotation = QuaternionOpExtensions.RequiredRotation(currentOrientation, desiredOrientation);

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

        //return Vector3.zero;
    }

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
    public Vector3 ComputeRequiredAngularAccelerationY(float angleX, float angleY, float angleZ, Quaternion currentOrientation, Quaternion desiredOrientation, Vector3 currentAngularVelocity, Vector3 extForces, float deltaTime, bool printY)
    {
        // 1. Limit angles for the antagonistic controller.
        // If I put here other Euler angle, it does not work!
        Vector3 minAngleEulerY = new Vector3(0f, -20f, 0f);
        Vector3 maxAngleEulerY = new Vector3(0f, 20f, 0f);
        Quaternion minAngleQuaternionY = Quaternion.Euler(minAngleEulerY);
        Quaternion maxAngleQuaternionY = Quaternion.Euler(maxAngleEulerY);

        if (printY)
        {
            Debug.Log("minAngleQuaternionY: " + minAngleQuaternionY);
            Debug.Log("maxAngleQuaternionY: " + maxAngleQuaternionY);
        }

        // 2. Retrieve Target Quaternion, which is already calculated by the upper class.
        //Quaternion requiredRotationY = desiredOrientation;
        //Quaternion requiredRotationY = new Quaternion(0f, desiredOrientation.y, 0f, desiredOrientation.w);
        Quaternion requiredRotationY = Quaternion.Euler(new Vector3(0f, angleY, 0f));

        if (printY)
        {
            Debug.Log("requiredRotationY: " + requiredRotationY);
        }

        // 3. Isoline and gains.
        float interceptY = (0f) / (minAngleQuaternionY.y - requiredRotationY.y);
        float slopeY = (minAngleQuaternionY.y - requiredRotationY.y) / (requiredRotationY.y - requiredRotationY.y);
        KPH = KPL * slopeY + interceptY;
        //float interceptY = (0f) / (minAngleEuler.y - angleY);
        //float slopeY = (minAngleEuler.y - angleY) / (angleY - maxAngleEuler.y);
        //KPH = KPL * slopeY + interceptY;

        // 4. Calculate low and upper errors.
        //Quaternion lowError = QuaternionOpExtensions.SubtractTwo(minAngleQuaternion, currentOrientation);
        //Quaternion highError = QuaternionOpExtensions.SubtractTwo(maxAngleQuaternion, currentOrientation);
        Quaternion lowError = QuaternionOpExtensions.SubtractTwo(minAngleQuaternionY, new Quaternion(0f, currentOrientation.y, 0f, currentOrientation.w));
        Quaternion highError = QuaternionOpExtensions.SubtractTwo(maxAngleQuaternionY, new Quaternion(0f, currentOrientation.y, 0f, currentOrientation.w));

        if (printY)
        {
            Debug.Log("lowError: " + minAngleQuaternionY + " - " + new Quaternion(0f, currentOrientation.y, 0f, currentOrientation.w) + " = " + lowError);
            Debug.Log("highError: " + maxAngleQuaternionY + " - " + new Quaternion(0f, currentOrientation.y, 0f, currentOrientation.w) + " = " + highError);
        }

        // 5. Angular Velocity.
        Quaternion angularVelocity = ToEulerAngleQuaternion(currentAngularVelocity);
        Quaternion delta = angularVelocity * requiredRotationY;

        var orthogonalizeMatrix = new Matrix4x4()
        {
            m00 =
                                          -requiredRotationY.x * -requiredRotationY.x + -requiredRotationY.y * -requiredRotationY.y +
                                          -requiredRotationY.z * -requiredRotationY.z,
            m01 =
                                          -requiredRotationY.x * requiredRotationY.w + -requiredRotationY.y * -requiredRotationY.z +
                                          -requiredRotationY.z * requiredRotationY.y,
            m02 =
                                          -requiredRotationY.x * requiredRotationY.z + -requiredRotationY.y * requiredRotationY.w +
                                          -requiredRotationY.z * -requiredRotationY.x,
            m03 =
                                          -requiredRotationY.x * -requiredRotationY.y + -requiredRotationY.y * requiredRotationY.x +
                                          -requiredRotationY.z * requiredRotationY.w,
            m10 =
                                          requiredRotationY.w * -requiredRotationY.x + -requiredRotationY.z * -requiredRotationY.y +
                                          requiredRotationY.y * -requiredRotationY.z,
            m11 =
                                          requiredRotationY.w * requiredRotationY.w + -requiredRotationY.z * -requiredRotationY.z +
                                          requiredRotationY.y * requiredRotationY.y,
            m12 =
                                          requiredRotationY.w * requiredRotationY.z + -requiredRotationY.z * requiredRotationY.w +
                                          requiredRotationY.y * -requiredRotationY.x,
            m13 =
                                          requiredRotationY.w * -requiredRotationY.y + -requiredRotationY.z * requiredRotationY.x +
                                          requiredRotationY.y * requiredRotationY.w,
            m20 =
                                          requiredRotationY.z * -requiredRotationY.x + requiredRotationY.w * -requiredRotationY.y +
                                          -requiredRotationY.x * -requiredRotationY.z,
            m21 =
                                          requiredRotationY.z * requiredRotationY.w + requiredRotationY.w * -requiredRotationY.z +
                                          -requiredRotationY.x * requiredRotationY.y,
            m22 =
                                          requiredRotationY.z * requiredRotationY.z + requiredRotationY.w * requiredRotationY.w +
                                          -requiredRotationY.x * -requiredRotationY.x,
            m23 =
                                          requiredRotationY.z * -requiredRotationY.y + requiredRotationY.w * requiredRotationY.x +
                                          -requiredRotationY.x * requiredRotationY.w,
            m30 =
                                          -requiredRotationY.y * -requiredRotationY.x + requiredRotationY.x * -requiredRotationY.y +
                                          requiredRotationY.w * -requiredRotationY.z,
            m31 =
                                          -requiredRotationY.y * requiredRotationY.w + requiredRotationY.x * -requiredRotationY.z +
                                          requiredRotationY.w * requiredRotationY.y,
            m32 =
                                          -requiredRotationY.y * requiredRotationY.z + requiredRotationY.x * requiredRotationY.w +
                                          requiredRotationY.w * -requiredRotationY.x,
            m33 =
                                          -requiredRotationY.y * -requiredRotationY.y + requiredRotationY.x * requiredRotationY.x +
                                          requiredRotationY.w * requiredRotationY.w,
        };

        // From here, review

        Quaternion neededAngularVelocity = GetOutput(lowError, highError, delta, deltaTime);
        neededAngularVelocity = MultiplyAsVector(orthogonalizeMatrix, neededAngularVelocity);
        Quaternion doubleNegative = neededAngularVelocity.Multiply(-2.0f);
        Quaternion resultY = doubleNegative * Quaternion.Inverse(requiredRotationY);

        if (printY)
        {
            Debug.Log("resultY: " + resultY);
            Debug.Log("---------------------------------");
        }

        return new Vector3(resultY.x, resultY.y, resultY.z);
    }

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
    public Vector3 ComputeRequiredAngularAccelerationZ(float angleX, float angleY, float angleZ, Quaternion currentOrientation, Quaternion desiredOrientation, Vector3 currentAngularVelocity, Vector3 extForces, float deltaTime, bool printZ)
    {
        // 1. Limit angles for the antagonistic controller.
        // If I put here other Euler angle, it does not work!
        Vector3 minAngleEulerZ = new Vector3(0f, 0f, -25f);
        Vector3 maxAngleEulerZ = new Vector3(0f, 0f, 25f);
        Quaternion minAngleQuaternionZ = Quaternion.Euler(minAngleEulerZ);
        Quaternion maxAngleQuaternionZ = Quaternion.Euler(maxAngleEulerZ);
        
        if (printZ)
        {
            Debug.Log("minAngleQuaternionZ: " + minAngleQuaternionZ);
            Debug.Log("maxAngleQuaternionZ: " + maxAngleQuaternionZ);
        }

        // 2. Retrieve Target Quaternion, which is already calculated by the upper class.
        //Quaternion requiredRotationZ = desiredOrientation;
        //Quaternion requiredRotationZ = new Quaternion(0f, 0f, desiredOrientation.z, desiredOrientation.w);
        Quaternion requiredRotationZ = Quaternion.Euler(new Vector3(0f, 0f, angleZ));

        if (printZ)
        {
            Debug.Log("requiredRotationZ: " + requiredRotationZ);
        }

        // 3. Isoline and gains.
        float interceptZ = (0f) / (minAngleQuaternionZ.z - requiredRotationZ.z);
        float slopeZ = (minAngleQuaternionZ.z - requiredRotationZ.z) / (requiredRotationZ.z - maxAngleQuaternionZ.z);
        KPH = KPL * slopeZ + interceptZ;
        //float interceptZ = (0f) / (minAngleEuler.z - angleZ);
        //float slopeZ = (minAngleEuler.z - angleZ) / (angleZ - maxAngleEuler.z);
        //KPH = KPL * slopeZ + interceptZ;

        // 4. Calculate low and upper errors.
        //Quaternion lowError = QuaternionOpExtensions.SubtractTwo(minAngleQuaternion, currentOrientation);
        //Quaternion highError = QuaternionOpExtensions.SubtractTwo(maxAngleQuaternion, currentOrientation);
        Quaternion lowError = QuaternionOpExtensions.SubtractTwo(minAngleQuaternionZ, new Quaternion(0f, 0f, currentOrientation.z, currentOrientation.w));
        Quaternion highError = QuaternionOpExtensions.SubtractTwo(maxAngleQuaternionZ, new Quaternion(0f, 0f, currentOrientation.z, currentOrientation.w));

        if (printZ)
        {
            Debug.Log("lowError: " + minAngleQuaternionZ + " - " + new Quaternion(0f, 0f, currentOrientation.z, currentOrientation.w) + " = " + lowError);
            Debug.Log("highError: " + maxAngleQuaternionZ + " - " + new Quaternion(0f, 0f, currentOrientation.z, currentOrientation.w) + " = " + highError);
        }

        // 5. Angular Velocity.
        Quaternion angularVelocity = ToEulerAngleQuaternion(currentAngularVelocity);
        Quaternion delta = angularVelocity * requiredRotationZ;

        var orthogonalizeMatrix = new Matrix4x4()
        {
            m00 =
                                          -requiredRotationZ.x * -requiredRotationZ.x + -requiredRotationZ.y * -requiredRotationZ.y +
                                          -requiredRotationZ.z * -requiredRotationZ.z,
            m01 =
                                          -requiredRotationZ.x * requiredRotationZ.w + -requiredRotationZ.y * -requiredRotationZ.z +
                                          -requiredRotationZ.z * requiredRotationZ.y,
            m02 =
                                          -requiredRotationZ.x * requiredRotationZ.z + -requiredRotationZ.y * requiredRotationZ.w +
                                          -requiredRotationZ.z * -requiredRotationZ.x,
            m03 =
                                          -requiredRotationZ.x * -requiredRotationZ.y + -requiredRotationZ.y * requiredRotationZ.x +
                                          -requiredRotationZ.z * requiredRotationZ.w,
            m10 =
                                          requiredRotationZ.w * -requiredRotationZ.x + -requiredRotationZ.z * -requiredRotationZ.y +
                                          requiredRotationZ.y * -requiredRotationZ.z,
            m11 =
                                          requiredRotationZ.w * requiredRotationZ.w + -requiredRotationZ.z * -requiredRotationZ.z +
                                          requiredRotationZ.y * requiredRotationZ.y,
            m12 =
                                          requiredRotationZ.w * requiredRotationZ.z + -requiredRotationZ.z * requiredRotationZ.w +
                                          requiredRotationZ.y * -requiredRotationZ.x,
            m13 =
                                          requiredRotationZ.w * -requiredRotationZ.y + -requiredRotationZ.z * requiredRotationZ.x +
                                          requiredRotationZ.y * requiredRotationZ.w,
            m20 =
                                          requiredRotationZ.z * -requiredRotationZ.x + requiredRotationZ.w * -requiredRotationZ.y +
                                          -requiredRotationZ.x * -requiredRotationZ.z,
            m21 =
                                          requiredRotationZ.z * requiredRotationZ.w + requiredRotationZ.w * -requiredRotationZ.z +
                                          -requiredRotationZ.x * requiredRotationZ.y,
            m22 =
                                          requiredRotationZ.z * requiredRotationZ.z + requiredRotationZ.w * requiredRotationZ.w +
                                          -requiredRotationZ.x * -requiredRotationZ.x,
            m23 =
                                          requiredRotationZ.z * -requiredRotationZ.y + requiredRotationZ.w * requiredRotationZ.x +
                                          -requiredRotationZ.x * requiredRotationZ.w,
            m30 =
                                          -requiredRotationZ.y * -requiredRotationZ.x + requiredRotationZ.x * -requiredRotationZ.y +
                                          requiredRotationZ.w * -requiredRotationZ.z,
            m31 =
                                          -requiredRotationZ.y * requiredRotationZ.w + requiredRotationZ.x * -requiredRotationZ.z +
                                          requiredRotationZ.w * requiredRotationZ.y,
            m32 =
                                          -requiredRotationZ.y * requiredRotationZ.z + requiredRotationZ.x * requiredRotationZ.w +
                                          requiredRotationZ.w * -requiredRotationZ.x,
            m33 =
                                          -requiredRotationZ.y * -requiredRotationZ.y + requiredRotationZ.x * requiredRotationZ.x +
                                          requiredRotationZ.w * requiredRotationZ.w,
        };

        // From here, review

        Quaternion neededAngularVelocity = GetOutput(lowError, highError, delta, deltaTime);
        neededAngularVelocity = MultiplyAsVector(orthogonalizeMatrix, neededAngularVelocity);
        Quaternion doubleNegative = neededAngularVelocity.Multiply(-2.0f);
        Quaternion resultZ = doubleNegative * Quaternion.Inverse(requiredRotationZ);

        if (printZ)
        {
            Debug.Log("resultZ: " + resultZ);
            Debug.Log("---------------------------------");
        }

        return new Vector3(resultZ.x, resultZ.y, resultZ.z);
    }

    private Quaternion GetOutput(Quaternion lowError, Quaternion highError, Quaternion delta, float deltaTime)
    {
        var output = new Quaternion
        {
            x = this._internalAntagonisticController[0].GetOutput(lowError.x, highError.x, delta.x, deltaTime),
            y = this._internalAntagonisticController[1].GetOutput(lowError.y, highError.y, delta.y, deltaTime),
            z = this._internalAntagonisticController[2].GetOutput(lowError.z, highError.z, delta.z, deltaTime),
            w = this._internalAntagonisticController[3].GetOutput(lowError.w, highError.w, delta.w, deltaTime)
        };

        return output;
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

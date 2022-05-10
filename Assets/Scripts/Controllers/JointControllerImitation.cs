/****************************************************
 * File: JointControllerImitation.cs
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
using System;

public class JointControllerImitation
{
    #region Read-only & Static Fields

    // Set of Antagonistic Controllers for each joint. Each will contain 3 controllers, one for each axis.
    private readonly AntagonisticController[] _internalAntagonisticController;

    #endregion

    #region Instance Fields

    public float slopeX;
    public float interceptX;

    public float slopeY;
    public float interceptY;

    public float slopeZ;
    public float interceptZ;

    public float slopeXCurrent;
    public float interceptXCurrent;

    public float slopeYCurrent;
    public float interceptYCurrent;

    public float slopeZCurrent;
    public float interceptZCurrent;

    #endregion

    #region Constructors

    public JointControllerImitation(float plx, float phx, float kix, float kdx,
                                    float ply, float phy, float kiy, float kdy,
                                    float plz, float phz, float kiz, float kdz)
    {
        if ((plx < 0.0f) || (phx < 0.0f) || (kix < 0.0f) || (kdx < 0.0f) || 
            (ply < 0.0f) || (phy < 0.0f) || (kiy < 0.0f) || (kdy < 0.0f) || 
            (plz < 0.0f) || (phz < 0.0f) || (kiz < 0.0f) || (kdz < 0.0f))
        {
            throw new ArgumentOutOfRangeException("Gain Parameters", "Gains must be non-negative numbers.");
        }

        this._internalAntagonisticController = new[]
        {
            new AntagonisticController(plx, phx, kix, kdx),
            new AntagonisticController(ply, phy, kiy, kdy),
            new AntagonisticController(plz, phz, kiz, kdz),
        };
    }

    #endregion

    #region Instance Properties

    public float KPLX
    {
        get
        {
            return this._internalAntagonisticController[0].KPL;
        }
        set
        {
            this._internalAntagonisticController[0].KPL = value;
        }
    }

    public float KPHX
    {
        get
        {
            return this._internalAntagonisticController[0].KPH;
        }
        set
        {
            this._internalAntagonisticController[0].KPH = value;
        }
    }

    public float KDX
    {
        get
        {
            return this._internalAntagonisticController[0].KD;
        }
        set
        {
            this._internalAntagonisticController[0].KD = value;
        }
    }

    public float KIX
    {
        get
        {
            return this._internalAntagonisticController[0].KI;
        }
        set
        {
            this._internalAntagonisticController[0].KI = value;
        }
    }

    public float KPLY
    {
        get
        {
            return this._internalAntagonisticController[1].KPL;
        }
        set
        {
            this._internalAntagonisticController[1].KPL = value;
        }
    }

    public float KPHY
    {
        get
        {
            return this._internalAntagonisticController[1].KPH;
        }
        set
        {
            this._internalAntagonisticController[1].KPH = value;
        }
    }

    public float KDY
    {
        get
        {
            return this._internalAntagonisticController[1].KD;
        }
        set
        {
            this._internalAntagonisticController[1].KD = value;
        }
    }

    public float KIY
    {
        get
        {
            return this._internalAntagonisticController[1].KI;
        }
        set
        {
            this._internalAntagonisticController[1].KI = value;
        }
    }

    public float KPLZ
    {
        get
        {
            return this._internalAntagonisticController[2].KPL;
        }
        set
        {
            this._internalAntagonisticController[2].KPL = value;
        }
    }

    public float KPHZ
    {
        get
        {
            return this._internalAntagonisticController[2].KPH;
        }
        set
        {
            this._internalAntagonisticController[2].KPH = value;
        }
    }

    public float KDZ
    {
        get
        {
            return this._internalAntagonisticController[2].KD;
        }
        set
        {
            this._internalAntagonisticController[2].KD = value;
        }
    }

    public float KIZ
    {
        get
        {
            return this._internalAntagonisticController[2].KI;
        }
        set
        {
            this._internalAntagonisticController[2].KI = value;
        }
    }

    public float SlopeX
    {
        get
        {
            return slopeX;
        }
    }

    public float SlopeXCurrent
    {
        get
        {
            return slopeXCurrent;
        }
    }

    public float SlopeY
    {
        get
        {
            return slopeY;
        }
    }

    public float SlopeYCurrent
    {
        get
        {
            return slopeYCurrent;
        }
    }

    public float SlopeZ
    {
        get
        {
            return slopeZ;
        }
    }

    public float SlopeZCurrent
    {
        get
        {
            return slopeZCurrent;
        }
    }

    public float InterceptX
    {
        get
        {
            return interceptX;
        }
    }

    public float InterceptXCurrent
    {
        get
        {
            return interceptXCurrent;
        }
    }

    public float InterceptY
    {
        get
        {
            return interceptY;
        }
    }

    public float InterceptYCurrent
    {
        get
        {
            return interceptYCurrent;
        }
    }

    public float InterceptZ
    {
        get
        {
            return interceptZ;
        }
    }

    public float InterceptZCurrent
    {
        get
        {
            return interceptZCurrent;
        }
    }

    #endregion

    #region Instance Methods

    public Vector3 ComputeRequiredAntagonisticTorque(float minSoftLimitX, float maxSoftLimitX, float minSoftLimitY, float maxSoftLimitY, float minSoftLimitZ, float maxSoftLimitZ,
                                                     float minHardLimitX, float maxHardLimitX, float minHardLimitY, float maxHardLimitY, float minHardLimitZ, float maxHardLimitZ,
                                                     Transform currentTransform,
                                                     Quaternion currentLocalOrientation, Quaternion kinematicLocalOrientation, 
                                                     Rigidbody _objectRigidbody, Vector3 gravityTorqueVectorLocal, 
                                                     float fixedDeltaTime, 
                                                     bool debugModeAntagonistic, bool drawModeAntagonistic,
                                                     GameObject gameObject)
    {
        #region Orientations - X

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionX = GetRotationComponentAboutAxis(currentLocalOrientation, Vector3.right);
        Quaternion kinematicLocalOrientationQuaternionX = GetRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.right);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionXAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionXAxis = Vector3.zero;
        currentLocalOrientationQuaternionX.ToAngleAxis(out currentLocalOrientationQuaternionXAngle, out currentLocalOrientationQuaternionXAxis);
        currentLocalOrientationQuaternionXAxis.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionXAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionXAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionX.ToAngleAxis(out kinematicLocalOrientationQuaternionXAngle, out kinematicLocalOrientationQuaternionXAxis);
        kinematicLocalOrientationQuaternionXAxis.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionXAngleCorrected = (currentLocalOrientationQuaternionXAngle > 180) ? currentLocalOrientationQuaternionXAngle - 360 : currentLocalOrientationQuaternionXAngle;
        float kinematicLocalOrientationQuaternionXAngleCorrected = (kinematicLocalOrientationQuaternionXAngle > 180) ? kinematicLocalOrientationQuaternionXAngle - 360 : kinematicLocalOrientationQuaternionXAngle;

        // Print axis and angle
        if (debugModeAntagonistic)
        {
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Current Orientation X Axis: " + currentLocalOrientationQuaternionXAxis + " | Current Orientation X Angle: " + currentLocalOrientationQuaternionXAngleCorrected);
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Kinematic Orientation X Axis: " + kinematicLocalOrientationQuaternionXAxis + " | Kinematic Orientation X Angle: " + kinematicLocalOrientationQuaternionXAngleCorrected);
        }

        if(drawModeAntagonistic)
        {
            Debug.DrawRay(currentTransform.position, currentTransform.TransformDirection(currentLocalOrientationQuaternionXAxis), Color.red);
        }

        #endregion

        #region Estimate Errors directly from Axis-Angle - X

        float currentLocalErrorMinX = minSoftLimitX - currentLocalOrientationQuaternionXAngleCorrected;
        float currentLocalErrorMaxX = maxSoftLimitX - currentLocalOrientationQuaternionXAngleCorrected;

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinX: " + currentLocalErrorMinX + " | currentLocalErrorMaxX: " + currentLocalErrorMaxX);

        // Clamping Current Errors
        // 3) DEBUG: Limiting also current errors to keep coherence. However, you won't need it since the current hand (ragdoll) will never surpass the hard-limit, and the error is estimated wtr. the soft-limits
        //float currentLocalErrorMinXClamp = Mathf.Clamp(currentLocalErrorMinX, -100f, 0f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinXClamp: " + currentLocalErrorMinXClamp);
        //float currentLocalErrorMaxXClamp = Mathf.Clamp(currentLocalErrorMaxX, 0f, 100f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMaxXClamp: " + currentLocalErrorMaxXClamp);

        float kinematicLocalErrorMinX = minSoftLimitX - kinematicLocalOrientationQuaternionXAngleCorrected;
        float kinematicLocalErrorMaxX = maxSoftLimitX - kinematicLocalOrientationQuaternionXAngleCorrected;

        // Clamping Kinematic Errors
        float kinematicLocalErrorMinXClamp = Mathf.Clamp(kinematicLocalErrorMinX, (minSoftLimitX - maxHardLimitX), (minSoftLimitX - minHardLimitX));
        float kinematicLocalErrorMaxXClamp = Mathf.Clamp(kinematicLocalErrorMaxX, (maxSoftLimitX - maxHardLimitX), (maxSoftLimitX - minHardLimitX));

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinXClamp: " + kinematicLocalErrorMinXClamp + " | kinematicLocalErrorMaxXClamp: " + kinematicLocalErrorMaxXClamp);

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Total Current Error X: " + (currentLocalErrorMinX - currentLocalErrorMaxX));

        #endregion

        #region Isoline with Angle errors - X

        // 1) DEBUG: Not used clamped values, makes the hand to stop first in the hard limit, but the negative slope is still calculated
        //interceptX = (-gravityTorqueVectorLocal.x) / kinematicLocalErrorMaxX;
        //slopeX = (kinematicLocalErrorMinX) / (-(kinematicLocalErrorMaxX));
        //KPHX = KPLX * slopeX + interceptX;

        // Isoline with Clamped Errors
        // 2) DEBUG: This makes the slope to be limited within the range of the hard limits
        interceptX = (-gravityTorqueVectorLocal.x) / (kinematicLocalErrorMaxXClamp);
        slopeX = (kinematicLocalErrorMinXClamp) / (-(kinematicLocalErrorMaxXClamp));
        KPHX = KPLX * slopeX + interceptX; // ERROR: KPH is NaN is the denominator is 0

        #endregion

        #region Isoline with CURRENT Angle errors - X (Debug)

        interceptXCurrent = (-gravityTorqueVectorLocal.x) / currentLocalErrorMaxX;
        slopeXCurrent = (currentLocalErrorMinX) / (-(currentLocalErrorMaxX));

        #endregion

        #region Orientations - Y

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionY = GetRotationComponentAboutAxis(currentLocalOrientation, Vector3.up);
        Quaternion kinematicLocalOrientationQuaternionY = GetRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.up);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionYAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionYAxis = Vector3.zero;
        currentLocalOrientationQuaternionY.ToAngleAxis(out currentLocalOrientationQuaternionYAngle, out currentLocalOrientationQuaternionYAxis);
        currentLocalOrientationQuaternionYAxis.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionYAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionYAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionY.ToAngleAxis(out kinematicLocalOrientationQuaternionYAngle, out kinematicLocalOrientationQuaternionYAxis);
        kinematicLocalOrientationQuaternionYAxis.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionYAngleCorrected = (currentLocalOrientationQuaternionYAngle > 180) ? currentLocalOrientationQuaternionYAngle - 360 : currentLocalOrientationQuaternionYAngle;
        float kinematicLocalOrientationQuaternionYAngleCorrected = (kinematicLocalOrientationQuaternionYAngle > 180) ? kinematicLocalOrientationQuaternionYAngle - 360 : kinematicLocalOrientationQuaternionYAngle;

        // Print axis and angle
        if (debugModeAntagonistic)
        {
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Current Orientation Y Axis: " + currentLocalOrientationQuaternionYAxis + " | Current Orientation Y Angle: " + currentLocalOrientationQuaternionYAngleCorrected);
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Kinematic Orientation Y Axis: " + kinematicLocalOrientationQuaternionYAxis + " | Kinematic Orientation Y Angle: " + kinematicLocalOrientationQuaternionYAngleCorrected);
        }

        if (drawModeAntagonistic)
        {
            Debug.DrawRay(currentTransform.position, currentTransform.TransformDirection(currentLocalOrientationQuaternionYAxis), Color.green);
        }

        #endregion

        #region Estimate Errors directly from Axis-Angle - Y

        float currentLocalErrorMinY = minSoftLimitY - currentLocalOrientationQuaternionYAngleCorrected;
        float currentLocalErrorMaxY = maxSoftLimitY - currentLocalOrientationQuaternionYAngleCorrected;
        
        if(debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinY: " + currentLocalErrorMinY + " | currentLocalErrorMaxY: " + currentLocalErrorMaxY);

        float kinematicLocalErrorMinY = minSoftLimitY - kinematicLocalOrientationQuaternionYAngleCorrected;
        float kinematicLocalErrorMaxY = maxSoftLimitY - kinematicLocalOrientationQuaternionYAngleCorrected;

        // Clamping Kinematic Errors
        float kinematicLocalErrorMinYClamp = Mathf.Clamp(kinematicLocalErrorMinY, (minSoftLimitY - maxHardLimitY), (minSoftLimitY - minHardLimitY));
        float kinematicLocalErrorMaxYClamp = Mathf.Clamp(kinematicLocalErrorMaxY, (maxSoftLimitY - maxHardLimitY), (maxSoftLimitY - minHardLimitY));

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinYClamp: " + kinematicLocalErrorMinYClamp + " | kinematicLocalErrorMaxYClamp: " + kinematicLocalErrorMaxYClamp);

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Total Current Error Y: " + (currentLocalErrorMinY - currentLocalErrorMaxY));

        #endregion

        #region Isoline with Angle errors - Y

        // Isoline with Clamped Errors
        interceptY = (-gravityTorqueVectorLocal.y) / kinematicLocalErrorMaxYClamp;
        slopeY = (kinematicLocalErrorMinYClamp) / (-(kinematicLocalErrorMaxYClamp));
        KPHY = KPLY * slopeY + interceptY; // ERROR: KPH is NaN is the denominator is 0

        #endregion

        #region Isoline with CURRENT Angle errors - Y (Debug)

        interceptYCurrent = (-gravityTorqueVectorLocal.y) / currentLocalErrorMaxY;
        slopeYCurrent = (currentLocalErrorMinY) / (-(currentLocalErrorMaxY));

        #endregion

        #region Orientations - Z

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionZ = GetRotationComponentAboutAxis(currentLocalOrientation, Vector3.forward);
        Quaternion kinematicLocalOrientationQuaternionZ = GetRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.forward);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionZAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionZAxis = Vector3.zero;
        currentLocalOrientationQuaternionZ.ToAngleAxis(out currentLocalOrientationQuaternionZAngle, out currentLocalOrientationQuaternionZAxis);
        currentLocalOrientationQuaternionZAxis.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionZAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionZAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionZ.ToAngleAxis(out kinematicLocalOrientationQuaternionZAngle, out kinematicLocalOrientationQuaternionZAxis);
        kinematicLocalOrientationQuaternionZAxis.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionZAngleCorrected = (currentLocalOrientationQuaternionZAngle > 180) ? currentLocalOrientationQuaternionZAngle - 360 : currentLocalOrientationQuaternionZAngle;
        float kinematicLocalOrientationQuaternionZAngleCorrected = (kinematicLocalOrientationQuaternionZAngle > 180) ? kinematicLocalOrientationQuaternionZAngle - 360 : kinematicLocalOrientationQuaternionZAngle;

        // Print axis and angle
        if (debugModeAntagonistic)
        {
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Current Orientation Z Axis: " + currentLocalOrientationQuaternionZAxis + " | Current Orientation Z Angle: " + currentLocalOrientationQuaternionZAngleCorrected);
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Kinematic Orientation Z Axis: " + kinematicLocalOrientationQuaternionZAxis + " | Kinematic Orientation Z Angle: " + kinematicLocalOrientationQuaternionZAngleCorrected);
        }

        if (drawModeAntagonistic)
        {
            Debug.DrawRay(currentTransform.position, currentTransform.TransformDirection(currentLocalOrientationQuaternionZAxis), Color.blue);
        }

        #endregion

        #region Estimate Errors directly from Axis-Angle - Z

        float currentLocalErrorMinZ = minSoftLimitZ - currentLocalOrientationQuaternionZAngleCorrected;
        float currentLocalErrorMaxZ = maxSoftLimitZ - currentLocalOrientationQuaternionZAngleCorrected;

        if(debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinZ: " + currentLocalErrorMinZ + " | currentLocalErrorMaxZ: " + currentLocalErrorMaxZ);

        float kinematicLocalErrorMinZ = minSoftLimitZ - kinematicLocalOrientationQuaternionZAngleCorrected;
        float kinematicLocalErrorMaxZ = maxSoftLimitZ - kinematicLocalOrientationQuaternionZAngleCorrected;
        
        // Clamping Kinematic Errors
        float kinematicLocalErrorMinZClamp = Mathf.Clamp(kinematicLocalErrorMinZ, (minSoftLimitZ - maxHardLimitZ), (minSoftLimitZ - minHardLimitZ));
        float kinematicLocalErrorMaxZClamp = Mathf.Clamp(kinematicLocalErrorMaxZ, (maxSoftLimitZ - maxHardLimitZ), (maxSoftLimitZ - minHardLimitZ));

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinZClamp: " + kinematicLocalErrorMinZClamp + " | kinematicLocalErrorMaxZClamp: " + kinematicLocalErrorMaxZClamp);

        if (debugModeAntagonistic)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Total Current Error Z: " + (currentLocalErrorMinZ - currentLocalErrorMaxZ));

        #endregion

        #region Isoline with Angle errors - Z

        // Isoline with Clamped Errors
        interceptZ = (-gravityTorqueVectorLocal.z) / kinematicLocalErrorMaxZClamp;
        slopeZ = (kinematicLocalErrorMinZClamp) / (-(kinematicLocalErrorMaxZClamp));
        KPHZ = KPLZ * slopeZ + interceptZ; // ERROR: KPH is NaN is the denominator is 0

        #endregion

        #region Isoline with CURRENT Angle errors - Z (Debug)

        interceptZCurrent = (-gravityTorqueVectorLocal.z) / currentLocalErrorMaxZ;
        slopeZCurrent = (currentLocalErrorMinZ) / (-(currentLocalErrorMaxZ));

        #endregion

        #region Torque Estimation

        Vector3 torqueApplied = GetOutput(currentLocalErrorMinX, currentLocalErrorMaxX, 
                                          currentLocalErrorMinY, currentLocalErrorMaxY, 
                                          currentLocalErrorMinZ, currentLocalErrorMaxZ, 
                                          _objectRigidbody.angularVelocity.magnitude, fixedDeltaTime);

        if (debugModeAntagonistic)
        {
            Debug.Log("[INFO] torqueApplied (acceleration): " + torqueApplied);
        }

        #endregion

        #region Inertia

        torqueApplied = _objectRigidbody.inertiaTensorRotation * torqueApplied;
        torqueApplied.Scale(_objectRigidbody.inertiaTensor);
        torqueApplied = Quaternion.Inverse(_objectRigidbody.inertiaTensorRotation) * torqueApplied;
        
        if (debugModeAntagonistic)
        {
            Debug.Log("[INFO] torqueApplied (torque): " + torqueApplied);
        }

        if (drawModeAntagonistic)
        {
            Debug.DrawRay(currentTransform.position, currentTransform.TransformDirection(torqueApplied), Color.yellow);
        }

        #endregion

        return torqueApplied;
    }

    private Vector3 GetOutput(float lowErrorX, float highErrorX, float lowErrorY, float highErrorY, float lowErrorZ, float highErrorZ, float delta, float deltaTime)
    {
        var output = new Vector3
        {
            x = this._internalAntagonisticController[0].GetOutput(lowErrorX, highErrorX, delta, deltaTime),
            y = this._internalAntagonisticController[1].GetOutput(lowErrorY, highErrorY, delta, deltaTime),
            z = this._internalAntagonisticController[2].GetOutput(lowErrorZ, highErrorZ, delta, deltaTime),
        };

        return output;
    }

    // Swing-Twist Decomposition - TODO 
    private Quaternion GetRotationComponentAboutAxis(Quaternion rotation, Vector3 direction)
    {
        // Extract complex part of the quaternion and represent it as a direction
        Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);

        float dotProd = Vector3.Dot(rotationAxis, direction);

        // Shortcut calculation of `projection` requires `direction` to be normalized
        Vector3 projection = new Vector3(direction.x * dotProd, direction.y * dotProd, direction.z * dotProd);

        Quaternion twist = new Quaternion(
                projection.x, projection.y, projection.z, rotation.w).normalized;
        if (dotProd < 0.0)
        {
            // Ensure `twist` points towards `direction`
            twist.x = -twist.x;
            twist.y = -twist.y;
            twist.z = -twist.z;
            twist.w = -twist.w;
            // Rotation angle `twist.angle()` is now reliable
        }

        // If I want also the swing, remember that the quaternion is decomposed into two concatenated quaternions, Q = (S)wing * (T)wist
        // Therefore, S = RT^1

        return twist;
    }

    // Full Swing-Twist Decomposition - TODO
    private Quaternion SwingTwistDecomposition(Quaternion q, Vector3 twistAxis)
    {
        // Extract complex part of the quaternion and represent it as a direction
        Vector3 r = new Vector3(q.x, q.y, q.z);

        Vector3 rotatedTwistAxis = q * twistAxis;
        Vector3 swingAxis = Vector3.Cross(twistAxis, rotatedTwistAxis);

        float swingAngle = Vector3.Angle(twistAxis, rotatedTwistAxis);
        Quaternion swing = Quaternion.AngleAxis(swingAngle, swingAxis);

        Vector3 p = Vector3.Project(r, twistAxis);

        Quaternion twist = new Quaternion(p.x, p.y, p.z, q.w).normalized;

        swing = q * Quaternion.Inverse(twist);

        return twist;
    }

    #endregion
}

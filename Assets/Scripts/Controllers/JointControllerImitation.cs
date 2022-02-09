/****************************************************
 * File: JointControllerImitation.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2022
   * Last update: 27/01/2022
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

    #endregion

    #region Instance Methods

    public Vector3 ComputeRequiredAntagonisticTorque(float minAngleX, float maxAngleX, float minAngleY, float maxAngleY, float minAngleZ, float maxAngleZ,
                                                     Quaternion currentLocalOrientation, Quaternion currentGlobalOrientation, 
                                                     Quaternion kinematicLocalOrientation, Quaternion kinematicGlobalOrientation, 
                                                     Quaternion desiredLocalRotation, Vector3 angularVelocity, Vector3 gravityTorqueVectorLocal, float fixedDeltaTime, 
                                                     bool debugMode, Transform currentTransform)
    {

        #region Orientations - X

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionX = getRotationComponentAboutAxis(currentLocalOrientation, Vector3.right);
        Quaternion kinematicLocalOrientationQuaternionX = getRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.right);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionXAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionXAxis = Vector3.zero;
        currentLocalOrientationQuaternionX.ToAngleAxis(out currentLocalOrientationQuaternionXAngle, out currentLocalOrientationQuaternionXAxis);
        currentLocalOrientationQuaternionX.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionXAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionXAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionX.ToAngleAxis(out kinematicLocalOrientationQuaternionXAngle, out kinematicLocalOrientationQuaternionXAxis);
        kinematicLocalOrientationQuaternionX.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionXAngleCorrected = (currentLocalOrientationQuaternionXAngle > 180) ? currentLocalOrientationQuaternionXAngle - 360 : currentLocalOrientationQuaternionXAngle;
        float kinematicLocalOrientationQuaternionXAngleCorrected = (kinematicLocalOrientationQuaternionXAngle > 180) ? kinematicLocalOrientationQuaternionXAngle - 360 : kinematicLocalOrientationQuaternionXAngle;

        // Print axis and angle
        if (debugMode)
        {
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Current Orientation X Axis: " + currentLocalOrientationQuaternionXAxis + " | Current Orientation X Angle: " + currentLocalOrientationQuaternionXAngleCorrected);
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Kinematic Orientation X Axis: " + kinematicLocalOrientationQuaternionXAxis + " | Kinematic Orientation X Angle: " + kinematicLocalOrientationQuaternionXAngleCorrected);
        }

        #endregion

        #region Estimate Errors directly from Axis-Angle - X

        float currentLocalErrorMinX = minAngleX - currentLocalOrientationQuaternionXAngleCorrected;
        float currentLocalErrorMaxX = maxAngleX - currentLocalOrientationQuaternionXAngleCorrected;

        if (debugMode)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinX: " + currentLocalErrorMinX + " | currentLocalErrorMaxX: " + currentLocalErrorMaxX);

        // TEST - Clamping Current Errors
        //float currentLocalErrorMinXClamp = Mathf.Clamp(currentLocalErrorMinX, -99f, -1f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinXClamp: " + currentLocalErrorMinXClamp);
        //float currentLocalErrorMaxXClamp = Mathf.Clamp(currentLocalErrorMaxX, 1f, 99f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMaxXClamp: " + currentLocalErrorMaxXClamp);

        float kinematicLocalErrorMinX = minAngleX - kinematicLocalOrientationQuaternionXAngleCorrected;
        float kinematicLocalErrorMaxX = maxAngleX - kinematicLocalOrientationQuaternionXAngleCorrected;
        
        if(debugMode)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinX: " + kinematicLocalErrorMinX + " | kinematicLocalErrorMaxX: " + kinematicLocalErrorMaxX);

        // TEST - Clamping Kinematic Errors
        //float kinematicLocalErrorMinXClamp = Mathf.Clamp(kinematicLocalErrorMinX, -99f, -1f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinXClamp: " + kinematicLocalErrorMinXClamp);
        //float kinematicLocalErrorMaxXClamp = Mathf.Clamp(kinematicLocalErrorMaxX, 1f, 99f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMaxXClamp: " + kinematicLocalErrorMaxXClamp);

        #endregion

        #region Isoline with Angle errors - X

        float interceptX = (0f) / kinematicLocalErrorMaxX;
        float slopeX = (kinematicLocalErrorMinX) / (-(kinematicLocalErrorMaxX));
        KPHX = KPLX * slopeX + interceptX;

        // TEST - Isoline with Clamped Errors
        //float interceptX = (0f) / kinematicLocalErrorMaxXClamp;
        //float slopeX = (kinematicLocalErrorMinXClamp) / (-(kinematicLocalErrorMaxXClamp));
        //KPHX = KPLX * slopeX + interceptX; // ERROR: KPH is NaN

        #endregion

        #region Orientations - Y

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionY = getRotationComponentAboutAxis(currentLocalOrientation, Vector3.up);
        Quaternion kinematicLocalOrientationQuaternionY = getRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.up);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionYAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionYAxis = Vector3.zero;
        currentLocalOrientationQuaternionY.ToAngleAxis(out currentLocalOrientationQuaternionYAngle, out currentLocalOrientationQuaternionYAxis);
        currentLocalOrientationQuaternionY.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionYAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionYAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionY.ToAngleAxis(out kinematicLocalOrientationQuaternionYAngle, out kinematicLocalOrientationQuaternionYAxis);
        kinematicLocalOrientationQuaternionY.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionYAngleCorrected = (currentLocalOrientationQuaternionYAngle > 180) ? currentLocalOrientationQuaternionYAngle - 360 : currentLocalOrientationQuaternionYAngle;
        float kinematicLocalOrientationQuaternionYAngleCorrected = (kinematicLocalOrientationQuaternionYAngle > 180) ? kinematicLocalOrientationQuaternionYAngle - 360 : kinematicLocalOrientationQuaternionYAngle;

        // Print axis and angle
        if (debugMode)
        {
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Current Orientation Y Axis: " + currentLocalOrientationQuaternionYAxis + " | Current Orientation Y Angle: " + currentLocalOrientationQuaternionYAngleCorrected);
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Kinematic Orientation Y Axis: " + kinematicLocalOrientationQuaternionYAxis + " | Kinematic Orientation Y Angle: " + kinematicLocalOrientationQuaternionYAngleCorrected);
        }

        #endregion

        #region Estimate Errors directly from Axis-Angle - Y

        float currentLocalErrorMinY = minAngleY - currentLocalOrientationQuaternionYAngleCorrected;
        float currentLocalErrorMaxY = maxAngleY - currentLocalOrientationQuaternionYAngleCorrected;
        
        if(debugMode)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinY: " + currentLocalErrorMinY + " | currentLocalErrorMaxY: " + currentLocalErrorMaxY);

        // TEST - Clamping Current Errors
        //float currentLocalErrorMinYClamp = Mathf.Clamp(currentLocalErrorMinY, -100f, -1f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinYClamp: " + currentLocalErrorMinYClamp);
        //float currentLocalErrorMaxYClamp = Mathf.Clamp(currentLocalErrorMaxY, 1f, 100f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMaxYClamp: " + currentLocalErrorMaxYClamp);

        float kinematicLocalErrorMinY = minAngleY - kinematicLocalOrientationQuaternionYAngleCorrected;
        float kinematicLocalErrorMaxY = maxAngleY - kinematicLocalOrientationQuaternionYAngleCorrected;
        
        if(debugMode)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinY: " + kinematicLocalErrorMinY + " | kinematicLocalErrorMaxY: " + kinematicLocalErrorMaxY);

        // TEST - Clamping Kinematic Errors
        //float kinematicLocalErrorMinYClamp = Mathf.Clamp(kinematicLocalErrorMinY, -100f, -1f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinYClamp: " + kinematicLocalErrorMinYClamp);
        //float kinematicLocalErrorMaxYClamp = Mathf.Clamp(kinematicLocalErrorMaxY, 1f, 100f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMaxYClamp: " + kinematicLocalErrorMaxYClamp);

        #endregion

        #region Isoline with Angle errors - Y

        float interceptY = (0f) / kinematicLocalErrorMaxY;
        float slopeY = (kinematicLocalErrorMinY) / (-(kinematicLocalErrorMaxY));
        KPHY = KPLY * slopeY + interceptY;

        // TEST - Isoline with Clamped Errors
        //float interceptY = (0f) / kinematicLocalErrorMaxYClamp;
        //float slopeY = (kinematicLocalErrorMinYClamp) / (-(kinematicLocalErrorMaxYClamp));
        //KPHY = KPLY * slopeY + interceptY; // ERROR: KPH is NaN

        #endregion

        #region Orientations - Z

        // Swing-Twist Decomposition of current/kinematic orientation on main axis
        Quaternion currentLocalOrientationQuaternionZ = getRotationComponentAboutAxis(currentLocalOrientation, Vector3.forward);
        Quaternion kinematicLocalOrientationQuaternionZ = getRotationComponentAboutAxis(kinematicLocalOrientation, Vector3.forward);

        // Axis-Angle Conversion of current orientation
        float currentLocalOrientationQuaternionZAngle = 0.0f;
        Vector3 currentLocalOrientationQuaternionZAxis = Vector3.zero;
        currentLocalOrientationQuaternionZ.ToAngleAxis(out currentLocalOrientationQuaternionZAngle, out currentLocalOrientationQuaternionZAxis);
        currentLocalOrientationQuaternionZ.Normalize();

        // Axis-Angle Conversion of kinematic orientation
        float kinematicLocalOrientationQuaternionZAngle = 0.0f;
        Vector3 kinematicLocalOrientationQuaternionZAxis = Vector3.zero;
        kinematicLocalOrientationQuaternionZ.ToAngleAxis(out kinematicLocalOrientationQuaternionZAngle, out kinematicLocalOrientationQuaternionZAxis);
        kinematicLocalOrientationQuaternionZ.Normalize();

        // Angles to be negative when < 0f
        float currentLocalOrientationQuaternionZAngleCorrected = (currentLocalOrientationQuaternionZAngle > 180) ? currentLocalOrientationQuaternionZAngle - 360 : currentLocalOrientationQuaternionZAngle;
        float kinematicLocalOrientationQuaternionZAngleCorrected = (kinematicLocalOrientationQuaternionZAngle > 180) ? kinematicLocalOrientationQuaternionZAngle - 360 : kinematicLocalOrientationQuaternionZAngle;

        // Print axis and angle
        if (debugMode)
        {
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Current Orientation Z Axis: " + currentLocalOrientationQuaternionZAxis + " | Current Orientation Z Angle: " + currentLocalOrientationQuaternionZAngleCorrected);
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] Kinematic Orientation Z Axis: " + kinematicLocalOrientationQuaternionZAxis + " | Kinematic Orientation Z Angle: " + kinematicLocalOrientationQuaternionZAngleCorrected);
        }

        #endregion

        #region Estimate Errors directly from Axis-Angle - Z

        float currentLocalErrorMinZ = minAngleZ - currentLocalOrientationQuaternionZAngleCorrected;
        float currentLocalErrorMaxZ = maxAngleZ - currentLocalOrientationQuaternionZAngleCorrected;

        if(debugMode)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinZ: " + currentLocalErrorMinZ + " | currentLocalErrorMaxZ: " + currentLocalErrorMaxZ);

        // TEST - Clamping Current Errors
        //float currentLocalErrorMinZClamp = Mathf.Clamp(currentLocalErrorMinZ, -180f, -3f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMinZClamp: " + currentLocalErrorMinZClamp);
        //float currentLocalErrorMaxZClamp = Mathf.Clamp(currentLocalErrorMaxZ, 3f, 180f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] currentLocalErrorMaxZClamp: " + currentLocalErrorMaxZClamp);

        float kinematicLocalErrorMinZ = minAngleZ - kinematicLocalOrientationQuaternionZAngleCorrected;
        float kinematicLocalErrorMaxZ = maxAngleZ - kinematicLocalOrientationQuaternionZAngleCorrected;
        
        if(debugMode)
            Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinZ: " + kinematicLocalErrorMinZ + " | kinematicLocalErrorMaxZ: " + kinematicLocalErrorMaxZ);

        // TEST - Clamping Kinematic Errors
        //float kinematicLocalErrorMinZClamp = Mathf.Clamp(kinematicLocalErrorMinZ, -180f, -3f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMinZClamp: " + kinematicLocalErrorMinZClamp);
        //float kinematicLocalErrorMaxZClamp = Mathf.Clamp(kinematicLocalErrorMaxZ, 3f, 180f);
        //Debug.Log("[INFO: " + currentTransform.gameObject.name + "] kinematicLocalErrorMaxZClamp: " + kinematicLocalErrorMaxZClamp);

        #endregion

        #region Isoline with Angle errors - Z

        float interceptZ = (-gravityTorqueVectorLocal.z) / kinematicLocalErrorMaxZ;
        float slopeZ = (kinematicLocalErrorMinZ) / (-(kinematicLocalErrorMaxZ));
        KPHZ = KPLZ * slopeZ + interceptZ;

        // TEST - Isoline with Clamped Errors
        //float interceptZ = (-gravityTorqueVectorLocal.z) / kinematicLocalErrorMaxZClamp;
        //float slopeZ = (kinematicLocalErrorMinZClamp) / (-(kinematicLocalErrorMaxZClamp));
        //KPHZ = KPLZ * slopeZ + interceptZ;

        #endregion

        Vector3 torqueApplied = GetOutput(currentLocalErrorMinX, currentLocalErrorMaxX, currentLocalErrorMinY, currentLocalErrorMaxY, currentLocalErrorMinZ, currentLocalErrorMaxZ, angularVelocity.magnitude, Time.fixedDeltaTime);

        // TEST - Torque with Clamped Errors
        //Vector3 torqueApplied = GetOutput(currentLocalErrorMinXClamp, currentLocalErrorMaxXClamp, currentLocalErrorMinY, currentLocalErrorMaxY, currentLocalErrorMinZ, currentLocalErrorMaxZ, angularVelocity.magnitude, Time.fixedDeltaTime);

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
    private Quaternion getRotationComponentAboutAxis(Quaternion rotation, Vector3 direction)
    {
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
        return twist;
    }

    #endregion
}

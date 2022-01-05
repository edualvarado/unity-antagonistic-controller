// <copyright file="ControlledObject.cs" company="VacuumBreather">
//      Copyright © 2015 VacuumBreather. All rights reserved.
// </copyright>

namespace VacuumBreather
{
    #region Using Directives

    using UnityEngine;

    #endregion

    /// <summary>
    ///     An object that uses the Quaternion PID Controller to orient itself.
    /// </summary>
    public class ControlledObject : MonoBehaviour
    {
        #region Read-only & Static Fields

        private readonly PidQuaternionController _pidController = new PidQuaternionController(8.0f, 0.0f, 0.05f);

        #endregion

        #region Fields

        private Transform _currentTransform;
        private Rigidbody _objectRigidbody;

        public float Kp;
        public float Ki;
        public float Kd;

        #endregion

        #region Instance Properties

        public Quaternion DesiredOrientation { get; set; }

        #endregion

        #region Instance Methods

        private void Awake()
        {
            this._currentTransform = transform;
            this._objectRigidbody = GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            if (DesiredOrientation == null || this._currentTransform == null || this._objectRigidbody == null)
            {
                return;
            }

            this._pidController.Kp = this.Kp;
            this._pidController.Ki = this.Ki;
            this._pidController.Kd = this.Kd;

            // The PID controller takes the current orientation of an object, its desired orientation and the current angular velocity
            // and returns the required angular acceleration to rotate towards the desired orientation.
            Vector3 requiredAngularAcceleration = this._pidController.ComputeRequiredAngularAcceleration(this._currentTransform.rotation,
                                                                                                         DesiredOrientation,
                                                                                                         this._objectRigidbody.angularVelocity,
                                                                                                         Time.fixedDeltaTime);

            this._objectRigidbody.AddTorque(requiredAngularAcceleration, ForceMode.Acceleration);
        }

        #endregion
    }
}
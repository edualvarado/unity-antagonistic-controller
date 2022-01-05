// <copyright file="PidController.cs" company="VacuumBreather">
//      Copyright © 2015 VacuumBreather. All rights reserved.
// </copyright>

namespace VacuumBreather
{
    #region Using Directives

    using System;
    using UnityEngine;

    #endregion

    /// <summary>
    ///     A standard PID controller implementation.
    /// </summary>
    /// <remarks>
    ///     See https://en.wikipedia.org/wiki/PID_controller.
    /// </remarks>
    public class PidController
    {
        #region Constants

        private const float MaxOutput = 1000.0f;

        #endregion

        #region Fields

        private float _integralMax;
        private float _integral;

        private float _kp;
        private float _ki;
        private float _kd;

        #endregion

        #region Constructors

        /// <summary>
        ///     Initializes a new instance of the <see cref="PidController" /> class.
        /// </summary>
        /// <param name="kp">The proportional gain.</param>
        /// <param name="ki">The integral gain.</param>
        /// <param name="kd">The derivative gain.</param>
        /// <exception cref="ArgumentOutOfRangeException">If one of the parameters is negative.</exception>
        public PidController(float kp, float ki, float kd)
        {
            if (kp < 0.0f)
            {
                throw new ArgumentOutOfRangeException("kp", "kp must be a non-negative number.");
            }

            if (ki < 0.0f)
            {
                throw new ArgumentOutOfRangeException("ki", "ki must be a non-negative number.");
            }

            if (kd < 0.0f)
            {
                throw new ArgumentOutOfRangeException("kd", "kd must be a non-negative number.");
            }

            Kp = kp;
            Ki = ki;
            Kd = kd;

            this._integralMax = MaxOutput / Ki;
        }

        #endregion

        #region Instance Properties

        /// <summary>
        ///     Gets or sets the proportional gain.
        /// </summary>
        /// <value>
        ///     The proportional gain.
        /// </value>
        public float Kp
        {
            get
            {
                return this._kp;
            }
            set
            {
                if (value < 0.0f)
                {
                    throw new ArgumentOutOfRangeException("value", "Kp must be a non-negative number.");
                }

                this._kp = value;
            }
        }

        /// <summary>
        ///     Gets or sets the integral gain.
        /// </summary>
        /// <value>
        ///     The integral gain.
        /// </value>
        public float Ki
        {
            get
            {
                return this._ki;
            }
            set
            {
                if (value < 0.0f)
                {
                    throw new ArgumentOutOfRangeException("value", "Ki must be a non-negative number.");
                }

                this._ki = value;

                this._integralMax = MaxOutput / Ki;
                this._integral = Mathf.Clamp(this._integral, -this._integralMax, this._integralMax);
            }
        }

        /// <summary>
        ///     Gets or sets the derivative gain.
        /// </summary>
        /// <value>
        ///     The derivative gain.
        /// </value>
        public float Kd
        {
            get
            {
                return this._kd;
            }
            set
            {
                if (value < 0.0f)
                {
                    throw new ArgumentOutOfRangeException("value", "Kd must be a non-negative number.");
                }

                this._kd = value;
            }
        }

        #endregion

        #region Instance Methods

        /// <summary>
        ///     Computes the corrective output.
        /// </summary>
        /// <param name="error">The current error of the signal.</param>
        /// <param name="delta">The delta of the signal since last frame.</param>
        /// <param name="deltaTime">The delta time.</param>
        /// <returns>The corrective output.</returns>
        public float ComputeOutput(float error, float delta, float deltaTime)
        {
            this._integral += (error * deltaTime);
            this._integral = Mathf.Clamp(this._integral, -this._integralMax, this._integralMax);

            float derivative = delta / deltaTime;
            float output = (Kp * error) + (Ki * this._integral) + (Kd * derivative);

            output = Mathf.Clamp(output, -MaxOutput, MaxOutput);

            return output;
        }

        #endregion
    }
}
// <copyright file="QuaternionExtensions.cs" company="VacuumBreather">
//      Copyright © 2015 VacuumBreather. All rights reserved.
// </copyright>

namespace VacuumBreather
{
    #region Using Directives

    using UnityEngine;

    #endregion

    /// <summary>
    ///     Extension methods for the Quaternion struct.
    /// </summary>
    public static class QuaternionExtensions
    {
        #region Class Methods

        /// <summary>
        ///     Multiplies the quaternion by a scalar.
        /// </summary>
        /// <param name="quaternion">The quaternion.</param>
        /// <param name="scalar">The scalar.</param>
        /// <returns>
        ///     The product of the quaternion and the scalar.
        /// </returns>
        public static Quaternion Multiply(this Quaternion quaternion, float scalar)
        {
            return new Quaternion((float)((double)quaternion.x * (double)scalar),
                                  (float)((double)quaternion.y * (double)scalar),
                                  (float)((double)quaternion.z * (double)scalar),
                                  (float)((double)quaternion.w * (double)scalar));
        }

        /// <summary>
        ///     Computes the required rotation to make the first orientation match the second.
        /// </summary>
        /// <param name="from">The first orientation.</param>
        /// <param name="to">The second orientation.</param>
        /// <returns>The required rotation to make the first orientation match the second.</returns>
        public static Quaternion RequiredRotation(Quaternion from, Quaternion to)
        {
            Quaternion requiredRotation = to * Quaternion.Inverse(from);

            // Flip the sign if w is negative.
            // This makes sure we always rotate the shortest angle to match the desired rotation.
            if (requiredRotation.w < 0.0f)
            {
                requiredRotation.x *= -1.0f;
                requiredRotation.y *= -1.0f;
                requiredRotation.z *= -1.0f;
                requiredRotation.w *= -1.0f;
            }

            return requiredRotation;
        }

        /// <summary>
        ///     Subtracts one Quaternion from another.
        /// </summary>
        /// <param name="lhs">The left hand side operand.</param>
        /// <param name="rhs">The right hand side operand</param>
        /// <returns>The difference between both quaternions.</returns>
        public static Quaternion Subtract(this Quaternion lhs, Quaternion rhs)
        {
            return new Quaternion((float)((double)lhs.x - (double)rhs.x),
                                  (float)((double)lhs.y - (double)rhs.y),
                                  (float)((double)lhs.z - (double)rhs.z),
                                  (float)((double)lhs.w - (double)rhs.w));
        }

        #endregion
    }
}
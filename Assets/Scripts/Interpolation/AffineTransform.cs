/****************************************************
 * File: AffineTransform.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Hybrid Blending Animations
   * Last update: 01/08/2021
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Value type to encapsulate transformation/quaternion data for the bone transforms.
/// Value type: Every time we create an instance of this struct, the value is copied.
/// Changes made to that variable do not affect the other variables.
/// Augment with methods if needed.
/// </summary>
public struct AffineTransform
{
    // Fields
    public Vector3 translation;
    public Quaternion rotation;

    // Properties
    public Vector3 Translation
    {
        get { return translation; }
        set { translation = value; }
    }

    public Quaternion Rotation
    {
        get { return rotation; }
        set { rotation = value; }
    }

    // Constructor
    public AffineTransform(Vector3 t, Quaternion r)
    {
        translation = t;
        rotation = r;
    }

}

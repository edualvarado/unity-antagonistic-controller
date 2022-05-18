/****************************************************
 * File: SafetyRegion.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegion : MonoBehaviour
{
    #region Instance Fields

    [Header("Safety Region - Settings INTERACTIVE")]
    public Transform originRegion;
    public Vector3 originOffset;
    public float radiusRegion;
    [Range(0.1f, 2f)] public float reactionTime;

    #endregion
}

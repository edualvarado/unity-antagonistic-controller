using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ImitateHands : MonoBehaviour
{
    [Header("Others")]
    public bool copyKinematicHands;
    public Transform[] kinematicRightHandBones;
    public Transform[] ragdollRightHandBones;
    public Transform[] kinematicLeftHandBones;
    public Transform[] ragdollLeftHandBones;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (copyKinematicHands)
        {
            for (int i = 0; i < kinematicLeftHandBones.Length; i++)
            {
                ragdollLeftHandBones[i].localRotation = kinematicLeftHandBones[i].localRotation;
                ragdollRightHandBones[i].localRotation = kinematicRightHandBones[i].localRotation;
            }
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DetectHit : MonoBehaviour
{
    SetSkeletonsV3 setSkeletonsV3;

    public Transform rightShoulderKin;
    public Transform leftShoulderKin;

    public Transform rightShoulderPhy;
    public Transform leftShoulderPhy;

    //public AnchorPoint anchor;
    public enum AnchorPoint
    {
        Hips,
        //Spine,
        Chest,
        UpperChest,
        //Neck,
        Head,

        LeftShoulder,
        LeftUpperArm,
        LeftLowerArm,

        RightShoulder,
        RightUpperArm,
        RightLowerArm,
    }

    private void Start()
    {
        setSkeletonsV3 = GameObject.Find("ybot_rig_arms_v6_split_interpolated - v3").GetComponent<SetSkeletonsV3>();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.CompareTag("Dynamic Obstacle"))
        {
            Debug.Log("NEW - i was hit");
            setSkeletonsV3.anchor = SetSkeletonsV3.AnchorPoint.RightLowerArm;
            setSkeletonsV3.anchor = SetSkeletonsV3.AnchorPoint.UpperChest;

        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PrintLocalPositionLimbs : MonoBehaviour
{
    public Transform hips;
    public Transform upperLeg;
    public Transform lowerLeg;
    public Transform abdomen;
    public Transform chest;
    public Transform shoulder;
    public Transform upperArm;
    public Transform foreArm;
    public Transform hand;
    public Transform head;

    // Start is called before the first frame update
    void Start()
    {


    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log("hips: " + (hips.position - this.transform.position).ToString("F2"));
        Debug.Log("upperLeg: " + (upperLeg.position - this.transform.position).ToString("F2"));
        Debug.Log("lowerLeg " + (lowerLeg.position - this.transform.position).ToString("F2"));
        Debug.Log("abdomen: " + (abdomen.position - this.transform.position).ToString("F2"));
        Debug.Log("chest: " + (chest.position - this.transform.position).ToString("F2"));
        Debug.Log("shoulder: " + (shoulder.position - this.transform.position).ToString("F2"));
        Debug.Log("upperArm: " + (upperArm.position - this.transform.position).ToString("F2"));
        Debug.Log("foreArm: " + (foreArm.position - this.transform.position).ToString("F2"));
        Debug.Log("hand: " + (hand.position - this.transform.position).ToString("F2"));
        Debug.Log("head: " + (head.position - this.transform.position).ToString("F2"));

    }
}

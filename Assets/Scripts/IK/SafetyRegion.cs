using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafetyRegion : MonoBehaviour
{

    public bool activateIK = false;

    public FastIKFabric leftHand;
    public FastIKFabric rightHand;

    private CapsuleCollider colliderSafetyRegion;

    // Start is called before the first frame update
    void Start()
    {
        colliderSafetyRegion = GetComponent<CapsuleCollider>();
    }

    // Update is called once per frame
    void Update()
    {
        leftHand.activateIK = activateIK;
        rightHand.activateIK = activateIK;
    }

    //Upon collision with another GameObject, this GameObject will reverse direction
    private void OnTriggerStay(Collider other)
    {
        if(other.CompareTag("Obstacle"))
        {
            //leftHand.activateIK = true;
            //leftHand.SetTarget(other.transform);
            Debug.Log("other: " + other.transform.position);
        }
        else
        {
            //leftHand.activateIK = false;

        }
    }
}

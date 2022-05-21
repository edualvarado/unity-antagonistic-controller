using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShotTrigger : MonoBehaviour
{

    public BalancePole2D obstacle;

    public void Start()
    {
        obstacle.targetAngle = -62f;
    }

    private void OnTriggerEnter(Collider other)
    {
        if(other.name != "LeftSafetyRegion")
            obstacle.targetAngle = 0f;
    }
}

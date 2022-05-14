using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Obstacle
{
    public Collider obstacle;
    public Vector3 location;
    public float distance;
    public float realMass;
    public float expectedMass;
    public float realVelocity;
    public float expectedVelocity;

    public Obstacle(Collider obstacle, Vector3 location, float distance, float realMass, float expectedMass, float realVelocity, float expectedVelocity)
    {
        this.obstacle = obstacle;
        this.location = location;
        this.distance = distance;
        this.realMass = realMass;
        this.expectedMass = expectedMass;
        this.realVelocity = realVelocity; // So far, only velocity when it enters the region (does not change when it moves inside). Is it enough?
        this.expectedVelocity = expectedVelocity; // So far, only velocity when it enters the region (does not change when it moves inside). Is it enough?
    }

}

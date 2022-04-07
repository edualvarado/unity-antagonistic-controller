using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Obstacle
{
    public Collider obstacle;
    public Vector3 location;
    public float distance;
    public float mass;

    public Obstacle(Collider obstacle, Vector3 location, float distance, float mass)
    {
        this.obstacle = obstacle;
        this.location = location;
        this.distance = distance;
        this.mass = mass;
    }

}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstacleDynamics : MonoBehaviour
{
    public float realMass;
    public float expectedMass;

    public float realVelocity;
    public float expectedVelocity;

    public bool setSameMass;
    public bool setSameVelocity;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        realMass = GetComponent<Rigidbody>().mass;

        realVelocity = GetComponent<Rigidbody>().velocity.sqrMagnitude;

        if (setSameMass)
            expectedMass = realMass;

        if (setSameVelocity)
            expectedVelocity = realVelocity;
    }
}

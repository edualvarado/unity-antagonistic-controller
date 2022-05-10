using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstacleMass : MonoBehaviour
{

    public float realMass;
    public float expectedMass;

    public bool setSameMass;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        realMass = GetComponent<Rigidbody>().mass;

        if (setSameMass)
            expectedMass = realMass;
    }
}

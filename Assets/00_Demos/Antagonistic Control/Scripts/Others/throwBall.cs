using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class throwBall : MonoBehaviour
{

    public GameObject prefab;
    public float frequency = 1f;
    public Vector3 force;
    public float accT;
    public float accTDestroy;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        accT += Time.deltaTime;
        accTDestroy += Time.deltaTime;

        if (accT > frequency)
        {
            GameObject ball = Instantiate(prefab, this.transform);
            ball.GetComponent<Rigidbody>().AddRelativeForce(new Vector3(force.x, force.y, force.z), ForceMode.Impulse);
            accT = 0f;
        }

    }
}

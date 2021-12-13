using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class throwBall : MonoBehaviour
{

    public GameObject prefab;
    public float f = 1f;
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

        if (accT > f)
        {
            GameObject ball = Instantiate(prefab, this.transform);
            accT = 0f;
        }
    }
}

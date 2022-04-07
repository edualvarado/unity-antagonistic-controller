using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveHangingBall : MonoBehaviour
{

    public Vector3 speed;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(transform.position.x + (speed.x * Time.deltaTime), transform.position.y + (speed.y * Time.deltaTime), transform.position.z + (speed.z * Time.deltaTime));
    }
}

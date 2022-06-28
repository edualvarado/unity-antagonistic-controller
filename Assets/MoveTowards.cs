using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveTowards : MonoBehaviour
{
    public float speed = 3;
    public Vector3 target = Vector3.zero;
    private Vector3 origin;

    void Start()
    {
        origin = transform.position;
    }

    void Update()
    {
        transform.position = Vector3.MoveTowards(transform.position, target, speed * Time.deltaTime);
        //if (transform.position == target) target = origin;
    }
}

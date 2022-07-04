using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MimicQuaternion : MonoBehaviour
{
    public Transform mimic;
    public Vector3 offset;

    public bool worldOffset;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (!worldOffset)
            transform.rotation = mimic.rotation * Quaternion.Euler(offset);
        else
            transform.rotation = Quaternion.Euler(offset) * mimic.rotation;

        float angle = 0f;
        Vector3 axis = Vector3.zero;

        transform.rotation.ToAngleAxis(out angle, out axis);
        Debug.DrawRay(transform.position, axis, Color.red);

    }
}

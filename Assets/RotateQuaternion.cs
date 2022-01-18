using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateQuaternion : MonoBehaviour
{
    public Transform initialTransform;
    public Transform targetTransform;
    public Quaternion rot;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        rot = Quaternion.FromToRotation(initialTransform.forward, targetTransform.forward);
        transform.rotation = transform.rotation * rot;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BasicQuaternions : MonoBehaviour
{

    public Transform target;
    public float angle;
    public Quaternion eulerRotation;

    public Transform child;
    public Quaternion childRotation;

    public Quaternion FromAToB;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Quaternion.LookRotation
        Vector3 relativePos = target.position - transform.position;
        Quaternion rotation = Quaternion.LookRotation(relativePos, Vector3.up);
        //transform.rotation = rotation;
        //transform.rotation = Quaternion.Slerp(transform.rotation, rotation, Time.deltaTime);

        // Quaternion.Angle()
        angle = Quaternion.Angle(transform.rotation, target.rotation);

        // Quaternion.Euler()
        eulerRotation = Quaternion.Euler(0, 30f, 0f);
        //transform.rotation = Quaternion.Slerp(transform.rotation, eulerRotation, Time.deltaTime);

        // Quaternion.FromToRotation()
        Quaternion fromToRotation = Quaternion.FromToRotation(Vector3.up, target.forward);
        //transform.rotation = Quaternion.Slerp(transform.rotation, fromToRotation, Time.deltaTime);

        // Conjugate
        Quaternion qConjugate = new Quaternion(-transform.rotation.x, -transform.rotation.y, -transform.rotation.z, transform.rotation.w);
        //Quaternion qRotate = target.rotation * qConjugate; // If unit length (like quaternion for rotations) then conjugate is the inverse.
        Quaternion qRotate = target.rotation * Quaternion.Inverse(transform.rotation); // If unit length (like quaternion for rotations) then conjugate is the inverse.
        transform.rotation = Quaternion.Slerp(transform.rotation, qRotate, Time.deltaTime);

        // Child Global Rotation
        childRotation = transform.rotation * child.localRotation;

        // Inverse
        FromAToB = Quaternion.Inverse(target.rotation) * childRotation;

    }
}

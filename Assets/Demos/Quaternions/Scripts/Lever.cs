using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lever : MonoBehaviour
{
    public Transform cube; 

    public Transform lever0Base;
    public Transform cube0;
    public Transform lever0;

    public Transform lever1Base;
    public Transform cube1;
    public Transform lever1;

    public Transform lever2Base;
    public Transform cube2;
    public Transform lever2;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //Quaternion offset1 = Quaternion.Inverse(cube1.rotation) * lever1.rotation; // local child = inv(global parent) * global child
        Quaternion offset1 = lever1.rotation * Quaternion.Inverse(cube1.rotation);
        transform.rotation = cube2.rotation * offset1; // global child = global parent * local child

        //Quaternion offset2 = Quaternion.Inverse(cube0.rotation) * Quaternion.Inverse(cube1.rotation) * lever2.rotation; // local child = inv(global parent) * global child
        //cube.rotation = cube0.rotation * cube1.rotation * offset2; // global child = global parent * local child

    }
}

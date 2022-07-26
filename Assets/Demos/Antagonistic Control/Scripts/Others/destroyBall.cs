using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class destroyBall : MonoBehaviour
{

    private void OnCollisionEnter(Collision other)
    {
       if(other.gameObject.CompareTag("Ground"))
        {
            Destroy(this.gameObject);
        }
                
    }
}

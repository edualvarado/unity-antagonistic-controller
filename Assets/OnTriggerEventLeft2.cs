using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class OnTriggerEventLeft2 : MonoBehaviour
{
    public UnityEvent<Collider> onTriggerEnter;
    public UnityEvent<Collider> onTriggerStay;
    public UnityEvent<Collider> onTriggerExit;

    void OnTriggerEnter(Collider col)
    {
        if (onTriggerEnter != null) onTriggerEnter.Invoke(col);
    }

    void OnTriggerStay(Collider col)
    {
        if (onTriggerStay != null) onTriggerStay.Invoke(col);
    }
    void OnTriggerExit(Collider col)
    {
        if (onTriggerExit != null) onTriggerExit.Invoke(col);
    }
}

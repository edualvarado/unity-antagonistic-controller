using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Swing : MonoBehaviour
{
    public float startAngle = -45f;
    public float targetAngle = 45f;

    public float speed = 1.5f;

    Quaternion qStart, qEnd;

    void Start()
    {
        qStart = Quaternion.AngleAxis(startAngle, Vector3.right);
        qEnd = Quaternion.AngleAxis(-startAngle, Vector3.right);
    }

    void Update()
    {
        transform.rotation = Quaternion.Euler(new Vector3(Mathf.Lerp(startAngle, targetAngle, (Mathf.Sin(Time.time * speed) + 1.0f) / 2.0f), 0f, 0f));
    }
}

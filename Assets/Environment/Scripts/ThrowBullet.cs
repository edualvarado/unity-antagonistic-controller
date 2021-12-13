using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ThrowBullet : MonoBehaviour
{
    public GameObject bullet;
    float elapsed = 0f;
    float elapsed2 = 0f;

    public bool stopBullet = false;
    public float forceBullet = 1f;
    public float massBullet = 1f;

    // Start is called before the first frame update
    void Start()
    {

    }

    void Update()
    {
        if (!stopBullet)
        {
            elapsed += Time.deltaTime;
            if (elapsed >= 1f)
            {
                elapsed = elapsed % 1f;
                Shoot();
            }
        }
    }

    void Shoot()
    {
        GameObject proj = Instantiate(bullet);
        proj.GetComponent<Rigidbody>().mass = massBullet;
        proj.transform.position = new Vector3(this.transform.position.x, this.transform.position.y + 0.1f, this.transform.position.z);
        proj.GetComponent<Rigidbody>().AddForce(transform.forward * forceBullet, ForceMode.Impulse);
        elapsed2 += Time.deltaTime;

        if (elapsed2 >= 5f)
        {
            elapsed2 = elapsed2 % 5f;
            Destroy(proj);
        }
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class bounder : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }
	public float force = 10f;
	public float forceOffset = 0.1f;
	void OnCollisionStay(Collision collision)
	{
		if (collision.gameObject.tag == "bound")
		{

			ContactPoint contact = collision.contacts[0];
			MeshDeformer deformer = collision.gameObject.GetComponent<MeshDeformer>();
			if (deformer)
			{
				Debug.Log(deformer.gameObject.name);
				Vector3 point = contact.point;
				point += contact.normal * forceOffset;
				deformer.AddDeformingForce(point, force);
			}
		}
	}
	// Update is called once per frame
	void Update()
    {
        
    }
}

using UnityEngine;

public static class ConfigurableJointExtensions
{
	/// <summary>
	/// Sets a joint's targetRotation to match a given local rotation.
	/// The joint transform's local rotation must be cached on Start and passed into this method.
	/// </summary>
	public static void SetTargetRotationLocal(this ConfigurableJoint joint, Quaternion targetLocalRotation, Quaternion startLocalRotation)
	{
		if (joint.configuredInWorldSpace)
		{
			Debug.LogError("SetTargetRotationLocal should not be used with joints that are configured in world space. For world space joints, use SetTargetRotation.", joint);
		}
		SetTargetRotationInternal(joint, targetLocalRotation, startLocalRotation, Space.Self);
	}

	/// <summary>
	/// Sets a joint's targetRotation to match a given world rotation.
	/// The joint transform's world rotation must be cached on Start and passed into this method.
	/// </summary>
	public static void SetTargetRotation(this ConfigurableJoint joint, Quaternion targetWorldRotation, Quaternion startWorldRotation)
	{
		if (!joint.configuredInWorldSpace)
		{
			Debug.LogError("SetTargetRotation must be used with joints that are configured in world space. For local space joints, use SetTargetRotationLocal.", joint);
		}
		SetTargetRotationInternal(joint, targetWorldRotation, startWorldRotation, Space.World);
	}

	/// <summary>
	/// Gets a joint's targetRotation to match a given local rotation.
	/// The joint transform's local rotation must be cached on Start and passed into this method.
	/// </summary>
	public static Quaternion GetTargetRotationLocal(this ConfigurableJoint joint, Quaternion targetLocalRotation, Quaternion startLocalRotation, Quaternion currentLocalRotation, Transform transform)
	{
		if (joint.configuredInWorldSpace)
		{
			Debug.LogError("SetTargetRotationLocal should not be used with joints that are configured in world space. For world space joints, use SetTargetRotation.", joint);
		}
		return GetTargetRotationInternal(joint, targetLocalRotation, startLocalRotation, currentLocalRotation, Space.Self, transform);
	}

	/// <summary>
	/// Gets a joint's targetRotation to match a given world rotation.
	/// The joint transform's world rotation must be cached on Start and passed into this method.
	/// </summary>
	public static Quaternion GetTargetRotation(this ConfigurableJoint joint, Quaternion targetWorldRotation, Quaternion startWorldRotation, Quaternion currentWorldRotation, Transform transform)
	{
		if (!joint.configuredInWorldSpace)
		{
			Debug.LogError("SetTargetRotation must be used with joints that are configured in world space. For local space joints, use SetTargetRotationLocal.", joint);
		}
		return GetTargetRotationInternal(joint, targetWorldRotation, startWorldRotation, currentWorldRotation, Space.World, transform);
	}

	//----

	static void SetTargetRotationInternal(ConfigurableJoint joint, Quaternion targetRotation, Quaternion startRotation, Space space)
	{
        // Calculate the rotation expressed by the joint's axis and secondary axis
        var right = joint.axis;
        var forward = Vector3.Cross(joint.axis, joint.secondaryAxis).normalized;
        var up = Vector3.Cross(forward, right).normalized;
        Quaternion worldToJointSpace = Quaternion.LookRotation(forward, up);

		// Transform into world space
		Quaternion resultRotation = Quaternion.Inverse(worldToJointSpace);

		// Counter-rotate and apply the new local rotation.
		// Joint space is the inverse of world space, so we need to invert our value
		if (space == Space.World)
		{
			resultRotation *= startRotation * Quaternion.Inverse(targetRotation);
		}
		else
		{
			resultRotation *= Quaternion.Inverse(targetRotation) * startRotation;
		}

		// Transform back into joint space
		resultRotation *= worldToJointSpace;

		// Set target rotation to our newly calculated rotation
		joint.targetRotation = resultRotation;
	}

	static Quaternion GetTargetRotationInternal(ConfigurableJoint joint, Quaternion targetRotation, Quaternion startRotation, Quaternion currentRotation, Space space, Transform transform)
	{
        // Calculate the rotation expressed by the joint's axis and secondary axis <- I think I should not change this.
        var right = joint.axis;
        var forward = Vector3.Cross(joint.axis, joint.secondaryAxis).normalized;
        var up = Vector3.Cross(forward, right).normalized;
        Quaternion worldToJointSpace = Quaternion.LookRotation(forward, up);

		//Debug.DrawRay(transform.position, right, Color.red);
		//Debug.DrawRay(transform.position, forward, Color.blue);
		//Debug.DrawRay(transform.position, up, Color.green);

		// This is the joint space for the right upper arm
		//var right = joint.secondaryAxis;
		//var up = Vector3.Cross(joint.secondaryAxis, joint.axis).normalized;
		//var forward = Vector3.Cross(right, up).normalized;
		//Quaternion worldToJointSpace = Quaternion.LookRotation(forward, up);

		// Transform into world space
		Quaternion resultRotation = Quaternion.Inverse(worldToJointSpace);

		// Counter-rotate and apply the new local rotation.
		// Joint space is the inverse of world space, so we need to invert our value
		if (space == Space.World)
		{
			resultRotation *= startRotation * Quaternion.Inverse(targetRotation);
		}
		else
		{
			resultRotation *= Quaternion.Inverse(targetRotation) * startRotation;
		}

		// Transform back into joint space
		resultRotation *= worldToJointSpace;

		// Return the rotation that needs to be in the physical arm to follow the kinematic model
		return resultRotation;
	}

	//----

	/// <summary>
	/// Adjust ConfigurableJoint settings to closely match CharacterJoint behaviour
	/// </summary>
	public static void SetupAsCharacterJoint(this ConfigurableJoint joint)
	{
		joint.xMotion = ConfigurableJointMotion.Locked;
		joint.yMotion = ConfigurableJointMotion.Locked;
		joint.zMotion = ConfigurableJointMotion.Locked;
		joint.angularXMotion = ConfigurableJointMotion.Limited;
		joint.angularYMotion = ConfigurableJointMotion.Limited;
		joint.angularZMotion = ConfigurableJointMotion.Limited;
		joint.breakForce = Mathf.Infinity;
		joint.breakTorque = Mathf.Infinity;

		joint.rotationDriveMode = RotationDriveMode.Slerp;
		var slerpDrive = joint.slerpDrive;
		slerpDrive.mode = JointDriveMode.Position;
		slerpDrive.maximumForce = Mathf.Infinity;
		joint.slerpDrive = slerpDrive;
	}
}
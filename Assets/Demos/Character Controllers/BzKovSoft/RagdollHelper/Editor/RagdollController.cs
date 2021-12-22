using System;
using UnityEditor;
using UnityEngine;

namespace BzKovSoft.RagdollHelper.Editor
{
	class RagdollController
	{
		GameObject _go;
		Func<Vector3> _getPlayerDirection;
		RagdollProperties _ragdollProperties = new RagdollProperties
		{
			asTrigger = false,
			isKinematic = false,
			rigidAngularDrag = 0.3f,
			rigidDrag = 0.3f
		};
		int _ragdollTotalWeight = 60;           // weight of character (by default 60)

		public RagdollController(GameObject go, Func<Vector3> getPlayerDirection)
		{
			_go = go;
			_getPlayerDirection = getPlayerDirection;
		}

		public void DrawRagdollPanel(bool humanoidSelected)
		{
			GUILayout.BeginVertical("box");
			
			if (humanoidSelected)
			{
				GUILayout.Label("Ragdoll:");
				if (GUILayout.Button("Create"))
					CreateRagdoll();
				if (GUILayout.Button("Remove"))
					RemoveRagdoll();
				//if (GUILayout.Button("Make Profi"))
				//	ConvertToSmartRagdoll();

				_ragdollTotalWeight = EditorGUILayout.IntField("Total Weight:", _ragdollTotalWeight);

				_ragdollProperties.Draw();
			}
			else
			{
				GUILayout.Label("Ragdoll creator supported only for humanoids");
			}

			GUILayout.EndVertical();
		}

		/// <summary>
		/// Remove all colliders, joints, and rigids from "_go" object
		/// </summary>
		void RemoveRagdoll()
		{
			Ragdoller ragdoller = new Ragdoller(_go.transform, _getPlayerDirection());
			ragdoller.ClearRagdoll();
		}

		/// <summary>
		/// Create Ragdoll components on _go object
		/// </summary>
		void CreateRagdoll()
		{
			Ragdoller ragdoller = new Ragdoller(_go.transform, _getPlayerDirection());
			ragdoller.ApplyRagdoll(_ragdollTotalWeight, _ragdollProperties);
		}

		//void ConvertToSmartRagdoll()
		//{
		//	Ragdoller ragdoller = new Ragdoller(_go.transform, _getPlayerDirection());
		//	ragdoller.ClearRagdoll();
		//}
	}
}
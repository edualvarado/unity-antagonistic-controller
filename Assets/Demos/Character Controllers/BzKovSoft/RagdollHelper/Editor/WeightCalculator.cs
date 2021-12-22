using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BzKovSoft.RagdollHelper.Editor
{
	/// <summary>
	/// Calculates weight of each character's bone
	/// </summary>
	struct WeightCalculator
	{
		public readonly float Pelvis;
		public readonly float Hip;
		public readonly float Knee;
		public readonly float Foot;
		public readonly float Arm;
		public readonly float Elbow;
		public readonly float Hand;
		public readonly float Chest;
		public readonly float Head;

		public WeightCalculator(float totalWeight, bool withTips)
		{
			Pelvis = totalWeight * 0.20f;
			Chest = totalWeight * 0.20f;
			Head = totalWeight * 0.05f;

			if (withTips)
			{
				Hip = totalWeight * 0.20f / 2f;
				Knee = totalWeight * 0.15f / 2f;
				Foot = totalWeight * 0.05f / 2f;

				Arm = totalWeight * 0.08f / 2f;
				Elbow = totalWeight * 0.05f / 2f;
				Hand = totalWeight * 0.02f / 2f;
			}
			else
			{
				Hip = totalWeight * 0.20f / 2f;
				Knee = totalWeight * 0.20f / 2f;
				Foot = 0f;

				Arm = totalWeight * 0.08f / 2f;
				Elbow = totalWeight * 0.07f / 2f;
				Hand = 0f;
			}

			float checkSum =
				Pelvis +
				Hip * 2f +
				Knee * 2f +
				Foot * 2f +
				Arm * 2f +
				Elbow * 2f +
				Hand * 2f +
				Chest +
				Head;
			if (Mathf.Abs(totalWeight - checkSum) > Mathf.Epsilon)
				Debug.LogError("totalWeight != checkSum (" + totalWeight.ToString() + ", " + checkSum.ToString() + ")");
		}
	}
}
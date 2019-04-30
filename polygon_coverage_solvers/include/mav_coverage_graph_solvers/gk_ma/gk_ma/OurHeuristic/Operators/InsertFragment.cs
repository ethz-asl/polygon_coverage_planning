using System;
using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	internal class InsertFragment : IRandomMutation
	{
		public Tour Run(Tour source)
		{
			int[] values = source.GetValues();
			int M = source.Task.ClusterCount;

			int rotation = Helper.RandIntLess(M);
			values = Tour.Rotate(values, rotation);

			int len = (int)(M * Helper.Rand(0.05f, 0.3f));
			int newPos = Helper.RandIntLess(M - len) + 1;

			int[] newValues = new int[M];
			Array.Copy(values, len, newValues, 0, newPos);
			Array.Copy(values, 0, newValues, newPos, len);
			Array.Copy(values, newPos + len, newValues, newPos + len, M - len - newPos);

			Tour result = new Tour(source.Task, newValues);
			return result;
		}
	}
}
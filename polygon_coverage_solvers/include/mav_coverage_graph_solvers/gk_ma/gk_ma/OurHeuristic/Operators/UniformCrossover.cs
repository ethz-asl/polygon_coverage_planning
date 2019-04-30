using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	internal class UniformCrossover : ICrossover
	{
		public int[] Cross(Tour parent1, Tour parent2)
		{
			int[] result = new int[parent1.Task.ClusterCount];

			int point1 = Helper.RandIntLess(parent1.Task.ClusterCount);
			int point2;
			do
			{
				point2 = Helper.RandIntLess(parent1.Task.ClusterCount);
			} while (point1 == point2);

			int sourcePosition1 = point1;
			int sourcePosition2 = point1;
			bool[] exists = new bool[parent1.Task.ClusterCount];
			for (int i = 0; i < parent1.Task.ClusterCount; i++)
			{
				while (exists[parent1.GetCluster(sourcePosition1)])
					sourcePosition1 = parent1.NextPos(sourcePosition1);

				result[i] = parent1[sourcePosition1];
				exists[parent1.GetCluster(sourcePosition1)] = true;

				if (i == parent1.RestoreIndex(point2 - point1))
					//if (Helper.RandIntLess(100) < percent)
				{
					Tour tempParent = parent1;
					parent1 = parent2;
					parent2 = tempParent;

					int tempSourcePosition = sourcePosition1;
					sourcePosition1 = sourcePosition2;
					sourcePosition2 = tempSourcePosition;
				}
			}

			return result;
		}

		///<summary>
		///Returns a <see cref="T:System.String"></see> that represents the current <see cref="T:System.Object"></see>.
		///</summary>
		///
		///<returns>
		///A <see cref="T:System.String"></see> that represents the current <see cref="T:System.Object"></see>.
		///</returns>
		///<filterpriority>2</filterpriority>
		public override string ToString()
		{
			return string.Format("uniform");
		}
	}
}
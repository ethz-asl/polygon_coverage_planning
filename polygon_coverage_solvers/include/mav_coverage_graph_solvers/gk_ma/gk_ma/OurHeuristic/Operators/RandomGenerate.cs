using GkMa.Loader;
using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	public class RandomGenerate : Generate
	{
		public RandomGenerate(Task task) : base(task)
		{
		}

		protected override int[] GenerateKeyPure()
		{
			Permutation permutation = Permutation.GenerateRandom(Task.ClusterCount);
			int[] result = new int[Task.ClusterCount];
			for (int i = 0; i < Task.ClusterCount; i++)
				result[i] = Task[permutation[i]][Helper.RandInt(0, Task[permutation[i]].Length - 1)];

			NativeHelper.ClusterOptimisation(result);

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
			return "random";
		}
	}
}
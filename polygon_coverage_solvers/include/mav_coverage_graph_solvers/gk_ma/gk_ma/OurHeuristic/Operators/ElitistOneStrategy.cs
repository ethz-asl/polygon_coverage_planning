using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	internal class ElitistOneStrategy : IOneElementStrategy
	{
		private readonly double max;

		public ElitistOneStrategy(double max)
		{
			this.max = max;
		}

		public void Run(Generation generation, out int index)
		{
			int maxIndex = (int)(generation.Size * max);
			index = (int)(Helper.Sqr(Helper.Rand(0, 1)) * maxIndex);
		}
	}
}
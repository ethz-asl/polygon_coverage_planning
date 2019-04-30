using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	internal class ElitistTwoStrategy : ITwoElementsStrategy
	{
		private readonly float part;

		public ElitistTwoStrategy(float part)
		{
			this.part = part;
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
			return string.Format("elitist {0}", part);
		}

		public void Run(Generation generation, out int index1, out int index2)
		{
			int n = (int)(generation.Size * part);

			index1 = Helper.RandIntLess(0, n);
			do
			{
				index2 = Helper.RandIntLess(0, generation.Size);
			} while (index1 == index2);
		}
	}
}
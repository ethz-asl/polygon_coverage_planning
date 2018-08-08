namespace GkMa.OurHeuristic.Types
{
	public enum OperatorType
	{
		Generation,
		Crossover,
		Mutation,
		Clone,
		Improvement,
	}

	public class OperatorInfo
	{
		private readonly Tour[] parents;
		private readonly string action;
		private readonly int ticks;


		private readonly OperatorType operatorType;

		public OperatorType OperatorType
		{
			get { return operatorType; }
		}

		public OperatorInfo(OperatorType operatorType, string action, long ticks, params Tour[] parents)
		{
			this.parents = parents;
			this.action = action;
			this.ticks = (int)ticks;
			this.operatorType = operatorType;
		}

		public Tour[] Parents
		{
			get { return parents; }
		}

		public string Action
		{
			get { return action; }
		}

		public int Ticks
		{
			get { return ticks; }
		}
	}
}
using GkMa.Loader;
using GkMa.OurHeuristic;

namespace GkMa
{
	public class OurSolver : Solver
	{
		private Algorithm solver;
		private int generationCount;

		public OurSolver(string problemFileName, bool binary)
			: this(new Task(problemFileName, binary))
		{
		}

		public OurSolver(Task task) : base(task)
		{
		}

        public OurSolver(int[][] m, int[][] clusters, bool isSymmetric) : base(m, clusters, isSymmetric) {}

		public int GenerationCount
		{
			get { return generationCount; }
		}

		public override void Solve()
		{
			solver = new Algorithm(Task.Rebuild(int.MaxValue / Task.ClusterCount));
			solver.Run();
			Len = solver.LastGeneration[0].Length;
			Milliseconds = (int)solver.TotalTime.TotalMilliseconds;
            generationCount = solver.GenerationCount;
            Solution = solver.LastGeneration[0].GetValues();
        }

        public int SolutionLength
        {
            get { return solution.Length; }
        }

        public int SolutionAtIndex(int index)
        {
            return solution[index];
        }
	}
}
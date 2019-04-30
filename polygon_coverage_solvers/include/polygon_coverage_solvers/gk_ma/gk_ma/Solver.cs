using GkMa.Loader;

namespace GkMa
{
	public abstract class Solver
	{
		private readonly Task task;
		private readonly string problemFileName;
        private int milliseconds;
        protected int[] solution;

		public int Milliseconds
		{
			get { return milliseconds; }
			protected set { milliseconds = value; }
		}

		private int len;

		public abstract void Solve();

		public int Len
		{
			get { return len; }
			protected set { len = value; }
		}

        public int[] Solution
        {
            get { return solution; }
            protected set { solution = value; }
        }

		protected Solver(Task task)
		{
			this.task = task;
        }

        protected Solver(int[][] m, int[][] clusters, bool isSymmetric)
        {
            this.task = new Task(m, clusters, isSymmetric);
        }

		protected bool TaskIsInFile
		{
			get { return problemFileName != null; }
		}


		public Task Task
		{
			get { return task; }
		}

		public string ProblemFileName
		{
			get { return problemFileName; }
		}

		protected Solver(string problemFileName)
		{
			this.problemFileName = problemFileName;
		}
	}
}
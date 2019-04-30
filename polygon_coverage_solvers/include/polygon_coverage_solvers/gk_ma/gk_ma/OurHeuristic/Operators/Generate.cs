using GkMa.Loader;

namespace GkMa.OurHeuristic.Operators
{
	public abstract class Generate
	{
		private readonly Task task;

		public Task Task
		{
			get { return task; }
		}

		protected Generate(Task task)
		{
			this.task = task;
		}

		protected int RandomElement()
		{
			return Helper.RandIntLess(0, Task.VertexCount);
		}

		protected abstract int[] GenerateKeyPure();

		public int[] GenerateKey()
		{
			return GenerateKeyPure();
		}
	}
}
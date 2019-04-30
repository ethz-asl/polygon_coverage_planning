namespace GkMa.Loader
{
	public struct Vertex
	{
		private readonly float x, y, z;

		public Vertex(float x, float y, float z)
		{
			this.x = x;
			this.y = y;
			this.z = z;
		}

		public Vertex(float x, float y)
			: this(x, y, 0)
		{
		}

		public float X
		{
			get { return x; }
		}

		public float Y
		{
			get { return y; }
		}

		public float Z
		{
			get { return z; }
		}
	}
}
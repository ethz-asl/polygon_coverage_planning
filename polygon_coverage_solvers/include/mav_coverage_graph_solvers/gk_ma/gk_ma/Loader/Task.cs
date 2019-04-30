using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace GkMa.Loader
{
	public class Task
	{
		private int[][] m;
		private int[][] clusters;
		private readonly Point[] vertexes;
		private Rectangle bounds = new Rectangle(0, 0, -1, -1);
		private int bestSolution;
		private bool isSymmetric;
		private bool isTriangle;
		private VertexInfo[] verticesInfo;

		private struct VertexInfo
		{
			private readonly int clusterIndex;
			private readonly int vertexIndex;

			public VertexInfo(int clusterIndex, int vertexIndex)
			{
				this.clusterIndex = clusterIndex;
				this.vertexIndex = vertexIndex;
			}

			public int ClusterIndex
			{
				get { return clusterIndex; }
			}

			public int VertexIndex
			{
				get { return vertexIndex; }
			}
		}

		public int[][] M
		{
			get { return m; }
		}

		public Task(string fileName, bool binary)
		{
			if (binary)
				using (FileStream stream = new FileStream(fileName, FileMode.Open))
					LoadFromStream(stream);
			else
				using (StreamReader reader = new StreamReader(fileName, Encoding.ASCII))
					LoadFromReader(reader);
		}

		private void LoadFromReader(TextReader reader)
		{
			int n = int.Parse(ReadAttribute(reader, "N"));
			m = new int[n][];
			clusters = new int[int.Parse(ReadAttribute(reader, "M"))][];
			isSymmetric = bool.Parse(ReadAttribute(reader, "Symmetric"));
			isTriangle = bool.Parse(ReadAttribute(reader, "Triangle"));

			for (int i = 0; i < clusters.Length; i++)
			{
				string line = reader.ReadLine();
				string[] split = line.Split(new[] {' '}, StringSplitOptions.RemoveEmptyEntries);
				if (split.Length == 0)
					throw new GtspException("No vertices provided for cluster {0}.", i);

				int count = int.Parse(split[0]);
				if (split.Length != count + 1)
					throw new GtspException("Wrong number of vertices in cluster {0}.", i);

				clusters[i] = new int[count];
				for (int j = 0; j < count; j++)
					clusters[i][j] = int.Parse(split[j + 1]) - 1;
			}

			for (int from = 0; from < n; from++)
			{
				string line = reader.ReadLine();
				string[] split = line.Split(new char[] {' '}, StringSplitOptions.RemoveEmptyEntries);
				if (split.Length != n)
					throw new GtspException("Weight matrix row size is wrong.");
				m[from] = new int[n];
				for (int to = 0; to < split.Length; to++)
					m[from][to] = int.Parse(split[to]);
			}

			UpdateVerticesInfo();
		}

		private void UpdateVerticesInfo()
		{
			verticesInfo = new VertexInfo[VertexCount];
			for (int i = 0; i < clusters.Length; i++)
				for (int j = 0; j < clusters[i].Length; j++)
					verticesInfo[clusters[i][j]] = new VertexInfo(i, j);
		}

		private static int ReadIntFromStream(Stream stream)
		{
			byte[] buf = new byte[4];
			stream.Read(buf, 0, 4);
			return BitConverter.ToInt32(buf, 0);
		}

		private void LoadFromStream(Stream stream)
		{
			int vertexCount = ReadIntFromStream(stream);
			int clusterCount = ReadIntFromStream(stream);
			isSymmetric = ReadIntFromStream(stream) != 0;
			//bestSolution = ReadIntFromStream(stream);

			clusters = new int[clusterCount][];
			for (int cluster = 0; cluster < clusterCount; cluster++)
			{
				int clusterSize = ReadIntFromStream(stream);
				clusters[cluster] = new int[clusterSize];
				for (int vertexinClusterIndex = 0; vertexinClusterIndex < clusterSize; vertexinClusterIndex++)
					clusters[cluster][vertexinClusterIndex] = ReadIntFromStream(stream);
			}

			m = new int[vertexCount][];
			byte[] buf = new byte[vertexCount * 4];
			for (int i = 0; i < vertexCount; i++)
			{
				stream.Read(buf, 0, buf.Length);
				unsafe
				{
					fixed (int* row = m[i] = new int[vertexCount])
						Marshal.Copy(buf, 0, (IntPtr)row, buf.Length);
				}
			}

			UpdateVerticesInfo();
		}

		public Task(int[][] m, int[][] clusters, Point[] vertexes, bool isSymmetric)
		{
			this.m = m;
			this.clusters = clusters;
			this.vertexes = vertexes;
			this.isSymmetric = isSymmetric;
			UpdateVerticesInfo();
#if DEBUG
			if (m.Length != m[0].Length)
				throw new GtspException("The matrix dimesions are not equel: {0} and {1}",
					m.GetLength(0), m.GetLength(1));
			int sum = 0;
			for (int i = 0; i < clusters.Length; i++)
				sum += clusters[i].Length;
			if (sum != VertexCount)
				throw new GtspException("The matrix size and total cluster sizes are different: {0} and {1}",
					VertexCount, sum);
#endif
        }

        public Task(int[][] m, int[][] clusters, bool isSymmetric)
        {
            this.m = m;
            this.clusters = clusters;
            this.isSymmetric = isSymmetric;
            UpdateVerticesInfo();
        }

		public Task(int[,] m, int[][] clusters, Point[] vertexes, bool isSymmetric)
		{
			this.m = new int[m.GetLength(0)][];
			for (int i = 0; i < this.m.Length; i++)
			{
				this.m[i] = new int[this.m.Length];
				for (int j = 0; j < this.m.Length; j++)
				{
					this.m[i][j] = m[i, j];
				}
			}
			this.clusters = clusters;
			this.vertexes = vertexes;
			this.isSymmetric = isSymmetric;
			UpdateVerticesInfo();
#if DEBUG
			if (m.GetLength(0) != m.GetLength(1))
				throw new GtspException("The matrix dimesions are not equel: {0} and {1}",
					m.GetLength(0), m.GetLength(1));
			int sum = 0;
			for (int i = 0; i < clusters.Length; i++)
				sum += clusters[i].Length;
			if (sum != VertexCount)
				throw new GtspException("The matrix size and total cluster sizes are different: {0} and {1}",
					VertexCount, sum);
#endif
		}

		public int VertexCount
		{
			get { return m.Length; }
		}

		public int ClusterCount
		{
			get { return clusters.Length; }
		}

		public int this[int from, int to]
		{
			get { return m[from][to]; }
		}

		public int[] this[int clusterIndex]
		{
			get { return clusters[clusterIndex]; }
		}

		public int GetVertexIndex(int clusterIndex, int vertexInCluster)
		{
			return clusters[clusterIndex][vertexInCluster];
		}

		public Point GetVertex(int clusterIndex, int vertexInCluster)
		{
			return GetVertex(GetVertexIndex(clusterIndex, vertexInCluster));
		}

		public Point GetVertex(int index)
		{
			return vertexes[index];
		}

		public Rectangle Bounds
		{
			get
			{
				BoundsNeeded();
				return bounds;
			}
		}

		private void BoundsNeeded()
		{
			if (bounds.Width < 0)
				UpdateBounds();
		}

		private void UpdateBounds()
		{
			if (vertexes == null)
			{
				bounds = new Rectangle(0, 0, 0, 0);
				return;
			}

			int left = vertexes[0].X, right = vertexes[0].X, top = vertexes[0].Y, bottom = vertexes[0].Y;
			foreach (Point vertex in vertexes)
			{
				if (vertex.X < left)
					left = vertex.X;
				else if (vertex.X > right)
					right = vertex.X;

				if (vertex.Y < top)
					top = vertex.Y;
				else if (vertex.Y > bottom)
					bottom = vertex.Y;
			}
			bounds = new Rectangle(left, top, right - left, bottom - top);
		}


		public int BestSolution
		{
			get { return bestSolution; }
			set { bestSolution = value; }
		}

		public bool ContainsVertexCoords
		{
			get { return vertexes != null; }
		}

		public bool IsSymmetric
		{
			get { return isSymmetric; }
		}

		public bool IsTriangle
		{
			get { return isTriangle; }
		}

		public void SaveToFile(string fileName, bool binary)
		{
			if (binary)
				using (FileStream stream = new FileStream(fileName, FileMode.Create))
					SaveToStream(stream);
			else
				using (StreamWriter writer = new StreamWriter(fileName, false, Encoding.ASCII))
					SaveToWriter(writer);
		}

		private static void WriteAttribute(TextWriter writer, string name, object value)
		{
			writer.WriteLine(string.Format("{0}: {1}", name, value));
		}

		private static string ReadAttribute(TextReader reader, string name)
		{
			string line = reader.ReadLine();
			int pos = line.IndexOf(":");
			string foundName = line.Substring(0, pos).Trim();
			if (foundName != name)
				throw new GtspException("'{0}' attribute expected, '{1}' found", name, foundName);

			return line.Substring(pos + 1).Trim();
		}

		public void SaveToWriter(TextWriter writer)
		{
			WriteAttribute(writer, "N", VertexCount);
			WriteAttribute(writer, "M", ClusterCount);
			WriteAttribute(writer, "Symmetric", IsSymmetric);

			for (int i = 0; i < ClusterCount; i++)
			{
				foreach (int vertex in this[i])
				{
					writer.Write(vertex);
					writer.Write(' ');
				}
				writer.WriteLine();
			}

			for (int from = 0; from < VertexCount; from++)
			{
				for (int to = 0; to < VertexCount; to++)
				{
					writer.Write(this[from, to]);
					writer.Write(' ');
				}
				writer.WriteLine();
			}
		}

		private static void WriteIntToStream(Stream stream, int value)
		{
			byte[] buf = BitConverter.GetBytes(value);
			stream.Write(buf, 0, buf.Length);
		}

		public void SaveToStream(Stream stream)
		{
			WriteIntToStream(stream, VertexCount);
			WriteIntToStream(stream, ClusterCount);
			WriteIntToStream(stream, IsSymmetric ? 1 : 0);
			WriteIntToStream(stream, BestSolution);

			for (int i = 0; i < ClusterCount; i++)
			{
				WriteIntToStream(stream, this[i].Length);
				foreach (int vertex in this[i])
					WriteIntToStream(stream, vertex);
			}

			byte[] buf = new byte[VertexCount * 4];
			for (int from = 0; from < VertexCount; from++)
			{
				unsafe
				{
					fixed (byte* b = buf)
						Marshal.Copy(m[from], 0, (IntPtr)b, VertexCount);
				}

				stream.Write(buf, 0, buf.Length);
			}
		}

		public void CheckInstance()
		{
			bool[] used = new bool[VertexCount];
			for (int cluster = 0; cluster < ClusterCount; cluster++)
			{
				for (int vertexIndex = 0; vertexIndex < this[cluster].Length; vertexIndex++)
				{
					if (used[this[cluster][vertexIndex]])
						throw new GtspException("Vertex is presented several times");

					used[this[cluster][vertexIndex]] = true;
				}
			}

			for (int i = 0; i < VertexCount; i++)
				if (!used[i])
					throw new GtspException("Vertex is not used.");

			if (IsSymmetric)
			{
				for (int from = 0; from < VertexCount; from++)
				{
					for (int to = 0; to < from; to++)
					{
						if (this[from, to] != this[to, from])
							throw new GtspException("No symmetry");
					}
				}
			}
		}

		public void ExcludeVertex(int clusterIndex, int vertexInCluster)
		{
			List<int> list = new List<int>(clusters[clusterIndex]);
			list.RemoveAt(vertexInCluster);
			clusters[clusterIndex] = list.ToArray();
		}

		public void ExcludeEdge(int from, int to)
		{
			m[from][to] = int.MaxValue;
			if (isSymmetric)
				m[to][from] = int.MaxValue;
		}

		public Task Rebuild(int infinite)
		{
			int newVertexCount = 0;
			bool[] used = new bool[VertexCount];
			for (int i = 0; i < ClusterCount; i++)
			{
				newVertexCount += this[i].Length;
				for (int j = 0; j < this[i].Length; j++)
				{
					if (used[this[i][j]])
						throw new GtspException("A vertex is used in several clusters");

					used[this[i][j]] = true;
				}
			}

			int[] newVertexIndices = new int[VertexCount];
			int[] oldVertexIndices = new int[newVertexCount];
			int index = 0;
			for (int i = 0; i < VertexCount; i++)
			{
				if (used[i])
				{
					oldVertexIndices[index] = i;
					newVertexIndices[i] = index++;
				}
			}

			int[][] newClusters = new int[ClusterCount][];
			for (int i = 0; i < ClusterCount; i++)
			{
				newClusters[i] = new int[this[i].Length];
				for (int j = 0; j < this[i].Length; j++)
					newClusters[i][j] = newVertexIndices[this[i][j]];
			}

			int[][] newM = new int[newVertexCount][];
			for (int from = 0; from < newVertexCount; from++)
			{
				newM[from] = new int[newVertexCount];
				for (int to = 0; to < newVertexCount; to++)
				{
					int oldL = this[oldVertexIndices[from], oldVertexIndices[to]];
					if (oldL == int.MaxValue)
						oldL = infinite;
					newM[from][to] = oldL;
				}
			}

			return new Task(newM, newClusters, null, isSymmetric);
		}

		public int EdgesExcluded
		{
			get
			{
				int result = 0;
				for (int from = 0; from < VertexCount; from++)
				{
					for (int to = 0; to < VertexCount; to++)
					{
						if (from == to)
							continue;

						if (this[from, to] == int.MaxValue)
							result++;
					}
				}

				return result;
			}
		}

		public long TotalEdgeCount
		{
			get
			{
				long result = 0;
				for (int fromClusterIndex = 0; fromClusterIndex < ClusterCount; fromClusterIndex++)
					for (int toClusterIndex = fromClusterIndex + 1; toClusterIndex < ClusterCount; toClusterIndex++)
						result += this[fromClusterIndex].Length * this[toClusterIndex].Length;

				return result * 2;
			}
		}

		public int ActualEdgeCount
		{
			get
			{
				int result = 0;
				for (int fromClusterIndex = 0; fromClusterIndex < ClusterCount; fromClusterIndex++)
				{
					for (int fromVertexIndex = 0; fromVertexIndex < this[fromClusterIndex].Length; fromVertexIndex++)
					{
						int fromVertex = this[fromClusterIndex][fromVertexIndex];
						for (int toClusterIndex = fromClusterIndex + 1; toClusterIndex < ClusterCount; toClusterIndex++)
						{
							for (int toVertexIndex = 0; toVertexIndex < this[toClusterIndex].Length; toVertexIndex++)
							{
								int toVertex = this[toClusterIndex][toVertexIndex];

								if (this[fromVertex, toVertex] != int.MaxValue)
									result++;
								if (this[toVertex, fromVertex] != int.MaxValue)
									result++;
							}
						}
					}
				}

				return result;
			}
		}

		public void SaveForFatih(string fileName)
		{
			using (StreamWriter writer = new StreamWriter(fileName, false, Encoding.ASCII))
				SaveForFatih(writer);
		}

		public void SaveForFatih(TextWriter writer)
		{
			writer.WriteLine("{0} {1} {2}", VertexCount, ClusterCount, BestSolution);
			for (int i = 0; i < VertexCount; i++)
			{
				int clusterIndex = -1;
				for (int j = 0; j < ClusterCount; j++)
				{
					if (Array.BinarySearch(this[j], i) >= 0)
					{
						clusterIndex = j;
						break;
					}
				}

				writer.WriteLine("{0} {1} {2}", vertexes[i].Y, vertexes[i].X, clusterIndex + 1);
			}
		}

		public int[] GetWeightsFrom(int fromVertex)
		{
			return m[fromVertex];
		}

		public int GetCluster(int vertex, out int vertexIndex)
		{
			vertexIndex = verticesInfo[vertex].VertexIndex;
			return verticesInfo[vertex].ClusterIndex;
		}

		public int GetCluster(int vertex)
		{
			return verticesInfo[vertex].ClusterIndex;
		}
	}
}
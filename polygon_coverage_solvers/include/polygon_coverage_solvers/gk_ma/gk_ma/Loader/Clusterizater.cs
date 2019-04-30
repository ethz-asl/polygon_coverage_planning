using System.Collections;
using System.Collections.Generic;

namespace GkMa.Loader
{
	public delegate int GetWeightHandler(int v1, int v2);

	internal class Clusterizater
	{
		private readonly int n;
		private readonly int m;
		private readonly GetWeightHandler getWeight;

		private class Set : IEnumerable<int>
		{
			private readonly bool[] belongs;

			public Set(int vertexCount, params int[] include)
			{
				belongs = new bool[vertexCount];
				foreach (int i in include)
					belongs[i] = true;
			}

			public bool this[int index]
			{
				get { return belongs[index]; }
			}

			///<summary>
			///Returns an enumerator that iterates through the collection.
			///</summary>
			///
			///<returns>
			///A <see cref="T:System.Collections.Generic.IEnumerator`1"></see> that can be used to iterate through the collection.
			///</returns>
			///<filterpriority>1</filterpriority>
			IEnumerator<int> IEnumerable<int>.GetEnumerator()
			{
				for (int i = 0; i < belongs.Length; i++)
					if (belongs[i])
						yield return i;
			}

			///<summary>
			///Returns an enumerator that iterates through a collection.
			///</summary>
			///
			///<returns>
			///An <see cref="T:System.Collections.IEnumerator"></see> object that can be used to iterate through the collection.
			///</returns>
			///<filterpriority>2</filterpriority>
			public IEnumerator GetEnumerator()
			{
				return ((IEnumerable<int>)this).GetEnumerator();
			}

			public void Include(int v)
			{
				belongs[v] = true;
			}
		}

		public Clusterizater(int n, int m, GetWeightHandler getWeight)
		{
			this.n = n;
			this.m = m;
			this.getWeight = getWeight;
		}

		public int[][] Clusterize()
		{
			int[] centers = new int[m];
			centers[0] = far(new Set(n, 0));
			Set centersSet = new Set(n, centers[0]);
			for (int i = 1; i < m; i++)
			{
				centers[i] = far(centersSet);
				centersSet.Include(centers[i]);
			}

			List<int>[] clusters = new List<int>[m];
			for (int i = 0; i < m; i++)
				clusters[i] = new List<int>();

			for (int v = 0; v < n; v++)
			{
				int min = int.MaxValue;
				int bestc = -1;
				for (int c = 0; c < m; c++)
				{
					int weight = getWeight(v, centers[c]);
					if (weight < min)
					{
						min = weight;
						bestc = c;
					}
				}

				clusters[bestc].Add(v);
			}

			int[][] result = new int[m][];
			for (int i = 0; i < m; i++)
				result[i] = clusters[i].ToArray();

			return result;
		}

		private int far(Set set)
		{
			int max = int.MinValue;
			int result = -1;
			for (int v = 0; v < n; v++)
			{
				if (set[v])
					continue;

				int min = int.MaxValue;
				foreach (int w in set)
				{
					int weight = getWeight(v, w);
					if (weight < min)
						min = weight;
				}

				if (max < min)
				{
					max = min;
					result = v;
				}
			}

			return result;
		}
	}
}
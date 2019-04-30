using System;
using System.Runtime.InteropServices;
using GkMa.Loader;
using GkMa.OurHeuristic.Operators;

namespace GkMa.OurHeuristic.Types
{
	public class Tour : IComparable<Tour>
	{
		private readonly Task task;
		private int[] vertices;
		private int length = -1;
		private int firstClusterPosition = -1;

		public Task Task
		{
			get { return task; }
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
			return Length.ToString();
		}

		///<summary>
		///Determines whether the specified <see cref="T:System.Object"></see> is equal to the current <see cref="T:System.Object"></see>.
		///</summary>
		///
		///<returns>
		///true if the specified <see cref="T:System.Object"></see> is equal to the current <see cref="T:System.Object"></see>; otherwise, false.
		///</returns>
		///
		///<param name="obj">The <see cref="T:System.Object"></see> to compare with the current <see cref="T:System.Object"></see>. </param><filterpriority>2</filterpriority>
		public override bool Equals(object obj)
		{
			Tour tour = obj as Tour;
			if (tour == null)
				return false;

			if (Length != tour.Length)
				return false;

			int delta = FirstClusterPosition - tour.FirstClusterPosition;
			Tour tour1;
			Tour tour2;
			if (delta < 0)
			{
				delta = -delta;
				tour1 = this;
				tour2 = tour;
			}
			else
			{
				tour1 = tour;
				tour2 = this;
			}

			int tourIndex = delta;
			for (int i = Task.ClusterCount - 1; i >= 0; i--)
			{
				if (tour1[i] != tour2[tourIndex])
					return false;

				tourIndex = PrevPos(tourIndex);
			}

			return true;
		}

		///<summary>
		///Serves as a hash function for a particular type. <see cref="M:System.Object.GetHashCode"></see> is suitable for use in hashing algorithms and data structures like a hash table.
		///</summary>
		///
		///<returns>
		///A hash code for the current <see cref="T:System.Object"></see>.
		///</returns>
		///<filterpriority>2</filterpriority>
		public override int GetHashCode()
		{
			return Length
			       + (this[(45467 + FirstClusterPosition) % task.ClusterCount] << 3)
			       + (this[(89012 + FirstClusterPosition) % task.ClusterCount] << 4)
			       + (this[(47732 + FirstClusterPosition) % task.ClusterCount] << 5)
			       + (this[(67893 + FirstClusterPosition) % task.ClusterCount] << 6);
		}

		///<summary>
		///Compares the current object with another object of the same type.
		///</summary>
		///
		///<returns>
		///A 32-bit signed integer that indicates the relative order of the objects being compared. The return value has the following meanings: Value Meaning Less than zero This object is less than the other parameter.Zero This object is equal to other. Greater than zero This object is greater than other.
		///</returns>
		///
		///<param name="other">An object to compare with this object.</param>
		public int CompareTo(Tour other)
		{
#if DEBUG
			if (Task != other.Task)
				throw new GtspException("Can not compare keys for different tasks");
#endif

			return Length.CompareTo(other.Length);
		}

		public int RestoreIndex(int index)
		{
			if (index < 0)
				return index + Task.ClusterCount;

			if (index >= Task.ClusterCount)
				return index - Task.ClusterCount;

			return index;
		}

		public int PrevPos(int position)
		{
			if (position == 0)
				return Task.ClusterCount - 1;

			return position - 1;
		}

		public int NextPos(int position)
		{
			if (position == Task.ClusterCount - 1)
				return 0;

			return position + 1;
		}

		public TourElement GetElementSafe(int position)
		{
			return GetElement(RestoreIndex(position));
		}

		#region First cluster position

		private void FirstClusterPositionNeeded()
        {
			if (firstClusterPosition >= 0)
				return;

			for (int i = 0; i < Task.ClusterCount; i++)
			{
				if (GetElement(i).ClusterIndex == 0)
				{
					firstClusterPosition = i;
					return;
				}
			}

			throw new GtspException("First cluster position not found");
		}

		public int FirstClusterPosition
		{
			get
			{
				FirstClusterPositionNeeded();
				return firstClusterPosition;
			}
		}

		private void InvalidateFirstClusterPosition()
		{
			firstClusterPosition = -1;
		}

		#endregion

		#region Constructors

		public Tour(Generate generate)
		{
			task = generate.Task;
			vertices = generate.GenerateKey();
			UpdateLength();
		}


		public Tour(Generation generation, ITwoElementsStrategy strategy, ICrossover crossover)
		{
			task = generation[0].Task;

			int index1, index2;
			strategy.Run(generation, out index1, out index2);
			Tour parent1 = generation[index1];
			Tour parent2 = generation[index2];

			parent1.CorrectRotation();
			parent2.CorrectRotation();

			vertices = crossover.Cross(parent1, parent2);
			UpdateLength();
		}

		public Tour(Task task, int[] vertices)
		{
			this.task = task;
			this.vertices = vertices;
			UpdateLength();
		}

		#endregion

		private struct RandomTourItem : IComparable<RandomTourItem>
		{
			public readonly int clusterIndex;
			public readonly int vertexInCluster;
			public readonly float valueFrac;

			public RandomTourItem(int clusterIndex, float value)
			{
				this.clusterIndex = clusterIndex;
				valueFrac = Frac(value);
				vertexInCluster = (int)Math.Truncate(value);
			}

			///<summary>
			///Compares the current object with another object of the same type.
			///</summary>
			///
			///<returns>
			///A 32-bit signed integer that indicates the relative order of the objects being compared. The return value has the following meanings: Value Meaning Less than zero This object is less than the other parameter.Zero This object is equal to other. Greater than zero This object is greater than other.
			///</returns>
			///
			///<param name="other">An object to compare with this object.</param>
			public int CompareTo(RandomTourItem other)
			{
				return valueFrac.CompareTo(other.valueFrac);
			}
		}

/*
		private class ItemCompararer : IComparer<RandomTourItem>
		{

			#region IComparer<Item> Members

			public int Compare(RandomTourItem x, RandomTourItem y)
			{
				return Frac(x.value).CompareTo(Frac(y.value));
			}

			#endregion
		}
*/

		#region Helper function and properties

		public static float Frac(float x)
		{
			return (float)(x - Math.Truncate(x));
		}


		public void UpdateLength()
		{
			length = Task[vertices[vertices.Length - 1], vertices[0]];
			for (int i = 1; i < vertices.Length; i++)
				length += Task[vertices[i - 1], vertices[i]];
		}

		#endregion


		#region Optimization

/*
		public bool Op2Full()
		{
			int bestDelta = 0;
			int bestV1 = -1, bestV2 = -1;
			for (int v1 = 0; v1 < Task.ClusterCount - 1; v1++)
			{
				for (int v2 = v1 + 1; v2 < Task.ClusterCount; v2++)
				{
					int delta = Op2LengthDelta(v1, v2);
					if (delta < bestDelta)
					{
						bestDelta = delta;
						bestV1 = v1;
						bestV2 = v2;
					}
				}
			}
			if (bestV1 < 0)
				return false;

			int oldLength = Length;
			Op2(bestV1, bestV2);
			UpdateLength();
			if (Length != oldLength + bestDelta)
				throw new GtspException("!=");
			return true;
		}
*/

/*
		private int Op2LengthDelta(int v1, int v2)
		{
			int vertexIndex1 = vertices[v1];
			int vertexIndex2 = vertices[v2];

			if (v2 == RestoreIndex(v1 + 1))
			{
				int prevIndex = vertices[PrevPos(v1)];
				int nextIndex = vertices[NextPos(v2)];
				return
					Task[prevIndex, vertexIndex2]
					+ Task[vertexIndex2, vertexIndex1]
					+ Task[vertexIndex1, nextIndex]
					- Task[prevIndex, vertexIndex1]
					- Task[vertexIndex1, vertexIndex2]
					- Task[vertexIndex2, nextIndex];
			}

			if (v1 == NextPos(v2))
				return Op2LengthDelta(v2, v1);

			int prevVertexIndex1 = vertices[PrevPos(v1)];
			int nextVertexIndex1 = vertices[NextPos(v1)];
			int prevVertexIndex2 = vertices[PrevPos(v2)];
			int nextVertexIndex2 = vertices[NextPos(v2)];
			return
				Task[prevVertexIndex1, vertexIndex2]
				+ Task[vertexIndex2, nextVertexIndex1]
				+ Task[prevVertexIndex2, vertexIndex1]
				+ Task[vertexIndex1, nextVertexIndex2]
				- Task[prevVertexIndex1, vertexIndex1]
				- Task[vertexIndex1, nextVertexIndex1]
				- Task[prevVertexIndex2, vertexIndex2]
				- Task[vertexIndex2, nextVertexIndex2];
		}
*/

/*
		public void Op2(int v1, int v2)
		{
			length += Op2LengthDelta(v1, v2);

			RandomTourItem temp = items[v1];
			items[v1] = items[v2];
			items[v2] = temp;

			float frac1 = items[v1].valueFrac;
			float frac2 = items[v2].valueFrac;
			items[v1].valueFrac
				= Frac(values[items[v1].clusterIndex]
				  = items[v1].vertexInCluster + frac2);
			items[v2].valueFrac
				= Frac(values[items[v2].clusterIndex]
				  = items[v2].vertexInCluster + frac1);
		}
*/

/*
		public bool Full2Opt()
		{
			//int bestDl = 0;
			//int bestV1 = -1;
			//int bestV2 = -1;
			LengthNeeded();
			for (int v1 = 0; v1 < Task.ClusterCount - 2; v1++)
				for (int v2 = v1 + 2; v2 < Task.ClusterCount; v2++)
				{
					int curIndex1 = items[v1].GetVertexIndex(Task);
					int nextIndex1 = items[v1 + 1].GetVertexIndex(Task);
					int curIndex2 = items[v2].GetVertexIndex(Task);
					int nextIndex2 = items[RestoreIndex(v2 + 1)].GetVertexIndex(Task);
					int dl =
						Task[curIndex1, curIndex2]
						+ Task[nextIndex1, nextIndex2]
						- Task[curIndex1, nextIndex1]
						- Task[curIndex2, nextIndex2];
					if (dl < 0)
					{
						v1++;
						for (int i = 0; i <= (v2 - v1) / 2; i++)
						{
							int clusterIndex1 = items[v1 + i].clusterIndex;
							int clusterIndex2 = items[v2 - i].clusterIndex;
							int vertexInCluster1 = items[v1 + i].vertexInCluster;
							int vertexInCluster2 = items[v2 - i].vertexInCluster;
							float frac1 = Frac(values[clusterIndex1]);
							float frac2 = Frac(values[clusterIndex2]);
							values[clusterIndex1] = vertexInCluster1 + frac2;
							values[clusterIndex2] = vertexInCluster2 + frac1;
						}
						int oldLength = Length;
						UpdateLength();
						if (Length != oldLength + dl)
						{
							//Console.WriteLine("Full2Opt !=");
							throw new GtspException("!=");
						}
						return true;
					}
				}

			return false;
		}
*/

		/*
				public void Swap(int tourPosition, int newVertexIndexInCluster)
				{
					int clusterIndex = items[tourPosition].clusterIndex;
					values[clusterIndex] = newVertexIndexInCluster + Frac(values[clusterIndex]);
					UpdateLength();
				}
		*/

		/*
				public void Swap()
				{
					int clusterIndex = Helper.RandInt(0, task.ClusterCount - 1);
					int vertexCount = task[clusterIndex].Length;
					int newVertexIndex = Helper.RandInt(0, vertexCount - 1);
					values[clusterIndex] = newVertexIndex + Frac(values[clusterIndex]);
				}
		*/

		#endregion


		#region Public methods and properties

		public int this[int index]
		{
			get { return vertices[index]; }
		}

		public int Length
		{
			get { return length; }
		}

		#endregion

		public TourElement GetElement(int position)
		{
            int vertexIndex;
			int cluster = Task.GetCluster(vertices[position], out vertexIndex);
			RandomTourItem item = new RandomTourItem(cluster, vertexIndex);
			return new TourElement(item.clusterIndex, item.vertexInCluster);
		}


		/// <summary>
		/// Removes vertex from position oldPosition and
		/// inserts it in position newPosition.
		/// </summary>
		/// <param name="oldPosition"></param>
		/// <param name="newPosition"></param>
		/// is undefined.</param>
		public void Insert(int oldPosition, int newPosition)
		{
			if (oldPosition == newPosition
			    || RestoreIndex(oldPosition + 1) == newPosition)
				return;

			if (oldPosition < newPosition)
			{
				int oldV = vertices[oldPosition];
				for (int i = oldPosition; i < newPosition; i++)
					vertices[i] = vertices[i + 1];
				vertices[newPosition] = oldV;
			}
			else
			{
				int oldV = vertices[oldPosition];
				for (int i = oldPosition; i > newPosition; i--)
					vertices[i] = vertices[i - 1];
				vertices[newPosition] = oldV;
			}

			InvalidateFirstClusterPosition();
		}

		///<summary>
		///Creates a new object that is a copy of the current instance.
		///</summary>
		///
		///<returns>
		///A new object that is a copy of this instance.
		///</returns>
		///<filterpriority>2</filterpriority>
		public object Clone()
		{
			Tour copy = (Tour)MemberwiseClone();
			copy.vertices = (int[])vertices.Clone();
			return copy;
		}

		public static int[] Rotate(int[] tour, int firstClusterPosition)
		{
			int[] newTour = new int[tour.Length];
			Array.Copy(tour, firstClusterPosition, newTour, 0, tour.Length - firstClusterPosition);
			Array.Copy(tour, 0, newTour, tour.Length - firstClusterPosition, firstClusterPosition);
			return newTour;
		}

		public void CorrectRotation()
		{
			if (FirstClusterPosition == 0)
				return;

			vertices = Rotate(vertices, firstClusterPosition);
			firstClusterPosition = 0;
		}

		public int[] GetValues()
		{
			return vertices;
		}

		public int GetCluster(int position)
		{
			return Task.GetCluster(vertices[position]);
		}

		public void CheckTourIsCorrect()
		{
			if (vertices.Length != Task.ClusterCount)
				throw new GtspException("Wrong tour size");
			bool[] used = new bool[Task.ClusterCount];
			for (int i = 0; i < Task.ClusterCount; i++)
			{
				int clusterIndex = GetCluster(i);
				if (used[clusterIndex])
					throw new GtspException("Tour contains cluster {0} several times.", clusterIndex);

				used[clusterIndex] = true;
			}

			int oldLen = length;
			UpdateLength();
			if (length != oldLen)
				throw new GtspException("Tour length was wrong");
		}


		[DllImport("native_helper")]
		private static extern unsafe int Improve(int* solution, int oldLen);

		public void Improve()
		{
#if DEBUG
			CheckTourIsCorrect();
#endif

			unsafe
			{
				fixed (int* values = vertices)
				{
					length = Improve(values, length);
				}
			}

#if DEBUG
			CheckTourIsCorrect();
#endif
		}
	}
}

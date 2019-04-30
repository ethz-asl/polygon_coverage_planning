using System;
using System.Collections;
using System.Collections.Generic;

namespace GkMa.OurHeuristic.Types
{
	public class Generation : IEnumerable<Tour>
	{
		private readonly List<Tour> tours = new List<Tour>();
		private readonly Dictionary<Tour, bool> keysTable = new Dictionary<Tour, bool>();
		private readonly int generationIndex;
		private readonly TimeCounter counter = new TimeCounter();

		public Generation(int generationIndex)
		{
			this.generationIndex = generationIndex;
		}

		public int GenerationIndex
		{
			get { return generationIndex; }
		}

		public void BeginProcess()
		{
			// counter.Start();
		}

		public void EndProcess()
		{
			// counter.Stop();
		}

		public TimeSpan ProcessTime
		{
			get { return counter.Interval; }
		}

		public int Size
		{
			get { return tours.Count; }
		}

		public Tour this[int index]
		{
			get { return tours[index]; }
/*
			set
			{
				tours[index] = value;
			}
*/
		}

		public void Add(Tour tour)
		{
			if (Contains(tour))
				throw new GtspException("The generation contains this tour");
            tours.Add(tour);
            keysTable[tour] = true;
		}

		public void Sort()
		{
			tours.Sort();
		}

		public bool Contains(Tour tour)
		{
			return keysTable.ContainsKey(tour);
		}

		///<summary>
		///Returns an enumerator that iterates through the collection.
		///</summary>
		///
		///<returns>
		///A <see cref="T:System.Collections.Generic.IEnumerator`1"></see> that can be used to iterate through the collection.
		///</returns>
		///<filterpriority>1</filterpriority>
		IEnumerator<Tour> IEnumerable<Tour>.GetEnumerator()
		{
			return tours.GetEnumerator();
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
			return tours.GetEnumerator();
		}

		public int GetTourPosition(Tour tour)
		{
			for (int i = 0; i < tours.Count; i++)
				if (tour == tours[i])
					return i;

			throw new GtspException("Tour not found in this generation");
		}
	}
}
using System;
using System.Collections.Generic;

namespace GkMa.OurHeuristic.Types
{
	public class Permutation : ICloneable
	{
		private readonly int[] values;

		public Permutation(int n)
		{
			values = new int[n];
			for (int i = 0; i < n; i++)
				values[i] = i;
		}

		public Permutation(int[] values)
		{
			this.values = values;
		}

		/// <summary>
		/// Returns random permutation of the given length.
		/// </summary>
		/// <param name="n"></param>
		/// <returns></returns>
		public static Permutation GenerateRandom(int n)
		{
			Permutation result = new Permutation(n);
			for (int i = 0; i < n - 1; i++)
				result.Swap(i, Helper.RandIntLess(i, n));

			return result;
		}

		/// <summary>
		/// Swaps two values. The positions may be identical.
		/// </summary>
		/// <param name="index1"></param>
		/// <param name="index2"></param>
		public void Swap(int index1, int index2)
		{
			int t = values[index1];
			values[index1] = values[index2];
			values[index2] = t;
		}

		/// <summary>
		/// Returns the value in the given position
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		public int this[int index]
		{
			get { return values[index]; }
		}

		/// <summary>
		/// Permutation elements count
		/// </summary>
		public int Length
		{
			get { return values.Length; }
		}

		private static void CreatePermutations(List<Permutation> result, int level)
		{
			Permutation source = result[result.Count - 1];
			for (int index2 = level + 1; index2 < source.Length; index2++)
			{
				Permutation permutation = (Permutation)source.Clone();
				permutation.Swap(level, index2);
				result.Add(permutation);
				if (level < source.Length - 1)
					CreatePermutations(result, level + 1);
			}
		}

		private static int Factorial(int n)
		{
			if (n < 2)
				return 1;

			int result = n;
			while (n > 2)
				result *= --n;

			return result;
		}

		public static IEnumerator<Permutation> Enumerate(int n)
		{
			List<Permutation> result = new List<Permutation>(Factorial(n));
			result.Add(new Permutation(n));
			CreatePermutations(result, 0);
			return result.GetEnumerator();
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
			return new Permutation((int[])values.Clone());
		}

		public T[] Apply<T>(T[] elements)
		{
			T[] result = new T[elements.Length];
			for (int i = 0; i < elements.Length; i++)
				result[i] = elements[this[i]];

			return result;
		}
	}
}

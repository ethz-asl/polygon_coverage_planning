using System;
using GkMa.Loader;
using GkMa.OurHeuristic.Operators;
using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic
{
	internal class Algorithm : GeneticAlgorithm
	{
		private int maxIdleGenerations;

		public Algorithm(Task task)
			: base(task)
		{
			NativeHelper.InitProblem(task);
		}

		#region First generation

		/// <summary>
		/// Max try number for first generation
		/// </summary>
		private int MaxTriesForFirstGeneration
		{
			get { return Task.IsSymmetric ? Task.ClusterCount * 2 : Task.ClusterCount * 4; }
		}

		protected override void FirstGeneration(Generation cur)
		{
			RandomGenerate generatingAlgorithm = new RandomGenerate(Task);

			for (int tries = 0; tries < MaxTriesForFirstGeneration; tries++)
            {
                Tour tour = new Tour(generatingAlgorithm);
                tour.Improve();
				if (!cur.Contains(tour))
                    cur.Add(tour);
            }
            cur.Sort();
		}

		#endregion

		#region Next generation

		private int ReproductionsCount
		{
			get { return 10 + Task.ClusterCount / 20 + GenerationCount / 5; }
		}

		private int CrossoverTries
		{
			get { return 8 * ReproductionsCount; }
		}

		private int MutationTries
		{
			get { return 2 * ReproductionsCount; }
		}

		private void Reproduction(Generation cur)
		{
			for (int i = 0; i < ReproductionsCount; i++)
			{
				Tour tour = (Tour)LastGeneration[i].Clone();
				cur.Add(tour);
			}
		}

		private readonly ElitistTwoStrategy crossoverStrategy = new ElitistTwoStrategy(0.33f);
		private readonly UniformCrossover crossoverOperator = new UniformCrossover();

		private void Crossover(Generation cur)
		{
			for (int i = 0; i < CrossoverTries; i++)
            {
				Tour tour = new Tour(LastGeneration, crossoverStrategy, crossoverOperator);
				tour.Improve();
				if (!cur.Contains(tour))
					cur.Add(tour);
			}
		}

		private readonly IRandomMutation mutationOperator = new InsertFragment();
		private readonly IOneElementStrategy mutationStrategy = new ElitistOneStrategy(0.75);

		private void Mutation(Generation cur)
		{
			for (int i = 0; i < MutationTries; i++)
			{
				int index;
				mutationStrategy.Run(LastGeneration, out index);

				Tour tour = mutationOperator.Run(LastGeneration[index]);

				tour.Improve();
				if (!cur.Contains(tour))
					cur.Add(tour);
			}
		}

		private void NextGenerationPure(Generation cur)
		{
			Reproduction(cur);
			Crossover(cur);
			Mutation(cur);
		}

		private int consecutiveIdleGenerationCount;

		protected override void NextGeneration(Generation cur)
		{
			NextGenerationPure(cur);
			cur.Sort();

			if (cur[0].Length != LastGeneration[0].Length)
			{
				maxIdleGenerations = Math.Max(maxIdleGenerations, consecutiveIdleGenerationCount);
				consecutiveIdleGenerationCount = 0;
			}
			else
				consecutiveIdleGenerationCount++;
		}

		#endregion

		protected override bool StopCondition()
		{
			return consecutiveIdleGenerationCount >
			       Math.Max(maxIdleGenerations * 3 / 2, (Task.ClusterCount / 20 + (Task.IsSymmetric ? 5 : 10)));
		}
	}

	internal class RandomisationPeriod
	{
	}
}
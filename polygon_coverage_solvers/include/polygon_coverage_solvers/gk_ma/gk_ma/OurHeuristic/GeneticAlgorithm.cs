using System;
using System.Diagnostics;
using System.Threading;
using GkMa.Loader;
using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic
{
	internal abstract class GeneticAlgorithm
	{
		private readonly Task task;
		private TimeSpan totalTime;
		private bool paused;

		protected GeneticAlgorithm(Task task)
		{
			this.task = task;
		}

		public Task Task
		{
			get { return task; }
		}

		public TimeSpan TotalTime
		{
			get { return totalTime; }
		}

		private Generation prev;
		private int generationCount;

		protected void AddGeneration(Generation generation)
		{
			prev = generation;
			generationCount++;
		}

		public bool Run()
		{
			// Process.GetCurrentProcess().PriorityClass = ProcessPriorityClass.High;
			// Thread.CurrentThread.Priority = ThreadPriority.Highest;
			// Process.GetCurrentProcess().ProcessorAffinity = (IntPtr)1;
			Generation generation = new Generation(0);

			generation.BeginProcess();
			FirstGeneration(generation);
			generation.EndProcess();
			AddGeneration(generation);

			OnGenerationCreated();

			while (!StopCondition())
			{
				generation = new Generation(LastGeneration.GenerationIndex + 1);
				generation.BeginProcess();
				NextGeneration(generation);
				generation.EndProcess();
				AddGeneration(generation);

				OnGenerationCreated();
			}
			return true;
		}

		protected abstract bool StopCondition();

		public int GenerationCount
		{
			get { return generationCount; }
		}

		protected virtual void OnGenerationCreated()
		{
			totalTime += LastGeneration.ProcessTime;
		}

		public Generation LastGeneration
		{
			get { return prev; }
		}

		public bool Paused
		{
			get { return paused; }
			set { paused = value; }
		}

		protected abstract void FirstGeneration(Generation cur);
		protected abstract void NextGeneration(Generation cur);
	}
}
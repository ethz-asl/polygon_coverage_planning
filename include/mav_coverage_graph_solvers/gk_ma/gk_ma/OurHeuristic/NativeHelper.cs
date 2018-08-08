using System;
using System.Runtime.InteropServices;
using System.Security;
using GkMa.Loader;

namespace GkMa.OurHeuristic
{
	public static class NativeHelper
	{
		[SuppressUnmanagedCodeSecurity]
        [DllImport("native_helper")]
		private static extern unsafe bool InitProblem(
			int vertexCount,
			int clusterCount,
			int* weights,
			int* clusterSizes,
			int** clusters,
			bool isSymmetric);

		private static Task previouslyInitedProblem;

		public static unsafe void InitProblem(Task task)
		{
			if (task == previouslyInitedProblem)
				return;

			previouslyInitedProblem = task;

			int[] clusterSizes = new int[task.ClusterCount];
			int*[] clusters = new int*[task.ClusterCount];
			for (int i = 0; i < clusterSizes.Length; i++)
			{
				clusterSizes[i] = task[i].Length;
				fixed (int* clusterVertices = task[i])
					clusters[i] = clusterVertices;
			}

			int[] weights = new int[task.VertexCount * task.VertexCount];
			for (int i = 0; i < task.VertexCount; i++)
				Array.Copy(task.GetWeightsFrom(i), 0, weights, i * task.VertexCount, task.VertexCount);

			fixed (int* weightsPtr = weights)
			fixed (int* clusterSizesPtr = clusterSizes)
			fixed (int** clustersPtr = clusters)
				if (!InitProblem(
				     	task.VertexCount,
				     	task.ClusterCount,
				     	weightsPtr,
				     	clusterSizesPtr,
				     	clustersPtr,
				     	task.IsSymmetric))
				{
					throw new GtspException("InitProblem failed.");
				}
		}


		[SuppressUnmanagedCodeSecurity]
		[DllImport("native_helper")]
		private static extern unsafe int ClusterOptimisation(int* solution);

		public static unsafe int ClusterOptimisation(int[] solution)
		{
			fixed (int* values = solution)
				return ClusterOptimisation(values);
		}
	}
}

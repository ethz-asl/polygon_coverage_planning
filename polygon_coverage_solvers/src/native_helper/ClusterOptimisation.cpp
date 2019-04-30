#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "polygon_coverage_solvers/gk_ma/native_helper/NativeHelper.h"

int *solution;
int clusterSequence[MAX_CLUSTER_COUNT];
int bestSolution[MAX_CLUSTER_COUNT];

int FindSmallestClusterPosition()
{
	int minSize = INT_MAX;
	int bestPos;
	for (int i = 0; i < clusterCount; i++)
	{
		int vertex = solution[i];
		int cluster = clusterByVertex[vertex];
		int clusterSize = clusterSizes[cluster];
		if (clusterSize < minSize)
		{
			if (clusterSize == 1)
				return i;

			minSize = clusterSize;
			bestPos = i;
		}
	}

	return bestPos;
}

void InitClusters()
{
	int firstPosition = FindSmallestClusterPosition();
	int index = 0;
	for (int i = firstPosition; i < clusterCount; i++)
		clusterSequence[index++] = clusterByVertex[solution[i]];

	for (int i = 0; i < firstPosition; i++)
		clusterSequence[index++] = clusterByVertex[solution[i]];
}

static int bestFrom[MAX_VERTEX_COUNT];
static int bestLen1[MAX_CLUSTER_SIZE];
static int bestLen2[MAX_CLUSTER_SIZE];

void RestoreSolution(int *vertexIndices, int firstVertex)
{
	int prev = firstVertex;
	for (int i = clusterCount - 1; i >= 0; i--)
		vertexIndices[i] = prev = bestFrom[prev];
}

int FindBestPath(int firstClusterVertex)
{
	int *bestCurLen = bestLen1;
	int *bestPrevLen = bestLen2;

	int toCluster = clusterSequence[1];
	int fromCluster;
	for (int i = 0; i < clusterSizes[toCluster]; i++)
	{
		int toVertex = clusters[toCluster][i];
		bestPrevLen[i] = weights[firstClusterVertex][toVertex];
		bestFrom[toVertex] = firstClusterVertex;
	}

	for (int pos = 2; pos < clusterCount; pos++)
	{
		toCluster = clusterSequence[pos];
		fromCluster = clusterSequence[pos - 1];
		for (int toVertexIndex = 0; toVertexIndex < clusterSizes[toCluster]; toVertexIndex++)
		{
			int toVertex = clusters[toCluster][toVertexIndex];
			bestCurLen[toVertexIndex] = INT_MAX;
			for (int fromVertexIndex = 0; fromVertexIndex < clusterSizes[fromCluster]; fromVertexIndex++)
			{
				int fromVertex = clusters[fromCluster][fromVertexIndex];
				int len = bestPrevLen[fromVertexIndex] + weights[fromVertex][toVertex];
				if (len < bestCurLen[toVertexIndex])
				{
					bestCurLen[toVertexIndex] = len;
					bestFrom[toVertex] = fromVertex;
				}
			}
		}

		{
			int *temp = bestCurLen;
			bestCurLen = bestPrevLen;
			bestPrevLen = temp;
		}
	}

	fromCluster = clusterSequence[clusterCount - 1];
	int minLen = INT_MAX;
	for (int i = clusterSizes[fromCluster] - 1; i >= 0; i--)
	{
		int fromVertex = clusters[fromCluster][i];
		int len = bestPrevLen[i] + weights[fromVertex][firstClusterVertex];
		if (len < minLen)
		{
			minLen = len;
			bestFrom[firstClusterVertex] = fromVertex;
		}
	}

	return minLen;
}

int ClusterOptimisation(int *solution)
{
	::solution = solution;
	InitClusters();

	int minLen = INT_MAX;
	int firstCluster = clusterSequence[0];
	for (int i = clusterSizes[firstCluster] - 1; i >= 0; i--)
	{
		int firstVertex = clusters[firstCluster][i];
		int len = FindBestPath(firstVertex);
		if (len < minLen)
		{
			if (i == 0)
			{
				RestoreSolution(solution, firstVertex);
				return len;
			}
			else
			{
				minLen = len;
				RestoreSolution(bestSolution, firstVertex);
			}
		}
	}

	memcpy(solution, bestSolution, clusterCount * sizeof(int));
	return minLen;
}

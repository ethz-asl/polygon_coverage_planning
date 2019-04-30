#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "polygon_coverage_solvers/gk_ma/native_helper/NativeHelper.h"

int clusterCount = 0;
int vertexCount = 0;
bool isSymmetric;
int clusterSizes[MAX_CLUSTER_COUNT];
int *clusters[MAX_CLUSTER_COUNT];
int clusterValues[MAX_VERTEX_COUNT];

int *weights[MAX_VERTEX_COUNT];
int weightValues[MAX_VERTEX_COUNT * MAX_VERTEX_COUNT];

int clusterByVertex[MAX_VERTEX_COUNT];

int *minClusterDistances[MAX_CLUSTER_COUNT];
int minClusterDistanceValues[MAX_CLUSTER_COUNT * MAX_CLUSTER_COUNT];


int CalculateMinClusterDistance(int fromCluster, int toCluster)
{
	int min = INT_MAX;
	int fromClusterSize = clusterSizes[fromCluster];
	int toClusterSize = clusterSizes[toCluster];
	for (int fromVertexIndex = fromClusterSize - 1; fromVertexIndex >= 0; fromVertexIndex--)
	{
		int fromVertex = clusters[fromCluster][fromVertexIndex];
		for (int toVertexIndex = clusterSizes[toCluster] - 1; toVertexIndex >= 0; toVertexIndex--)
		{
			int toVertex = clusters[toCluster][toVertexIndex];
			int w = weights[fromVertex][toVertex];
			if (w < min)
				min = w;
		}
	}

	return min;
}

bool InitProblem(int newVertexCount, int newClusterCount, int *newWeights, int *newClusterSizes, int **newClusters, bool newIsSymmetric)
{
	if (newClusterCount > MAX_CLUSTER_COUNT)
		return false;

	vertexCount = newVertexCount;
	clusterCount = newClusterCount;
	isSymmetric = newIsSymmetric;

	memcpy(clusterSizes, newClusterSizes, clusterCount * sizeof(int));

	int index = 0;
	for (int i = 0; i < clusterCount; i++)
	{
		clusters[i] = clusterValues + index;
		memcpy(clusters[i], newClusters[i], clusterSizes[i] * sizeof(int));
		index += clusterSizes[i];
	}

	for (int i = 0; i < clusterCount; i++)
		for (int j = 0; j < clusterSizes[i]; j++)
			clusterByVertex[clusters[i][j]] = i;

	for (int i = 0; i < vertexCount; i++)
	{
		weights[i] = weightValues + i * vertexCount;
		memcpy(weights[i], newWeights + i * vertexCount, vertexCount * sizeof(int));
	}

	for (int fromCluster = clusterCount - 1; fromCluster >= 0; fromCluster--)
		minClusterDistances[fromCluster] = minClusterDistanceValues + fromCluster * clusterCount;

	for (int fromCluster = clusterCount - 1; fromCluster >= 0; fromCluster--)
	{
		for (int toCluster = fromCluster - 1; toCluster >= 0; toCluster--)
			minClusterDistances[fromCluster][toCluster] = CalculateMinClusterDistance(fromCluster, toCluster);

		if (isSymmetric)
			for (int toCluster = fromCluster - 1; toCluster >= 0; toCluster--)
				minClusterDistances[toCluster][fromCluster] = minClusterDistances[fromCluster][toCluster];
		else
			for (int toCluster = clusterCount - 1; toCluster > fromCluster; toCluster--)
				minClusterDistances[fromCluster][toCluster] = CalculateMinClusterDistance(fromCluster, toCluster);
	}

	return true;
}

int RandomPosition()
{
	return rand() % clusterCount;
}

int PrevPos(int pos)
{
	if (pos == 0)
		return clusterCount - 1;

	return pos - 1;
}

int NextPos(int pos)
{
	if (pos == clusterCount - 1)
		return 0;

	return pos + 1;
}

int RestoreLargePos(int largePos)
{
	if (largePos >= clusterCount)
		return largePos - clusterCount;
	return largePos;
}

int RestoreSmallPos(int smallPos)
{
	if (smallPos < 0)
		return smallPos + clusterCount;
	return smallPos;
}

int CalculateLen(int *solution)
{
	int len = 0;
	int prevVertex = solution[clusterCount - 1];
	for (int i = 0; i < clusterCount; i++)
	{
		int curVertex = solution[i];
		len += weights[prevVertex][curVertex];
		prevVertex = curVertex;
	}

	return len;
}

int Weight(int v1, int v2)
{
	return weights[v1][v2];
}

int Weight(int v1, int v2, int v3)
{
	return weights[v1][v2] + weights[v2][v3];
}

int Weight(int v1, int v2, int v3, int v4)
{
	return weights[v1][v2] + weights[v2][v3] + weights[v3][v4];
}

int Weight(int v1, int v2, int v3, int v4, int v5)
{
	return weights[v1][v2] + weights[v2][v3] + weights[v3][v4] + weights[v4][v5];
}

int Weight(int v1, int v2, int v3, int v4, int v5, int v6)
{
	return weights[v1][v2] + weights[v2][v3] + weights[v3][v4] + weights[v4][v5] + weights[v5][v6];
}


int MinClusterDistance(int c1, int c2)
{
	return minClusterDistances[c1][c2];
}

int MinClusterDistance(int c1, int c2, int c3)
{
	return minClusterDistances[c1][c2]
		+ minClusterDistances[c2][c3];
}

int MinClusterDistance(int c1, int c2, int c3, int c4)
{
	return minClusterDistances[c1][c2]
		+ minClusterDistances[c2][c3]
		+ minClusterDistances[c3][c4];
}

int MinClusterDistance(int c1, int c2, int c3, int c4, int c5)
{
	return minClusterDistances[c1][c2]
		+ minClusterDistances[c2][c3]
		+ minClusterDistances[c3][c4]
		+ minClusterDistances[c4][c5];
}

int MinClusterDistance(int c1, int c2, int c3, int c4, int c5, int c6)
{
	return minClusterDistances[c1][c2]
		+ minClusterDistances[c2][c3]
		+ minClusterDistances[c3][c4]
		+ minClusterDistances[c4][c5]
		+ minClusterDistances[c5][c6];
}

int MinClusterDistanceForVertices(int v1, int v2)
{
	return MinClusterDistance(clusterByVertex[v1], clusterByVertex[v2]);
}

int MinClusterDistanceForVertices(int v1, int v2, int v3)
{
	return MinClusterDistance(
		clusterByVertex[v1],
		clusterByVertex[v2],
		clusterByVertex[v3]);
}

int MinClusterDistanceForVertices(int v1, int v2, int v3, int v4)
{
	return MinClusterDistance(
		clusterByVertex[v1],
		clusterByVertex[v2],
		clusterByVertex[v3],
		clusterByVertex[v4]);
}



int InsertWithClusterOptimisationCuts;
int InsertWithClusterOptimisationCutsOf;

int TotalPosDelta;
int TotalPosDeltaCount;

bool DumbCounters(char *fileName)
{
	FILE *f = fopen(fileName, "a");
	if (f == NULL)
		return false;

	fprintf(f, "InsertWithClusterOptimisationCuts %d\n", InsertWithClusterOptimisationCuts);
	fprintf(f, "InsertWithClusterOptimisationCutsOf %d\n", InsertWithClusterOptimisationCutsOf);
	InsertWithClusterOptimisationCuts = 0;
	InsertWithClusterOptimisationCutsOf = 0;

	fprintf(f, "TotalPosDelta %d\n", TotalPosDelta);
	fprintf(f, "TotalPosDeltaCount %d\n", TotalPosDeltaCount);
	TotalPosDelta = 0;
	TotalPosDeltaCount = 0;

	fclose(f);
	return true;
}

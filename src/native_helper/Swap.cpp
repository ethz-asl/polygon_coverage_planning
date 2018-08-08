#include "mav_coverage_planning/gk_ma/native_helper/Swap.h"

#include <limits.h>

int FullSwap(int *solution)
{
	int totalDelta = 0;

	int next2Pos = 0;
	for (int pos2 = clusterCount - 1; pos2 > 1; pos2--)
	{
		int prev2Pos = pos2 - 1;
		int prev2 = solution[prev2Pos];
		int cur2 = solution[pos2];
		int next2 = solution[next2Pos];

		for (int pos1 = pos2 - 2; pos1 > 0; pos1--)
		{
			int prev1Pos = pos1 - 1;
			int prev1 = solution[prev1Pos];
			int cur1 = solution[pos1];
			int next1Pos = pos1 + 1;
			int next1 = solution[next1Pos];

			int delta
				= Weight(prev1, cur2, next1)
				+ Weight(prev2, cur1, next2)
				- Weight(prev1, cur1, next1)
				- Weight(prev2, cur2, next2);

			if (delta < 0)
			{
				solution[pos1] = cur2;
				solution[pos2] = cur1;

				cur2 = solution[pos2];

				totalDelta += delta;
			}
		}

		next2Pos = pos2;
	}

	return totalDelta;
}


int NeighbourSwapWithClusterOptimisation(int prevVertex, int *v1, int *v2, int nextVertex)
{
	int oldLen = Weight(prevVertex, *v1, *v2, nextVertex);
	int cluster1 = clusterByVertex[*v1];
	int cluster2 = clusterByVertex[*v2];

	int bestV1, bestV2;

	if (MinClusterDistance(
		clusterByVertex[prevVertex], cluster2, cluster1, clusterByVertex[nextVertex])
		>= oldLen)
	{
		return 0;
	}

	int minDelta = 0;
	for (int v1index = clusterSizes[cluster1] - 1; v1index >= 0; v1index--)
	{
		int curV2 = clusters[cluster1][v1index];
		for (int v2index = clusterSizes[cluster2] - 1; v2index >= 0; v2index--)
		{
			int curV1 = clusters[cluster2][v2index];

			int len = Weight(prevVertex, curV1, curV2, nextVertex);
			int delta = len - oldLen;
			if (delta < minDelta)
			{
				minDelta = delta;
				bestV1 = curV1;
				bestV2 = curV2;
			}
		}
	}

	if (minDelta < 0)
	{
		*v1 = bestV1;
		*v2 = bestV2;
	}

	return minDelta;
}

int NeighbourSwapWithCO(int *solution)
{
	int totalDelta = 0;
	int prevPos = clusterCount - 3;
	int v1Pos = clusterCount - 2;
	int v2Pos = clusterCount - 1;

	for (int nextPos = 0; nextPos < clusterCount; nextPos++)
	{
		totalDelta += NeighbourSwapWithClusterOptimisation(
			solution[prevPos],
			solution + v1Pos,
			solution + v2Pos,
			solution[nextPos]);

		prevPos = v1Pos;
		v1Pos = v2Pos;
		v2Pos = nextPos;
	}

	return totalDelta;
}


int TrySwap(int prev, int c1, int c2, int c3, int next, int *vertices, int oldDist)
{
	int prevC = clusterByVertex[prev];
	int nextC = clusterByVertex[next];

	if (MinClusterDistance(prevC, c1, c2, c3, nextC) >= oldDist)
		return 0;

	int c1Size = clusterSizes[c1];
	int c2Size = clusterSizes[c2];
	int c3Size = clusterSizes[c3];

	int minTotal = oldDist;

	int bestV1;
	int bestV2;
	int bestV3;

	for (int v2Index = c2Size - 1; v2Index >= 0; v2Index--)
	{
		int curV2 = clusters[c2][v2Index];

		int leftMin = INT_MAX;
		int bestV1forCurV2;
		for (int v1Index = c1Size - 1; v1Index >= 0; v1Index--)
		{
			int curV1 = clusters[c1][v1Index];

			int left = Weight(prev, curV1, curV2);
			if (left < leftMin)
			{
				leftMin = left;
				bestV1forCurV2 = curV1;
			}
		}

		int rightMin = INT_MAX;
		int bestV3forCurV2;
		for (int v3Index = c3Size - 1; v3Index >= 0; v3Index--)
		{
			int curV3 = clusters[c3][v3Index];

			int right = Weight(curV2, curV3, next);
			if (right < rightMin)
			{
				rightMin = right;
				bestV3forCurV2 = curV3;
			}
		}

		if (leftMin + rightMin < minTotal)
		{
			bestV1 = bestV1forCurV2;
			bestV2 = curV2;
			bestV3 = bestV3forCurV2;
			minTotal = leftMin + rightMin;
		}
	}

	int delta = minTotal - oldDist;
	if (delta < 0)
	{
		vertices[0] = bestV1;
		vertices[1] = bestV2;
		vertices[2] = bestV3;
	}

	return delta;
}

int ThreeNeighbourFullSwap(int *solution)
{
	int totalDelta = 0;

	int prevPos = clusterCount - 4;
	int pos1 = clusterCount - 3;
	int pos2 = clusterCount - 2;
	int pos3 = clusterCount - 1;
	for (int nextPos = 0; nextPos < clusterCount - 1; nextPos++)
	{
		int prev = solution[prevPos];
		int v1 = solution[pos1];
		int v2 = solution[pos2];
		int v3 = solution[pos3];
		int next = solution[nextPos];

		int c1 = clusterByVertex[v1];
		int c2 = clusterByVertex[v2];
		int c3 = clusterByVertex[v3];

		int oldDist = Weight(prev, v1, v2, v3, next);

		int vertices1[3];
		int vertices2[3];
		int vertices3[3];

		int delta1 = TrySwap(prev, c2, c3, c1, next, vertices1, oldDist);
		int delta2 = TrySwap(prev, c3, c1, c2, next, vertices2, oldDist);
		int delta3 = TrySwap(prev, c3, c2, c1, next, vertices3, oldDist);
		if (delta1 < 0 || delta2 < 0 || delta3 < 0)
		{
			int *vertices;
			if (delta1 <= delta2 && delta1 <= delta3)
			{
				vertices = vertices1;
				totalDelta += delta1;
			}
			else if (delta2 <= delta1 && delta2 <= delta3)
			{
				vertices = vertices2;
				totalDelta += delta2;
			}
			else if (delta3 <= delta1 && delta3 <= delta2)
			{
				vertices = vertices3;
				totalDelta += delta3;
			}

			solution[pos1] = vertices[0];
			solution[pos2] = vertices[1];
			solution[pos3] = vertices[2];
		}

		prevPos = pos1;
		pos1 = pos2;
		pos2 = pos3;
		pos3 = nextPos;
	}

	return totalDelta;
}


int TrySwap(int prev, int c1, int c2, int c3, int c4, int next, int *vertices, int oldDist)
{
	int prevC = clusterByVertex[prev];
	int nextC = clusterByVertex[next];

	if (MinClusterDistance(prevC, c1, c2, c3, c4, nextC) >= oldDist)
		return 0;

	int c1Size = clusterSizes[c1];
	int c2Size = clusterSizes[c2];
	int c3Size = clusterSizes[c3];
	int c4Size = clusterSizes[c4];

	int bestV4[MAX_CLUSTER_SIZE];
	int minLenFromV3[MAX_CLUSTER_SIZE];

	for (int v3Index = c3Size - 1; v3Index >= 0; v3Index--)
	{
		minLenFromV3[v3Index] = INT_MAX;
		int v3 = clusters[c3][v3Index];

		for (int v4Index = c4Size - 1; v4Index >= 0; v4Index--)
		{
			int v4 = clusters[c4][v4Index];
			int len = Weight(v3, v4, next);
			if (minLenFromV3[v3Index] > len)
			{
				minLenFromV3[v3Index] = len;
				bestV4[v3Index] = v4;
			}
		}
	}

	int minTotal = oldDist;

	int bestV1;
	int bestV2;
	int bestV3Index;

	for (int v2Index = c2Size - 1; v2Index >= 0; v2Index--)
	{
		int curV2 = clusters[c2][v2Index];

		int leftMin = INT_MAX;
		int bestV1forCurV2;
		for (int v1Index = c1Size - 1; v1Index >= 0; v1Index--)
		{
			int curV1 = clusters[c1][v1Index];

			int left = Weight(prev, curV1, curV2);
			if (left < leftMin)
			{
				leftMin = left;
				bestV1forCurV2 = curV1;
			}
		}

		int rightMin = INT_MAX;
		int bestV3IndexForCurV2;
		for (int v3Index = c3Size - 1; v3Index >= 0; v3Index--)
		{
			int curV3 = clusters[c3][v3Index];

			int right = Weight(curV2, curV3) + minLenFromV3[v3Index];
			if (right < rightMin)
			{
				rightMin = right;
				bestV3IndexForCurV2 = v3Index;
			}
		}

		if (leftMin + rightMin < minTotal)
		{
			bestV1 = bestV1forCurV2;
			bestV2 = curV2;
			bestV3Index = bestV3IndexForCurV2;
			minTotal = leftMin + rightMin;
		}
	}

	int delta = minTotal - oldDist;
	if (delta < 0)
	{
		vertices[0] = bestV1;
		vertices[1] = bestV2;
		vertices[2] = clusters[c3][bestV3Index];
		vertices[3] = bestV4[bestV3Index];
	}

	return delta;
}

int permutations[13][4] =
{
	{ 1, 2, 3, 0},
	{ 1, 3, 0, 2},
	{ 1, 3, 2, 0},
	{ 2, 0, 3, 1},
	{ 2, 1, 3, 0},
	{ 2, 3, 0, 1},
	{ 2, 3, 1, 0},
	{ 3, 0, 1, 2},
	{ 3, 0, 2, 1},
	{ 3, 1, 0, 2},
	{ 3, 1, 2, 0},
	{ 3, 2, 0, 1},
	{ 3, 2, 1, 0},
};

int FourNeighbourFullSwap(int *solution)
{
	int totalDelta = 0;

	int prevPos = clusterCount - 5;
	int pos1 = clusterCount - 4;
	int pos2 = clusterCount - 3;
	int pos3 = clusterCount - 2;
	int pos4 = clusterCount - 1;
	for (int nextPos = 0; nextPos < clusterCount - 1; nextPos++)
	{
		int prev = solution[prevPos];
		int v1 = solution[pos1];
		int v2 = solution[pos2];
		int v3 = solution[pos3];
		int v4 = solution[pos4];
		int next = solution[nextPos];

		int c[4];
		c[0] = clusterByVertex[v1];
		c[1] = clusterByVertex[v2];
		c[2] = clusterByVertex[v3];
		c[3] = clusterByVertex[v4];

		int oldDist = Weight(prev, v1, v2, v3, v4, next);

		int curVertices[4];
		int bestVertices[4];

		int minDelta = INT_MAX;
		for (int i = 0; i < 13; i++)
		{
			int delta = TrySwap(
				prev,
				c[permutations[i][0]],
				c[permutations[i][1]],
				c[permutations[i][2]],
				c[permutations[i][3]],
				next,
				curVertices,
				oldDist);

			if (delta < minDelta)
			{
				minDelta = delta;
				bestVertices[0] = curVertices[0];
				bestVertices[1] = curVertices[1];
				bestVertices[2] = curVertices[2];
				bestVertices[3] = curVertices[3];
			}
		}

		if (minDelta < 0)
		{
			totalDelta += minDelta;
			solution[pos1] = bestVertices[0];
			solution[pos2] = bestVertices[1];
			solution[pos3] = bestVertices[2];
			solution[pos4] = bestVertices[3];
		}

		prevPos = pos1;
		pos1 = pos2;
		pos2 = pos3;
		pos3 = pos4;
		pos4 = nextPos;
	}

	return totalDelta;
}

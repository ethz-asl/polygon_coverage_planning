#include <string.h>
#include <stdlib.h>
#include "mav_coverage_planning/gk_ma/native_helper/KOpt.h"
#include "mav_coverage_planning/gk_ma/native_helper/NativeHelper.h"

int TwoOptFullSym(int *solution)
{
	int totalDelta = 0;
	for (int begin2pos = clusterCount - 1; begin2pos > 0; begin2pos--)
	{
		int end1pos = begin2pos - 1;
		int end1 = solution[end1pos];
		int begin2 = solution[begin2pos];

		int end2pos = clusterCount - 1;
		for (int begin1pos = 0; begin1pos < begin2pos - 1; begin1pos++)
		{
			int begin1 = solution[begin1pos];
			int end2 = solution[end2pos];

			int delta = Weight(end1, end2) + Weight(begin2, begin1)
				- (Weight(end1, begin2) + Weight(end2, begin1));

			if (delta < 0)
			{
				int len1 = end1pos - begin1pos + 1;
				int len2 = end2pos - begin2pos + 1;
				if (len2 <= 0)
					len2 += clusterCount;

				int p1;
				int p2;
				int len;
				if (len1 < len2)
				{
					p1 = begin1pos;
					p2 = end1pos;
					len = len1;
				}
				else
				{
					p1 = begin2pos;
					p2 = end2pos;
					len = len2;
				}

				for (int i = len / 2; i > 0; i--)
				{
					int temp = solution[p1];
					solution[p1] = solution[p2];
					solution[p2] = temp;
					p1 = NextPos(p1);
					p2 = PrevPos(p2);
				}

				totalDelta += delta;

				end1 = solution[end1pos];
				begin2 = solution[begin2pos];
			}

			end2pos = begin1pos;
		}
	}

	return totalDelta;
}

int TwoOptFullAsym(int *solution)
{
	int totalDelta = 0;

	for (int begin2pos = 0; begin2pos < clusterCount; begin2pos++)
	{
		int reverseDelta = 0;
		int end1pos = PrevPos(begin2pos);
		for (int len = 2; len < clusterCount - 1; len++)
		{
			int end2pos = RestoreLargePos(begin2pos + len - 1);
			int begin1pos = NextPos(end2pos);
			int prevEnd2pos = PrevPos(end2pos);
			reverseDelta += Weight(solution[end2pos], solution[prevEnd2pos])
				- Weight(solution[prevEnd2pos], solution[end2pos]);

			int delta = Weight(solution[end1pos], solution[end2pos])
				+ Weight(solution[begin2pos], solution[begin1pos])
				- Weight(solution[end1pos], solution[begin2pos])
				- Weight(solution[end2pos], solution[begin1pos])
				+ reverseDelta;

			if (delta < 0)
			{
				totalDelta += delta;
				reverseDelta = -reverseDelta;

				int p1 = begin2pos;
				int p2 = end2pos;
				for (int i = 0; i < len / 2; i++)
				{
					int temp = solution[p1];
					solution[p1] = solution[p2];
					solution[p2] = temp;

					p1 = NextPos(p1);
					p2 = PrevPos(p2);
				}
			}
		}
	}

	return totalDelta;
}

struct Edge
{
	int pos2;
	int length;
};

int CompareEdges(const void *p1, const void *p2)
{
	Edge *edge1 = (Edge*)p1;
	Edge *edge2 = (Edge*)p2;

	if (edge1->length < edge2->length)
		return -1;

	if (edge1->length > edge2->length)
		return 1;

	return 0;
}

Edge edges[MAX_CLUSTER_COUNT];
int edgeArraySize;

void InsertEdge(int pos2, int w)
{
	edges[0].pos2 = pos2;
	edges[0].length = w;

	for (int i = 1; i < edgeArraySize; i++)
		if (edges[i - 1].length > edges[i].length)
		{
			Edge temp = edges[i];
			edges[i] = edges[i - 1];
			edges[i - 1] = temp;
		}
}

int DirectTwoOptSym(int *solution)
{
	int n = clusterCount / 4;

	edgeArraySize = n;
	int prevV = solution[clusterCount - 1];
	for (int i = 0; i < n; i++)
	{
		int curV = solution[i];
		edges[i].pos2 = i;
		edges[i].length = weights[prevV][curV];
		prevV = curV;
	}

	qsort(edges, n, sizeof(Edge), CompareEdges);

	for (int i = n; i < clusterCount; i++)
	{
		int curV = solution[i];
		int w = weights[prevV][curV];
		if (w > edges[0].length)
			InsertEdge(i, w);
	}


	int totalDelta = 0;

	for (int begin2Index = n - 1; begin2Index > 0; begin2Index--)
	{
		int begin2pos = edges[begin2Index].pos2;
		int end1pos = PrevPos(begin2pos);
		int end1 = solution[end1pos];
		int begin2 = solution[begin2pos];

		for (int begin1Index = begin2Index - 1; begin1Index >= 0; begin1Index--)
		{
			int begin1pos = edges[begin1Index].pos2;
			int end2pos = PrevPos(begin1pos);
			if (begin1pos == end1pos || begin2pos == end2pos)
				continue;

			int begin1 = solution[begin1pos];
			int end2 = solution[end2pos];

			int delta = weights[end1][end2] + weights[begin2][begin1]
				- (weights[end1][begin2] + weights[end2][begin1]);

			if (delta < 0)
			{
				int len1 = end1pos - begin1pos + 1;
				int len2 = end2pos - begin2pos + 1;
				if (len1 <= 0)
					len1 += clusterCount;
				if (len2 <= 0)
					len2 += clusterCount;

				int p1;
				int p2;
				int len;
				if (len1 < len2)
				{
					p1 = begin1pos;
					p2 = end1pos;
					len = len1;
				}
				else
				{
					p1 = begin2pos;
					p2 = end2pos;
					len = len2;
				}

				for (int i = len / 2; i > 0; i--)
				{
					int temp = solution[p1];
					solution[p1] = solution[p2];
					solution[p2] = temp;
					p1 = NextPos(p1);
					p2 = PrevPos(p2);
				}

				totalDelta += delta;

				end1 = solution[end1pos];
				begin2 = solution[begin2pos];
			}
		}
	}

	return totalDelta;
}


int DirectTwoOptAsym(int *solution)
{
	int n = clusterCount / 4;

	edgeArraySize = n;
	int prevV = solution[clusterCount - 1];
	for (int i = 0; i < n; i++)
	{
		int curV = solution[i];
		edges[i].pos2 = i;
		edges[i].length = weights[prevV][curV];
		prevV = curV;
	}

	qsort(edges, n, sizeof(Edge), CompareEdges);

	for (int i = n; i < clusterCount; i++)
	{
		int curV = solution[i];
		int w = weights[prevV][curV];
		if (w > edges[0].length)
			InsertEdge(i, w);
	}


	int totalDelta = 0;

	for (int begin2Index = n - 1; begin2Index > 0; begin2Index--)
	{
		int begin2pos = edges[begin2Index].pos2;
		int end1pos = PrevPos(begin2pos);
		int end1 = solution[end1pos];
		int begin2 = solution[begin2pos];

		for (int begin1Index = begin2Index - 1; begin1Index >= 0; begin1Index--)
		{
			int begin1pos = edges[begin1Index].pos2;
			int end2pos = PrevPos(begin1pos);
			if (begin1pos == end1pos || begin2pos == end2pos)
				continue;

			int begin1 = solution[begin1pos];
			int end2 = solution[end2pos];

			int delta = weights[end1][end2] + weights[begin2][begin1]
				- (weights[end1][begin2] + weights[end2][begin1]);

			for (int p = begin2pos; p != end2pos; p = NextPos(p))
				delta -= Weight(solution[p], solution[NextPos(p)])
					- Weight(solution[NextPos(p)], solution[p]);

			if (delta < 0)
			{
				int p1 = begin2pos;
				int p2 = end2pos;
				int len = RestoreSmallPos(end2pos - begin2pos) + 1;

				for (int i = len / 2; i > 0; i--)
				{
					int temp = solution[p1];
					solution[p1] = solution[p2];
					solution[p2] = temp;
					p1 = NextPos(p1);
					p2 = PrevPos(p2);
				}

				totalDelta += delta;

				end1 = solution[end1pos];
				begin2 = solution[begin2pos];
			}
		}
	}

	return totalDelta;
}

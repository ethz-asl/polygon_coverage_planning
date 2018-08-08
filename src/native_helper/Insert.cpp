#include "mav_coverage_graph_solvers/gk_ma/native_helper/Insert.h"
#include "mav_coverage_graph_solvers/gk_ma/native_helper/NativeHelper.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>

static int *solution;

int InsertDelta(int oldPosition, int newPosition)
{
	int oldPrevV = solution[PrevPos(oldPosition)];
	int oldNextV = solution[NextPos(oldPosition)];
	int newPrevV = oldPosition < newPosition ? solution[newPosition] : solution[PrevPos(newPosition)];
	int newNextV = oldPosition < newPosition ? solution[NextPos(newPosition)] : solution[newPosition];
	int oldV = solution[oldPosition];
	int newV = solution[newPosition];
	return weights[oldPrevV][oldNextV]
		- (weights[oldPrevV][oldV] + weights[oldV][oldNextV])
		+ (weights[newPrevV][oldV] + weights[oldV][newNextV])
		- weights[newPrevV][newNextV];
}

void Insert(int oldPosition, int newPosition)
{
	int oldV = solution[oldPosition];

	if (oldPosition < newPosition)
		for (int i = oldPosition; i < newPosition; i++)
			solution[i] = solution[i + 1];
	else
		for (int i = oldPosition; i > newPosition; i--)
			solution[i] = solution[i - 1];

	solution[newPosition] = oldV;
}


int TryInsertWithClusterOptimisation(int oldPos, int newPos)
{
	int oldV = solution[oldPos];
	int oldPrevPos = PrevPos(oldPos);
	int oldPrev = solution[oldPrevPos];
	int oldNextPos = NextPos(oldPos);
	int oldNext = solution[oldNextPos];

	int newPrevPos = oldPos < newPos ? newPos : PrevPos(newPos);
	int newNextPos = oldPos < newPos ? NextPos(newPos) : newPos;

	//int newPrevPos = PrevPos(newPos);
	int newPrev = solution[newPrevPos];
	//int newNextPos = newPos;
	int newNext = solution[newNextPos];

	int oldWeight = Weight(oldPrev, oldV, oldNext) + Weight(newPrev, newNext);


	InsertWithClusterOptimisationCutsOf++;
	if (Weight(oldPrev, oldNext)
		+ MinClusterDistanceForVertices(newPrev, oldV, newNext)
		>= oldWeight)
	{
		InsertWithClusterOptimisationCuts++;
		return 0;
	}



	int cluster = clusterByVertex[oldV];

	int minL = INT_MAX;
	int bestNewV;
	for (int i = clusterSizes[cluster] - 1; i >= 0; i--)
	{
		int newV = clusters[cluster][i];
		int l = Weight(newPrev, newV, newNext);
		if (l < minL)
		{
			minL = l;
			bestNewV = newV;
		}
	}

	int delta = weights[oldPrev][oldNext] + minL - oldWeight;

	if (delta >= 0)
		return 0;

	//int oldLen = CalculateLen(solution);
	Insert(oldPos, newPos);
	solution[newPos] = bestNewV;
	//int newLen = CalculateLen(solution);

	//if (oldLen + delta != newLen)
	//	printf("");

	int posDelta = abs(oldPos - newPos);
	if (posDelta > clusterCount / 2)
		posDelta = clusterCount - posDelta;

	TotalPosDelta += posDelta;
	TotalPosDeltaCount++;

	return delta;
}

int InsertsWithCO(int *solution)
{
	::solution = solution;

	int totalDelta = 0;

	for (int oldPos = 0; oldPos < clusterCount - 1; oldPos++)
	{
		int oldV = solution[oldPos];
		int oldPrevPos = PrevPos(oldPos);
		int oldPrev = solution[oldPrevPos];
		int oldNextPos = NextPos(oldPos);
		int oldNext = solution[oldNextPos];

		int cluster = clusterByVertex[oldV];

		//int newPrevPos = oldPos < newPos ? newPos : PrevPos(newPos);
		//int newNextPos = oldPos < newPos ? NextPos(newPos) : newPos;

		int oldWeightFirstPartBefore = Weight(oldPrev, oldV, oldNext);
		int oldWeightFirstPartAfter = Weight(oldPrev, oldNext);

		for (int newPos = 0; newPos < oldPos - 1; newPos++)
		{
			int newPrevPos = PrevPos(newPos);
			int newNextPos = newPos;

			int newPrev = solution[newPrevPos];
			int newNext = solution[newNextPos];

			int oldWeight = oldWeightFirstPartBefore + Weight(newPrev, newNext);
			if (oldWeightFirstPartAfter + MinClusterDistanceForVertices(newPrev, oldV, newNext)	>= oldWeight)
				continue;

			int minL = INT_MAX;
			int bestNewV;
			for (int i = clusterSizes[cluster] - 1; i >= 0; i--)
			{
				int newV = clusters[cluster][i];
				int l = Weight(newPrev, newV, newNext);
				if (l < minL)
				{
					minL = l;
					bestNewV = newV;
				}
			}

			int delta = oldWeightFirstPartAfter + minL - oldWeight;
			if (delta >= 0)
				continue;

			Insert(oldPos, newPos);
			solution[newPos] = bestNewV;

			oldV = solution[oldPos];
			oldPrev = solution[oldPrevPos];
			oldNext = solution[oldNextPos];
			cluster = clusterByVertex[oldV];
			oldWeightFirstPartBefore = Weight(oldPrev, oldV, oldNext);
			oldWeightFirstPartAfter = Weight(oldPrev, oldNext);

			totalDelta += delta;
		}

		for (int newPos = oldPos + 1; newPos < clusterCount - 1; newPos++)
		{
			int newPrevPos = newPos;
			int newNextPos = newPos + 1;

			int newPrev = solution[newPrevPos];
			int newNext = solution[newNextPos];

			int oldWeight = oldWeightFirstPartBefore + Weight(newPrev, newNext);
			if (oldWeightFirstPartAfter + MinClusterDistanceForVertices(newPrev, oldV, newNext)	>= oldWeight)
				continue;

			int minL = INT_MAX;
			int bestNewV;
			for (int i = clusterSizes[cluster] - 1; i >= 0; i--)
			{
				int newV = clusters[cluster][i];
				int l = Weight(newPrev, newV, newNext);
				if (l < minL)
				{
					minL = l;
					bestNewV = newV;
				}
			}

			int delta = oldWeightFirstPartAfter + minL - oldWeight;
			if (delta >= 0)
				continue;

			Insert(oldPos, newPos);
			solution[newPos] = bestNewV;

			oldV = solution[oldPos];
			oldPrev = solution[oldPrevPos];
			oldNext = solution[oldNextPos];
			cluster = clusterByVertex[oldV];
			oldWeightFirstPartBefore = Weight(oldPrev, oldV, oldNext);
			oldWeightFirstPartAfter = Weight(oldPrev, oldNext);

			totalDelta += delta;
		}
	}

	return totalDelta;
}

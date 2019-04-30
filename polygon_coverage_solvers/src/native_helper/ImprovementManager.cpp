#include <stdlib.h>
#include <stdio.h>
#include "polygon_coverage_solvers/gk_ma/native_helper/ImprovementManager.h"
#include "polygon_coverage_solvers/gk_ma/native_helper/Swap.h"
#include "polygon_coverage_solvers/gk_ma/native_helper/Insert.h"
#include "polygon_coverage_solvers/gk_ma/native_helper/KOpt.h"
#include "polygon_coverage_solvers/gk_ma/native_helper/ClusterOptimisation.h"

struct LocalSearchHeuristic
{
	int (*heuristic)(int *solution);
	int heuristicId;
	int coverHeuristicIds;
};

enum
{
	FullSwapId = 1,
	InsertsWithCoId = 2,
	DirectTwoOptId = 4,
	TwoOptId = 8,
	NeighbourSwapWithCoId = 16,
	ThreeNeighbourFullSwapId = 32,
	FourNeighbourFullSwapId = 64,
	ClusterOptimisationId = 128,
};

LocalSearchHeuristic asymmetricH[] =
{
	{ FullSwap,					FullSwapId,					FullSwapId },
	{ InsertsWithCO,			InsertsWithCoId,			InsertsWithCoId },
	{ DirectTwoOptAsym,			DirectTwoOptId,				DirectTwoOptId | TwoOptId },
	{ TwoOptFullAsym,			TwoOptId,					TwoOptId },
	{ NeighbourSwapWithCO,		NeighbourSwapWithCoId,		NeighbourSwapWithCoId },
	{ ThreeNeighbourFullSwap,	ThreeNeighbourFullSwapId,	ThreeNeighbourFullSwapId },
};

LocalSearchHeuristic symmetricH[] =
{
	{ InsertsWithCO,			InsertsWithCoId,			InsertsWithCoId },
	{ DirectTwoOptSym,			DirectTwoOptId,				DirectTwoOptId | TwoOptId },
	{ TwoOptFullSym,			TwoOptId,					TwoOptId },
	{ NeighbourSwapWithCO,		NeighbourSwapWithCoId,		NeighbourSwapWithCoId },
	{ ThreeNeighbourFullSwap,	ThreeNeighbourFullSwapId,	ThreeNeighbourFullSwapId },
	{ FourNeighbourFullSwap,	FourNeighbourFullSwapId,	FourNeighbourFullSwapId },
};

int Improve(int *solution, int oldLen)
{
	LocalSearchHeuristic *H = isSymmetric ? symmetricH : asymmetricH;

	int onceFailedHeuristics = 0;
	int totalDelta = 0;
	bool idleCycle;
	do
	{
		idleCycle = true;
		for (int i = 0; i < 6; i++)
		{
			if (onceFailedHeuristics & H[i].coverHeuristicIds)
				continue;

			int delta = H[i].heuristic(solution);

			if (delta < 0)
			{
				idleCycle = false;
				totalDelta += delta;
			}
			else
				onceFailedHeuristics |= H[i].heuristicId;
		}
	} while (!idleCycle);

	return ClusterOptimisation(solution);
}

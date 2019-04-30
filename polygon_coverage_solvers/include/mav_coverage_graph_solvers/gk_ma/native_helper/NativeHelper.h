#ifndef __NATIVE_HELPER_H
#define __NATIVE_HELPER_H

#include "Swap.h"
#include "ClusterOptimisation.h"
#include "KOpt.h"
#include "Insert.h"

extern "C"
{
	__attribute__ ((visibility ("default"))) bool InitProblem(int vertexCount, int clusterCount, int *weights, int *clusterSizes, int **clusters, bool isSymmetric);
	__attribute__ ((visibility ("default"))) bool DumbCounters(char *fileName);
}

extern int clusterCount;
extern int vertexCount;
extern bool isSymmetric;
extern int clusterSizes[];
extern int *clusters[];
extern int *weights[];

extern int clusterByVertex[];
extern int *minClusterDistances[];

#define MAX_CLUSTER_COUNT 1024
#define MAX_VERTEX_COUNT (MAX_CLUSTER_COUNT * 5)
#define MAX_CLUSTER_SIZE 128

extern int RandomPosition();
extern int PrevPos(int pos);
extern int NextPos(int pos);
extern int RestoreLargePos(int largePos);
extern int RestoreSmallPos(int smallPos);

int CalculateLen(int *solution);

extern int Weight(int v1, int v2);
extern int Weight(int v1, int v2, int v3);
extern int Weight(int v1, int v2, int v3, int v4);
extern int Weight(int v1, int v2, int v3, int v4, int v5);
extern int Weight(int v1, int v2, int v3, int v4, int v5, int v6);
extern int MinClusterDistance(int c1, int c2);
extern int MinClusterDistance(int c1, int c2, int c3);
extern int MinClusterDistance(int c1, int c2, int c3, int c4);
extern int MinClusterDistance(int c1, int c2, int c3, int c4, int c5);
extern int MinClusterDistance(int c1, int c2, int c3, int c4, int c5, int c6);
extern int MinClusterDistanceForVertices(int v1, int v2);
extern int MinClusterDistanceForVertices(int v1, int v2, int v3);
extern int MinClusterDistanceForVertices(int v1, int v2, int v3, int v4);


extern int InsertWithClusterOptimisationCuts;
extern int InsertWithClusterOptimisationCutsOf;

extern int TotalPosDelta;
extern int TotalPosDeltaCount;

#endif

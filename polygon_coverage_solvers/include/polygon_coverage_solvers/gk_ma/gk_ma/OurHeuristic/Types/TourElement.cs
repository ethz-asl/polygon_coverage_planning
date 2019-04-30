using GkMa.Loader;

namespace GkMa.OurHeuristic.Types
{
	public struct TourElement
	{
		private readonly byte clusterIndex;
		private byte vertexInCluster;

		public TourElement(int clusterIndex, int vertexInCluster)
		{
			this.clusterIndex = (byte)clusterIndex;
			this.vertexInCluster = (byte)vertexInCluster;
		}

		public int ClusterIndex
		{
			get { return clusterIndex; }
		}

		public int VertexInCluster
		{
			get { return vertexInCluster; }
			set { vertexInCluster = (byte)value; }
		}

		public int GetVertexIndex(Task task)
		{
			return task[clusterIndex][vertexInCluster];
		}
	}
}
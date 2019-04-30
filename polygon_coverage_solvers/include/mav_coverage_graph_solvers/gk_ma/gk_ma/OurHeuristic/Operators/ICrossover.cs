using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	public interface ICrossover
	{
		int[] Cross(Tour parent1, Tour parent2);
	}
}
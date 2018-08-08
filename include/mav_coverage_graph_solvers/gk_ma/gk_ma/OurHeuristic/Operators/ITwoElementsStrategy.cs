using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	public interface ITwoElementsStrategy
	{
		void Run(Generation generation, out int index1, out int index2);
	}
}
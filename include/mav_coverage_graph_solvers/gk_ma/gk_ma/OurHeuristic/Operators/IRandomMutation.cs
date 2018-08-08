using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	public interface IRandomMutation
	{
		Tour Run(Tour source);
	}
}
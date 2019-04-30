using GkMa.OurHeuristic.Types;

namespace GkMa.OurHeuristic.Operators
{
	internal interface IOneElementStrategy
	{
		void Run(Generation generation, out int index);
	}
}
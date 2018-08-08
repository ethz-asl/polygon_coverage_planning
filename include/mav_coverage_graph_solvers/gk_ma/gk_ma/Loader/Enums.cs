namespace GkMa.Loader
{
	internal enum MatrixFormat
	{
		FullMatrix,
		UpperRow,
		LowerRow,
		UpperDiagRow,
		LowerDiagRow,
		UpperDiagCol,
		LowerDiagCol,
	}

	public enum EdgeWeightType
	{
		Explicit,
		Euc2D,
		Euc3D,
		Geo,
		Att,
		Ceil2D
	}

	internal enum CoordType
	{
		None,
		Twod,
		Threed,
	}
}
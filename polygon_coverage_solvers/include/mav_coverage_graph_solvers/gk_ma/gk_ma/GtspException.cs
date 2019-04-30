using System;

namespace GkMa
{
	public class GtspException : Exception
	{
		public GtspException(string message)
			: base(message)
		{
		}

		public GtspException(string format, params object[] prm)
			: base(string.Format(format, prm))
		{
		}
	}
}
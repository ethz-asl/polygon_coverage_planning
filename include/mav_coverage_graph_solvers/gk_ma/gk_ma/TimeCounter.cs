using System;
using System.Runtime.InteropServices;
using FILETIME=System.Runtime.InteropServices.ComTypes.FILETIME;

namespace GkMa
{
	/// <summary>
	/// Summary description for TimeCounter.
	/// </summary>
	public class TimeCounter
	{
		private static ulong FileTimeToLong(FILETIME time)
		{
			ulong h = (ulong)time.dwHighDateTime;
			ulong l = (ulong)time.dwLowDateTime;
			return (h << 32) | l;
		}

		private static ulong GetCurTime()
		{
			FILETIME lpCreationTime, lpExitTime, lpKernelTime, lpUserTime;

			if (!GetThreadTimes(
			     	GetCurrentThread(),
			     	out lpCreationTime,
			     	out lpExitTime,
			     	out lpKernelTime,
			     	out lpUserTime
			     	))
			{
				throw new Exception("Вызов функции GetThreadTimes завершился неуспехом.");
			}

			return FileTimeToLong(lpUserTime);
		}

		[DllImport("Kernel32.dll")]
		private static extern IntPtr GetCurrentThread();

		[DllImport("Kernel32.dll")]
		private static extern bool GetThreadTimes(
			IntPtr hProcess,
			out FILETIME lpCreationTime,
			out FILETIME lpExitTime,
			out FILETIME lpKernelTime,
			out FILETIME lpUserTime
			);

		private ulong start, stop;

		public void Start()
		{
			start = GetCurTime();
		}

		public void Stop()
		{
			stop = GetCurTime();
		}

		public double Seconds
		{
			get { return (stop - start) * 1e-7; }
		}

		public TimeSpan Interval
		{
			get
			{
				TimeSpan span = new TimeSpan((uint)((long)stop - (long)start));
				if (span.TotalMilliseconds < 0)
					throw new GtspException("duration < 0");
				return span;
			}
		}
	}
}
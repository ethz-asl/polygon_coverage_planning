using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text.RegularExpressions;

namespace GkMa
{
	public static class Helper
	{
		private static Random random = new Random(123456);

		public static float Sqr(float x)
		{
			return x * x;
		}

		public static float Rand(float min, float max)
		{
			return min + (float)random.NextDouble() * (max - min);
		}

		public static int RandInt(int min, int max)
		{
			return random.Next(min, max + 1);
		}

		public static int RandIntLess(int min, int max)
		{
			return random.Next(min, max);
		}

		public static int RandIntLess(int max)
		{
			return random.Next(0, max);
		}


		public static readonly DirectoryInfo ProgramDirectory =
			new FileInfo(Assembly.GetExecutingAssembly().Location).Directory;

		public static DirectoryInfo GetFilesDirectory(string name)
		{
			DirectoryInfo[] directories = ProgramDirectory.GetDirectories(name);
			switch (directories.Length)
			{
				case 0:
					return ProgramDirectory.CreateSubdirectory(name);

				case 1:
					return directories[0];

				default:
					throw new GtspException("Several directories with the same name found");
			}
		}

		public static FileInfo[] GetFiles(params string[] directoryNames)
		{
			List<FileInfo> list = new List<FileInfo>();
			foreach (string directoryName in directoryNames)
				list.AddRange(GetFilesDirectory(directoryName).GetFiles());

			list.Sort(delegate(FileInfo x, FileInfo y)
			          	{
			          		int vertexCountX, vertexCountY;
			          		string nameX, nameY;
			          		ParseTspFileName(x.Name, out vertexCountX, out nameX);
			          		ParseTspFileName(y.Name, out vertexCountY, out nameY);
			          		return vertexCountX.CompareTo(vertexCountY);
			          	});

			return list.ToArray();
		}

		public static void ParseTspFileName(string fileName, out int vertexCount, out string name)
		{
			Match match = Regex.Match(fileName, @"^(\w+?)(\d+)p?\.\w+$");
			if (!match.Success)
				throw new GtspException("Wrong GTSP file name: {0}", fileName);

			name = match.Groups[1].Value;
			vertexCount = int.Parse(match.Groups[2].Value);
		}

		public static void ParseGtspFileName(string fileName, out int clusterCount, out int vertexCount)
		{
			Match match = Regex.Match(fileName, @"^(\d+)\w+?(\d+)\.\w+$");
			if (!match.Success)
				throw new GtspException("Wrong GTSP file name: {0}", fileName);

			clusterCount = int.Parse(match.Groups[1].Value);
			vertexCount = int.Parse(match.Groups[2].Value);
		}

		public static string GetFullFileName(string fileName)
		{
			return ProgramDirectory.FullName + '/' + fileName;
		}

		public static void ResetRand()
		{
			random = new Random(123);
		}
	}
}
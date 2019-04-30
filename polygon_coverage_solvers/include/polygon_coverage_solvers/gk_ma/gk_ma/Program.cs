using System;
using System.IO;

namespace GkMa
{
	internal static class Program
	{
		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		[STAThread]
		private static void Main()
		{
			SolveUsingOurMa(binary: true);
		}

		private static readonly string[] problemNames
			= new string[]
			  	{
			  		"40d198",
			  		"40kroa200",
			  		"40krob200",
			  	};

		public static void SolveUsingOurMa(bool binary)
		{
			foreach (string problemName in problemNames)
			{
				//try
				{
					FileInfo fileInfo = new FileInfo(Helper.GetFullFileName("GTSP/" + problemName + (binary ? ".gtsp" : ".txt")));

					int vc, cc;
					Helper.ParseGtspFileName(fileInfo.Name, out cc, out vc);

					Helper.ResetRand();

					string instanceName = fileInfo.Name.Substring(0, fileInfo.Name.Length - fileInfo.Extension.Length);
					Console.Title = instanceName;
					for (int i = 0; i < 10; i++)
					{
						Console.Title += ".";
						OurSolver solver = new OurSolver(fileInfo.FullName, binary);
						solver.Solve();

						using (StreamWriter writer = new StreamWriter(Helper.GetFullFileName("solved.txt"), true))
						{
							writer.Write(
                                "{0}\t{1}\t{2}\t{3}\t{4} Solution:",
								DateTime.Now,
								instanceName,
								solver.Len,
								solver.Milliseconds,
                                solver.GenerationCount);

                            foreach (int vertex in solver.Solution)
                            {
                                writer.Write(" " + vertex);
                            }
                            writer.WriteLine("");
						}
					}
				}
/*
				catch (FileNotFoundException ex)
				{
					Console.ForegroundColor = ConsoleColor.Yellow;
					Console.WriteLine("File not found: \"{0}\"\nProbably, the GTSP instance file is not present at the required location.\n", ex.Message);
					Console.ForegroundColor = ConsoleColor.White;
				}
				catch (Exception ex)
				{
					Console.ForegroundColor = ConsoleColor.Red;
					Console.WriteLine("Unhandled exception: {0}.\n", ex.Message);
					Console.ForegroundColor = ConsoleColor.White;
					break;
				}
*/
			}
		}
	}
}

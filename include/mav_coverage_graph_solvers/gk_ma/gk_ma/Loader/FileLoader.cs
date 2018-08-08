using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;

namespace GkMa.Loader
{
	public delegate double CountWeightHandler(Vertex from, Vertex to);

	public class FileLoader
	{
		#region Private fields

		private string name;
		private string comment;
		private int vertexCount;
		private int clusterCount;
		private CoordType coordType;
		private EdgeWeightType weightType;
		private Vertex[] vertexes;
		private int[][] clusters;
		private int[][] weights;
		private MatrixFormat weigthFormat;
		private int bestSolution;

		#endregion

		#region Generation

		public FileLoader(string fileName)
		{
			Load(fileName);
		}

		private void Load(string fileName)
		{
			try
			{
				using (StreamReader reader = new StreamReader(fileName))
				{
					string line = reader.ReadLine();
					while (line != null && !CheckValue(line, "EOF"))
					{
						int colonPos = line.IndexOf(':');
						string value;
						string sectionName;
						if (colonPos < 0)
						{
							sectionName = line.Trim();
							value = null;
						}
						else
						{
							sectionName = line.Substring(0, colonPos).Trim();
							value = line.Substring(colonPos + 1).Trim();
						}

						switch (sectionName)
						{
							case "BEST_SOLUTION":
								bestSolution = int.Parse(value);
								break;

							case "NAME":
								name = value;
								break;

							case "COMMENT":
								comment = value;
								break;

							case "DIMENSION":
								vertexCount = int.Parse(value);
								break;

							case "GTSP_SETS":
								clusterCount = int.Parse(value);
								break;

							case "EDGE_WEIGHT_TYPE":
								ParseWeightType(value);
								break;

							case "TYPE":
								/*								switch (value)
																{
																	case "TSP":
																	case "GTSP":
																	case "AGTSP":
																		break;

																	default:
																		throw new GtspException("Only TSP, GTSP and AGTSP problems are supported ({0})", value);
																}*/
								break;

							case "EDGE_WEIGHT_FORMAT":
								ParseWeightFormat(value);
								break;

							case "NODE_COORD_TYPE":
								ParseCoordType(value);
								break;

							case "EDGE_WEIGHT_SECTION":
								LoadExplicitWeights(reader);
								break;

							case "NODE_COORD_SECTION":
								LoadVertexes(reader);
								break;

							case "GTSP_SET_SECTION":
								LoadGtspSection(reader);
								break;

							case "DISPLAY_DATA_TYPE":
								ParseDisplayDataType(value);
								break;

							case "DISPLAY_DATA_SECTION":
								LoadDisplayData(reader);
								break;

							case "FIXED_EDGES_SECTION":
								LoadFixedEdges(reader);
								break;

							case "":
								break;

							default:
								throw new GtspException("The section {0} is unknown", sectionName);
						}

						line = reader.ReadLine();
					}
				}

				if (clusterCount == 0)
					clusterCount = (vertexCount + 4) / 5;
			}
			catch (Exception exception)
			{
				throw new GtspException(
					"The task can not be read from file {0} because of the error {1}",
					fileName, exception);
			}
		}

		private void CountWeights()
		{
			switch (weightType)
			{
				case EdgeWeightType.Explicit:
					break;

				case EdgeWeightType.Euc2D:
					CountWeights(Euc2DWeight);
					break;

				case EdgeWeightType.Euc3D:
					CountWeights(Euc3DWeight);
					break;

				case EdgeWeightType.Geo:
					CountWeights(GeoWeight);
					break;

				case EdgeWeightType.Att:
					CountWeights(AttWeight);
					break;

				case EdgeWeightType.Ceil2D:
					CountWeights(Ceil2DWeight);
					break;

				default:
					throw new GtspException("Unsupported weight type: {0}", weightType);
			}

			if (clusters == null)
			{
				clusterCount = (vertexCount + 4) / 5;
				Clusterizater clusterizater = new Clusterizater(vertexCount, clusterCount, GetWeight);
				clusters = clusterizater.Clusterize();
			}
		}

		private static void LoadFixedEdges(TextReader reader)
		{
			string line;
			do
			{
				line = reader.ReadLine();
			} while (line.Trim() != "-1");
		}

		private void LoadDisplayData(TextReader reader)
		{
			for (int i = 0; i < vertexCount; i++)
				reader.ReadLine();
		}

		private void FillWeights(IEnumerator<Point> enumerator, TextReader reader)
		{
			while (enumerator.MoveNext())
				weights[enumerator.Current.Y][enumerator.Current.X] = (int)(ReadValue(reader) + 0.5);
		}

		private void CountWeights(CountWeightHandler counter)
		{
			if (vertexes == null)
				throw new GtspException("Vertices were not specified.");

			weights = new int[vertexes.Length][];
			for (int from = 0; from < vertexes.Length; from++)
			{
				weights[from] = new int[vertexes.Length];
				Vertex vertexesFrom = vertexes[from];
				for (int to = 0; to < vertexes.Length; to++)
				{
					Vertex vertexesTo = vertexes[to];
					double d = counter(vertexesFrom, vertexesTo);
					weights[from][to] = (int)(d + 0.5);
				}
			}
		}

		public Task Generate()
		{
			Point[] screenPositions = null;
			if (vertexes != null)
			{
				screenPositions = new Point[vertexCount];
				for (int i = 0; i < vertexCount; i++)
					screenPositions[i] = new Point((int)vertexes[i].X, (int)vertexes[i].Y);
			}

			EnsureWeights();
			Task task = new Task(weights, clusters, screenPositions, IsSymmetric);
			task.BestSolution = BestSolution;
			return task;
		}

		private void EnsureWeights()
		{
			if (weights == null || clusters == null)
				CountWeights();
		}

		#endregion

		#region Weight functions

		private int GetWeight(int from, int to)
		{
			if (from == to)
				return 0;

			return weights[from][to];
		}

		private static double Euc2DWeight(Vertex from, Vertex to)
		{
			return Math.Sqrt(
				Helper.Sqr(from.X - to.X) +
				Helper.Sqr(from.Y - to.Y));
		}

		private static double Euc3DWeight(Vertex from, Vertex to)
		{
			return Math.Sqrt(
				Helper.Sqr(from.X - to.X) +
				Helper.Sqr(from.Y - to.Y) +
				Helper.Sqr(from.Z - to.Z));
		}

		private static double Ceil2DWeight(Vertex from, Vertex to)
		{
			return Math.Ceiling(Math.Sqrt(
			                    	Helper.Sqr(from.X - to.X) +
			                    	Helper.Sqr(from.Y - to.Y)));
		}

		#region Geo

		private const double PI = 3.141592;
		private const double RRR = 6378.388;

		private static double ConvertGeoValue(double x)
		{
			int deg = (int)x;
			double min = x - deg;
			return PI * (deg + 5.0 * min / 3.0) / 180.0;
		}

		private static double GeoWeight(Vertex from, Vertex to)
		{
			double longitudeFrom = ConvertGeoValue(from.Y);
			double latitudeFrom = ConvertGeoValue(from.X);
			double longitudeTo = ConvertGeoValue(to.Y);
			double latitudeTo = ConvertGeoValue(to.X);
			double q1 = Math.Cos(longitudeFrom - longitudeTo);
			double q2 = Math.Cos(latitudeFrom - latitudeTo);
			double q3 = Math.Cos(latitudeFrom + latitudeTo);
			return (int)(RRR * Math.Acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);
		}

		private static double AttWeight(Vertex from, Vertex to)
		{
			float dx = from.X - to.X;
			float dy = from.Y - to.Y;
			float rij = (float)Math.Sqrt((Helper.Sqr(dx) + Helper.Sqr(dy)) / 10);
			int tij = (int)(rij + 0.5f);
			return tij < rij
			       	? tij + 1
			       	: tij;
		}

		#endregion

		#endregion

		#region Parsing

		private static bool CheckValue(string line, string value)
		{
			return line.Trim() == value;
		}

		private void ParseWeightFormat(string text)
		{
			switch (text)
			{
				case "FULL_MATRIX":
					weigthFormat = MatrixFormat.FullMatrix;
					break;

				case "LOWER_DIAG_COL":
					weigthFormat = MatrixFormat.LowerDiagCol;
					break;

				case "UPPER_ROW":
					weigthFormat = MatrixFormat.UpperRow;
					break;

				case "UPPER_DIAG_ROW":
					weigthFormat = MatrixFormat.UpperDiagRow;
					break;

				case "LOWER_DIAG_ROW":
					weigthFormat = MatrixFormat.LowerDiagRow;
					break;

				case "FUNCTION":
					break;

				default:
					throw new GtspException("{0} edge weight type is not supported.", text);
			}
		}

		private static float[] ParseFloats(string line)
		{
			string[] strs = line.Trim().Split(new char[] {' '}, StringSplitOptions.RemoveEmptyEntries);
			float[] result = new float[strs.Length];
			try
			{
				for (int i = 0; i < strs.Length; i++)
				{
					/*
										int dotPos = strs[i].IndexOfAny(new char[] { ',', '.' });
										if (dotPos < 0)
											dotPos = strs[i].Length;
										result[i] = int.Parse(strs[i].Substring(0, dotPos));
					*/
					result[i] = float.Parse(strs[i].Replace(',', '.'));
				}
			}
			catch
			{
				throw new GtspException("Unrecognized line: {0}", line);
			}

			return result;
		}

		private static int[] ParseInts(string line)
		{
			string[] strs = line.Trim().Split(separators, StringSplitOptions.RemoveEmptyEntries);
			int[] result = new int[strs.Length];
			try
			{
				for (int i = 0; i < strs.Length; i++)
					result[i] = int.Parse(strs[i]);
			}
			catch
			{
				throw new GtspException("Unrecognized line: {0}", line);
			}

			return result;
		}

		private void ParseWeightType(string text)
		{
			switch (text)
			{
				case "EUC_2D":
					weightType = EdgeWeightType.Euc2D;
					coordType = CoordType.Twod;
					break;

				case "EUC_3D":
					weightType = EdgeWeightType.Euc3D;
					coordType = CoordType.Threed;
					break;

				case "GEO":
					weightType = EdgeWeightType.Geo;
					coordType = CoordType.Twod;
					break;

				case "EXPLICIT":
					weightType = EdgeWeightType.Explicit;
					break;

				case "ATT":
					weightType = EdgeWeightType.Att;
					break;

				case "CEIL_2D":
					weightType = EdgeWeightType.Ceil2D;
					break;

				default:
					throw new GtspException("Unknown edge weight type: " + text);
			}
		}

		private static void ParseDisplayDataType(string text)
		{
			switch (text)
			{
				case "NO_DISPLAY":
				case "COORD_DISPLAY":
				case "TWOD_DISPLAY":
					break;
				default:
					throw new GtspException("Display data type {0} is not supported", text);
			}
		}

		private void ParseCoordType(string text)
		{
			switch (text)
			{
				case "TWOD_COORDS":
					coordType = CoordType.Twod;
					break;

				case "THREED_COORDS":
					coordType = CoordType.Threed;
					break;

				case "NO_COORDS":
					coordType = CoordType.None;
					break;

				default:
					throw new GtspException("Unknown coord type {0}", text);
			}
		}

		#endregion

		#region Loading sections

		private void LoadGtspSection(TextReader reader)
		{
			clusters = new int[clusterCount][];
			for (int i = 0; i < clusterCount; i++)
			{
				List<int> list = new List<int>(ParseInts(reader.ReadLine()));
				if (list[0] != i + 1)
					throw new GtspException("Wrong {0} cluster number {1}", i, list[0]);
				if (list[list.Count - 1] != -1)
					throw new GtspException("Last value in cluster description is {0}", list[list.Count - 1]);
				list.RemoveAt(list.Count - 1);
				list.RemoveAt(0);
				for (int j = 0; j < list.Count; j++)
				{
					list[j]--;
				}
				clusters[i] = list.ToArray();
			}
		}

		private void LoadVertexes(TextReader reader)
		{
			if (coordType == CoordType.None)
				coordType = CoordType.Twod;

			vertexes = new Vertex[vertexCount];
			for (int i = 0; i < vertexCount; i++)
			{
				float[] lineValues = ParseFloats(reader.ReadLine());
				if (lineValues[0] != i + 1)
					throw new GtspException("The vertex {0} has wrong number {1}", i, lineValues[0]);
				switch (coordType)
				{
					case CoordType.Twod:
						vertexes[i] = new Vertex(lineValues[1], lineValues[2]);
						break;
					case CoordType.Threed:
						vertexes[i] = new Vertex(lineValues[1], lineValues[2], lineValues[3]);
						break;
					default:
						throw new GtspException("Wrong coord type: {0}", coordType);
				}
			}
		}

		private void LoadExplicitWeights(TextReader reader)
		{
			weights = new int[vertexCount][];
			for (int i = 0; i < vertexCount; i++)
				weights[i] = new int[vertexCount];

			switch (weigthFormat)
			{
				case MatrixFormat.FullMatrix:
					FillWeights(EnumerateFullMatrix(), reader);
					break;

				case MatrixFormat.LowerDiagRow:
					FillWeights(EnumerateLowerDiagRow(), reader);
					for (int from = 0; from < vertexCount; from++)
						for (int to = from + 1; to < vertexCount; to++)
							weights[from][to] = weights[to][from];
					break;

				case MatrixFormat.UpperRow:
					FillWeights(EnumerateUpperRow(), reader);
					for (int from = 0; from < vertexCount; from++)
						for (int to = 0; to < from; to++)
							weights[from][to] = weights[to][from];
					break;

				case MatrixFormat.UpperDiagRow:
					FillWeights(EnumerateUpperDiagRow(), reader);
					for (int from = 0; from < vertexCount; from++)
						for (int to = 0; to < from; to++)
							weights[from][to] = weights[to][from];
					break;

					//case EdgeWeigthFormat.UpperDiagCol:
					//	break;
					//case EdgeWeigthFormat.LowerDiagCol:
					//	break;
				default:
					throw new GtspException("Unsupported weight format: {0}", weigthFormat);
			}
		}

		#endregion

		#region Enumerators for matrix

		private IEnumerator<Point> EnumerateFullMatrix()
		{
			for (int from = 0; from < vertexCount; from++)
				for (int to = 0; to < vertexCount; to++)
					yield return new Point(to, from);
		}

		private IEnumerator<Point> EnumerateLowerDiagRow()
		{
			for (int from = 0; from < vertexCount; from++)
				for (int to = 0; to <= from; to++)
					yield return new Point(to, from);
		}

		private IEnumerator<Point> EnumerateUpperRow()
		{
			for (int from = 0; from < vertexCount; from++)
				for (int to = from + 1; to < vertexCount; to++)
					yield return new Point(to, from);
		}

		private IEnumerator<Point> EnumerateUpperDiagRow()
		{
			for (int from = 0; from < vertexCount; from++)
				for (int to = from; to < vertexCount; to++)
					yield return new Point(to, from);
		}

		#endregion

		#region Reading numbers

		private int curPosition;
		private double[] curLine = new double[0];
		private static readonly char[] separators = new char[] {' '};

		private double ReadValue(TextReader reader)
		{
			while (curPosition >= curLine.Length)
			{
				curPosition = 0;
				string[] values = reader.ReadLine().Split(separators, StringSplitOptions.RemoveEmptyEntries);
				curLine = new double[values.Length];
				for (int i = 0; i < values.Length; i++)
					curLine[i] = double.Parse(values[i]);
			}

			return curLine[curPosition++];
		}

		#endregion

		#region Properties

		public EdgeWeightType WeightType
		{
			get { return weightType; }
		}

		public int BestSolution
		{
			get { return bestSolution; }
		}

		public double CanonicalLength
		{
			get
			{
				EnsureWeights();
				int len = 0;
				for (int i = 0; i < vertexCount - 1; i++)
					len += weights[i][i + 1];

				len += weights[vertexCount - 1][0];
				return len;
			}
		}

		public string Comment
		{
			get { return comment; }
		}

		public string Name
		{
			get { return name; }
		}


		public int VertexCount
		{
			get { return vertexCount; }
		}

		public int ClusterCount
		{
			get { return clusterCount; }
		}

		public bool IsSymmetric
		{
			get { return WeightType != EdgeWeightType.Explicit || weigthFormat != MatrixFormat.FullMatrix; }
		}

		#endregion
	}
}
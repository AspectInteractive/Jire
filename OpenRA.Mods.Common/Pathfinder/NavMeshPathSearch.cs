using System;
using System.Collections.Generic;
using System.Linq;
using OpenRA.Mods.Common.Activities;
using OpenRA.Mods.Common.HitShapes;
using OpenRA.Mods.Common.Traits;
using OpenRA.Primitives;
using OpenRA.Traits;
using static OpenRA.Mods.Common.Traits.MobileOffGrid;

namespace OpenRA.Mods.Common.Pathfinder
{
	public class NavMeshPathSearch : BaseOffGridPathSearch, IDisposable
	{

		private readonly NavMesh navMesh;
		private bool disposed;

		private class NavTriangle
		{
			public int Id { get; }
			public WPos[] Vertices { get; }
			public List<int> Neighbors { get; }
			public WPos Center { get; }

			public NavTriangle(int id, WPos v0, WPos v1, WPos v2)
			{
				Id = id;
				Vertices = new[] { v0, v1, v2 };
				Neighbors = new List<int>();
				Center = new WPos(
					(v0.X + v1.X + v2.X) / 3,
					(v0.Y + v1.Y + v2.Y) / 3
				);
			}

			public bool ContainsPoint(WPos point)
			{
				var v0 = Vertices[2] - Vertices[0];
				var v1 = Vertices[1] - Vertices[0];
				var v2 = point - Vertices[0];

				var dot00 = (Fix64)v0.X * (Fix64)v0.X + (Fix64)v0.Y * (Fix64)v0.Y;
				var dot01 = (Fix64)v0.X * (Fix64)v1.X + (Fix64)v0.Y * (Fix64)v1.Y;
				var dot02 = (Fix64)v0.X * (Fix64)v2.X + (Fix64)v0.Y * (Fix64)v2.Y;
				var dot11 = (Fix64)v1.X * (Fix64)v1.X + (Fix64)v1.Y * (Fix64)v1.Y;
				var dot12 = (Fix64)v1.X * (Fix64)v2.X + (Fix64)v1.Y * (Fix64)v2.Y;

				var denom = dot00 * dot11 - dot01 * dot01;
				if (denom == Fix64.Zero) return false;

				var invDenom = Fix64.One / denom;
				var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
				var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

				return (u > Fix64.Zero) && (v > Fix64.Zero) && (u + v < Fix64.One);
			}

			public WPos GetConnectionPoint(int neighborId)
			{
				int neighborIndex = Neighbors.IndexOf(neighborId);
				if (neighborIndex == -1) return Center;

				var v1 = Vertices[neighborIndex];
				var v2 = Vertices[(neighborIndex + 1) % 3];
				return new WPos((v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2);
			}

			public Fix64 DistanceTo(NavTriangle other)
			{
				var dx = (Fix64)(Center.X - other.Center.X);
				var dy = (Fix64)(Center.Y - other.Center.Y);
				return Fix64.Sqrt(dx * dx + dy * dy);
			}
		}

		private class PathNode : IComparable<PathNode>
		{
			public int TriangleId { get; }
			public Fix64 GCost { get; set; }
			public Fix64 HCost { get; set; }
			public Fix64 FCost => GCost + HCost;
			public PathNode Parent { get; set; }

			public PathNode(int triangleId)
			{
				TriangleId = triangleId;
				GCost = Fix64.Zero;
				HCost = Fix64.Zero;
			}

			public int CompareTo(PathNode other)
			{
				int result = FCost.CompareTo(other.FCost);
				return result != 0 ? result : HCost.CompareTo(other.HCost);
			}
		}

		private class NavMesh
		{
			private readonly Dictionary<int, NavTriangle> triangles;
			private readonly Dictionary<WPos, int> spatialHash;

			public NavMesh()
			{
				triangles = new Dictionary<int, NavTriangle>();
				spatialHash = new Dictionary<WPos, int>();
			}

			public void AddTriangle(NavTriangle triangle)
			{
				triangles[triangle.Id] = triangle;

				// Create spatial hash key by rounding to grid
				var hashKey = new WPos(
					triangle.Center.X / 1024 * 1024,
					triangle.Center.Y / 1024 * 1024
				);
				spatialHash[hashKey] = triangle.Id;
			}

			public void SetNeighbors(int triangleId, params int[] neighborIds)
			{
				if (triangles.TryGetValue(triangleId, out var triangle))
				{
					triangle.Neighbors.Clear();
					triangle.Neighbors.AddRange(neighborIds);
				}
			}

			public int FindTriangleContaining(WPos point)
			{
				// Try spatial hash first
				var hashKey = new WPos(
					point.X / 1024 * 1024,
					point.Y / 1024 * 1024
				);

				if (spatialHash.TryGetValue(hashKey, out int candidateId))
				{
					if (triangles[candidateId].ContainsPoint(point))
						return candidateId;
				}

				// Fallback to linear search
				foreach (var triangle in triangles.Values)
				{
					if (triangle.ContainsPoint(point))
						return triangle.Id;
				}

				return -1;
			}

			public List<WPos> FindPath(WPos start, WPos end)
			{
				int startTriangle = FindTriangleContaining(start);
				int endTriangle = FindTriangleContaining(end);

				if (startTriangle == -1 || endTriangle == -1)
					return new List<WPos>();

				if (startTriangle == endTriangle)
					return new List<WPos> { start, end };

				var openSet = new SortedSet<PathNode>();
				var closedSet = new HashSet<int>();
				var allNodes = new Dictionary<int, PathNode>();

				var startNode = new PathNode(startTriangle);
				allNodes[startTriangle] = startNode;
				openSet.Add(startNode);

				var endTriangleObj = triangles[endTriangle];

				while (openSet.Count > 0)
				{
					var current = openSet.Min;
					openSet.Remove(current);
					closedSet.Add(current.TriangleId);

					if (current.TriangleId == endTriangle)
						return ReconstructPath(current, start, end);

					var currentTriangle = triangles[current.TriangleId];
					foreach (var neighborId in currentTriangle.Neighbors)
					{
						if (closedSet.Contains(neighborId))
							continue;

						var neighborTriangle = triangles[neighborId];
						var tentativeGCost = current.GCost + currentTriangle.DistanceTo(neighborTriangle);

						if (!allNodes.TryGetValue(neighborId, out var neighbor))
						{
							neighbor = new PathNode(neighborId);
							allNodes[neighborId] = neighbor;
						}

						if (tentativeGCost < neighbor.GCost || !openSet.Contains(neighbor))
						{
							neighbor.GCost = tentativeGCost;
							neighbor.HCost = neighborTriangle.DistanceTo(endTriangleObj);
							neighbor.Parent = current;

							if (!openSet.Contains(neighbor))
								openSet.Add(neighbor);
						}
					}
				}

				return new List<WPos>();
			}

			private List<WPos> ReconstructPath(PathNode endNode, WPos start, WPos end)
			{
				var path = new List<WPos> { end };
				var current = endNode;

				while (current.Parent != null)
				{
					var currentTriangle = triangles[current.TriangleId];
					var connectionPoint = currentTriangle.GetConnectionPoint(current.Parent.TriangleId);
					path.Add(connectionPoint);
					current = current.Parent;
				}

				path.Add(start);
				path.Reverse();
				return path;
			}

			public void RebuildFromCorners(World world, IEnumerable<CCPos> corners)
			{
				triangles.Clear();
				spatialHash.Clear();

				var vertices = corners.Select(c => world.Map.WPosFromCCPos(c)).ToList();
				if (vertices.Count < 3) return;

				// Simple triangulation - for production, use Delaunay triangulation
				var triangulatedTriangles = TriangulateSimple(vertices);

				foreach (var triangle in triangulatedTriangles)
					AddTriangle(triangle);

				// Build triangle connectivity
				BuildTriangleConnectivity();
			}

			private List<NavTriangle> TriangulateSimple(List<WPos> vertices)
			{
				var result = new List<NavTriangle>();

				// Simple fan triangulation from first vertex
				for (int i = 1; i < vertices.Count - 1; i++)
				{
					var triangle = new NavTriangle(result.Count, vertices[0], vertices[i], vertices[i + 1]);
					result.Add(triangle);
				}

				return result;
			}

			private void BuildTriangleConnectivity()
			{
				// Build adjacency between triangles that share edges
				foreach (var triangle1 in triangles.Values)
				{
					foreach (var triangle2 in triangles.Values)
					{
						if (triangle1.Id >= triangle2.Id) continue;

						if (SharesEdge(triangle1, triangle2))
						{
							triangle1.Neighbors.Add(triangle2.Id);
							triangle2.Neighbors.Add(triangle1.Id);
						}
					}
				}
			}

			private bool SharesEdge(NavTriangle t1, NavTriangle t2)
			{
				int sharedVertices = 0;
				foreach (var v1 in t1.Vertices)
				{
					foreach (var v2 in t2.Vertices)
					{
						if (v1 == v2)
							sharedVertices++;
					}
				}
				return sharedVertices >= 2;
			}
		}

		public NavMeshPathSearch(Actor self, WPos source, WPos dest, int currDelayToRun = 2)
		: base(self, source, dest, currDelayToRun)
		{
			navMesh = new NavMesh();
			RebuildNavMesh(self.World);
		}

		private void RebuildNavMesh(World world)
		{
			var corners = new HashSet<CCPos>();

			// Collect obstacle corners from blocked cells
			for (var x = 0; x < world.Map.MapSize.X; x++)
			{
				for (var y = 0; y < world.Map.MapSize.Y; y++)
				{
					var cell = new CPos(x, y);
					if (IsCellBlocked(Self, locomotor, cell))
					{
						corners.Add(Map.TopLeftCCPos(cell));
						corners.Add(Map.TopRightCCPos(cell));
						corners.Add(Map.BottomLeftCCPos(cell));
						corners.Add(Map.BottomRightCCPos(cell));
					}
				}
			}

			navMesh.RebuildFromCorners(world, corners);
		}

		public static bool IsCellBlocked(Actor self, Locomotor locomotor, CPos? cell, BlockedByActor check = BlockedByActor.None)
		{
			if (cell == null)
				return true;

			return !locomotor.CanMoveFreelyInto(self, (CPos)cell, SubCell.FullCell, check, self, false);
		}

		public override List<WPos> FindPath(WPos start, WPos goal)
		{
			return navMesh.FindPath(start, goal);
		}

		// IOffGridPathSearch implementation
		public override void Expand(int maxExpansions)
		{
			// For nav mesh, expansion is not incremental like A*
			// The path is calculated in full during FindPath
			Running = false;
		}

		public bool IsPathObservable(WPos sourcePos, WPos destPos, IHitShape unitHitShape, bool useUnitRadius, int neighbours)
		{
			// Check if there's a direct line of sight between two positions
			var startTriangle = navMesh.FindTriangleContaining(sourcePos);
			var endTriangle = navMesh.FindTriangleContaining(destPos);

			// If both points are in the same triangle, they can see each other
			return startTriangle != -1 && startTriangle == endTriangle;
		}

		public void Dispose()
		{
			if (!disposed)
			{
				disposed = true;
			}
		}
	}
}

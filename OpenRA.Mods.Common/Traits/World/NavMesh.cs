using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using OpenRA.Graphics;
using OpenRA.Traits;
using OpenRA.Mods.Common;

namespace OpenRA.Mods.Common.Traits
{
	[TraitLocation(SystemActors.World)]
	[Desc("Generates and manages the navigation mesh for off-grid pathfinding.")]
	public class NavMeshInfo : TraitInfo, Requires<NavMeshPathfinderOverlayInfo>, Requires<LocomotorInfo>
	{
		[Desc("Grid size for spatial hashing optimization.")]
		public readonly int SpatialHashGridSize = 1024;

		public override object Create(ActorInitializer init) => new NavMesh(init.World, this);
	}

	public class NavMesh : IWorldLoaded
	{
		private readonly World world;
		private readonly NavMeshInfo info;
		private Locomotor locomotor;
		private NavMeshPathfinderOverlay overlay;

		// Navigation mesh data
		private List<Triangle> triangles;

		public NavMesh(World world, NavMeshInfo info)
		{
			this.world = world;
			this.info = info;
			triangles = new();
		}

		void IWorldLoaded.WorldLoaded(World w, WorldRenderer wr)
		{
			locomotor = world.WorldActor.TraitsImplementing<Locomotor>().FirstEnabledTraitOrDefault();
			overlay = world.WorldActor.TraitsImplementing<NavMeshPathfinderOverlay>().FirstOrDefault();

			// Build the navigation mesh once when the world loads
			RebuildNavMesh();
		}

		private void RebuildNavMesh()
		{
			triangles.Clear();

			// TEMPORARY TRIANGLE
			var sampleShape = new List<WPos>()
			{
				new(36600, 41000), // C
				new(11000, 28900), // A
				new(43000, 9000), // B
				new(17700, 13000), // E
				new(53800, 9200), // H
				new(55000, 32000), // I
				new(51000, 37000), // J
			};

			// TEMPORARY HOLE FOR CONSTRAINED VERSION
			var sampleHole = new List<WPos>()
			{
				new(32550, 21000), // G
				new(51040, 17800), // L
				new(24740, 27500), // M
				new(49000, -29700), // N
			};

			// Create a proper navigation mesh using Delaunay triangulation
			triangles = TriangulateByFlippingEdges(sampleShape);

			// Create a proper navigation mesh using Constrained Delaunay triangulation (add holes to delauney)
			//triangles = ConstrainedDelaunay.GenerateTriangulation(sampleShape, sampleHole);
			//triangles = ConstrainedDelauneyTriangulation.GenerateTriangulation(sampleShape, sampleHole);

			// Add to overlay
			foreach (var triangle in triangles)
				overlay.AddTriangle(triangle.AsList());
		}

		private bool IsCellBlockedForNavMesh(CPos cell)
		{
			if (!world.Map.Contains(cell))
				return true;

			return MobileOffGrid.CellIsBlockedCache(world.WorldActor, locomotor, cell);
		}

		// From triangle where each triangle has one vertex to half edge
		public static List<HalfEdge> TransformFromTriangleToHalfEdge(List<Triangle> triangles)
		{
			// Orient triangles so they have the correct orientation
			static void OrientTrianglesClockwise(List<Triangle> triangles)
			{
				for (var i = 0; i < triangles.Count; i++)
				{
					var tri = triangles[i];

					var v1 = new WPos(tri.v1.X, tri.v1.Z);
					var v2 = new WPos(tri.v2.X, tri.v2.Z);
					var v3 = new WPos(tri.v3.X, tri.v3.Z);

					if (!IsTriangleOrientedClockwise(v1, v2, v3))
						tri.ChangeOrientation();
				}
			}

			// Make sure the triangles have the same orientation
			OrientTrianglesClockwise(triangles);

			// First create a list with all possible half-edges
			var halfEdges = new List<HalfEdge>(triangles.Count * 3);

			for (var i = 0; i < triangles.Count; i++)
			{
				var t = triangles[i];

				var he1 = new HalfEdge(t.v1);
				var he2 = new HalfEdge(t.v2);
				var he3 = new HalfEdge(t.v3);

				he1.nextEdge = he2;
				he2.nextEdge = he3;
				he3.nextEdge = he1;

				he1.prevEdge = he3;
				he2.prevEdge = he1;
				he3.prevEdge = he2;

				// The vertex needs to know of an edge going from it
				he1.v.halfEdge = he2;
				he2.v.halfEdge = he3;
				he3.v.halfEdge = he1;

				// The face the half-edge is connected to
				t.halfEdge = he1;

				he1.t = t;
				he2.t = t;
				he3.t = t;

				// Add the half-edges to the list
				halfEdges.Add(he1);
				halfEdges.Add(he2);
				halfEdges.Add(he3);
			}

			// Find the half-edges going in the opposite direction
			for (var i = 0; i < halfEdges.Count; i++)
			{
				var he = halfEdges[i];

				var goingToVertex = he.v;
				var goingFromVertex = he.prevEdge.v;

				for (var j = 0; j < halfEdges.Count; j++)
				{
					// Dont compare with itself
					if (i == j)
						continue;

					var heOpposite = halfEdges[j];

					// Is this edge going between the vertices in the opposite direction
					if (goingFromVertex.position == heOpposite.v.position && goingToVertex.position == heOpposite.prevEdge.v.position)
					{
						he.oppositeEdge = heOpposite;
						break;
					}
				}
			}

			return halfEdges;
		}

		// Orient triangles so they have the correct orientation
		public static void OrientTrianglesClockwise(List<Triangle> triangles)
		{
			for (var i = 0; i < triangles.Count; i++)
			{
				var tri = triangles[i];

				var v1 = new WPos(tri.v1.X, tri.v1.Z);
				var v2 = new WPos(tri.v2.X, tri.v2.Z);
				var v3 = new WPos(tri.v3.X, tri.v3.Z);

				if (!IsTriangleOrientedClockwise(v1, v2, v3))
					tri.ChangeOrientation();
			}
		}

		public static float GetDeterminant(WPos p1, WPos p2, WPos p3)
			=> p1.X * p2.Y + p3.X * p1.Y + p2.X * p3.Y - p1.X * p3.Y - p3.X * p2.Y - p2.X * p1.Y;

		// Is a triangle in 2d space oriented clockwise or counter-clockwise
		// https://math.stackexchange.com/questions/1324179/how-to-tell-if-3-connected-points-are-connected-clockwise-or-counter-clockwise
		// https://en.wikipedia.org/wiki/Curve_orientation
		public static bool IsTriangleOrientedClockwise(WPos p1, WPos p2, WPos p3)
		{
			var determinant = GetDeterminant(p1, p2, p3);
			if (determinant > 0f)
				return false;
			return true;
		}

		// Is a quadrilateral convex? Assume no 3 points are colinear and the shape doesnt look like an hourglass
		public static bool IsQuadrilateralConvex(WPos a, WPos b, WPos c, WPos d)
		{
			var abc = IsTriangleOrientedClockwise(a, b, c);
			var abd = IsTriangleOrientedClockwise(a, b, d);
			var bcd = IsTriangleOrientedClockwise(b, c, d);
			var cad = IsTriangleOrientedClockwise(c, a, d);

			if (abc && abd && bcd && !cad)
				return true;
			else if (abc && abd && !bcd && cad)
				return true;
			else if (abc && !abd && bcd && cad)
				return true;
			// The opposite sign, which makes everything inverted
			else if (!abc && !abd && !bcd && cad)
				return true;
			else if (!abc && !abd && bcd && !cad)
				return true;
			else if (!abc && abd && !bcd && !cad)
				return true;

			return false;
		}

		// Is a point d inside, outside or on the same circle as a, b, c
		// https://gamedev.stackexchange.com/questions/71328/how-can-i-add-and-subtract-convex-polygons
		// Returns positive if inside, negative if outside, and 0 if on the circle
		public static float IsPointInsideOutsideOrOnCircle(WPos aVec, WPos bVec, WPos cVec, WPos dVec)
		{
			// This first part will simplify how we calculate the determinant
			var a = aVec.X - dVec.X;
			var d = bVec.X - dVec.X;
			var g = cVec.X - dVec.X;

			var b = aVec.Y - dVec.Y;
			var e = bVec.Y - dVec.Y;
			var h = cVec.Y - dVec.Y;

			var c = a * a + b * b;
			var f = d * d + e * e;
			var i = g * g + h * h;

			var determinant = a * e * i + b * f * g + c * d * h - g * e * c - h * f * a - i * d * b;
			return determinant;
		}

		// Sort the points along one axis. The first 3 points form a triangle. Consider the next point and connect it with all
		// previously connected points which are visible to the point. An edge is visible if the center of the edge is visible to the point.
		public static class IncrementalTriangulationAlgorithm
		{
			public static List<Triangle> TriangulatePoints(List<Vertex> points)
			{
				var triangles = new List<Triangle>();

				// Sort the points along x-axis
				// OrderBy is always soring in ascending order - use OrderByDescending to get in the other order
				points = points.OrderBy(n => n.position.X).ToList();

				// The first 3 vertices are always forming a triangle
				var newTriangle = new Triangle(points[0].position, points[1].position, points[2].position);

				triangles.Add(newTriangle);

				// All edges that form the triangles, so we have something to test against
				var edges = new List<Edge>
				{
					new(newTriangle.v1, newTriangle.v2),
					new(newTriangle.v2, newTriangle.v3),
					new(newTriangle.v3, newTriangle.v1)
				};

				// Add the other triangles one by one
				// Starts at 3 because we have already added 0,1,2
				for (var i = 3; i < points.Count; i++)
				{
					var currentPoint = points[i].position;

					// The edges we add this loop or we will get stuck in an endless loop
					var newEdges = new List<Edge>();

					// Is this edge visible? We only need to check if the midpoint of the edge is visible
					for (var j = 0; j < edges.Count; j++)
					{
						var currentEdge = edges[j];
						var midPoint = WPos.Zero + (WVec)(currentEdge.v1.position + (WVec)currentEdge.v2.position) / 2;
						var edgeToMidpoint = new Edge(currentPoint, midPoint);

						// Check if this line is intersecting
						var canSeeEdge = true;

						for (var k = 0; k < edges.Count; k++)
						{
							// Dont compare the edge with itself
							if (k == j)
							{
								continue;
							}

							if (AreEdgesIntersecting(edgeToMidpoint, edges[k]))
							{
								canSeeEdge = false;

								break;
							}
						}

						// This is a valid triangle
						if (canSeeEdge)
						{
							var edgeToPoint1 = new Edge(currentEdge.v1, new Vertex(currentPoint));
							var edgeToPoint2 = new Edge(currentEdge.v2, new Vertex(currentPoint));

							newEdges.Add(edgeToPoint1);
							newEdges.Add(edgeToPoint2);

							var newTri = new Triangle(edgeToPoint1.v1, edgeToPoint1.v2, edgeToPoint2.v1);

							triangles.Add(newTri);
						}
					}

					for (var j = 0; j < newEdges.Count; j++)
					{
						edges.Add(newEdges[j]);
					}
				}

				return triangles;
			}

			private static bool AreEdgesIntersecting(Edge edge1, Edge edge2)
			{
				var l1_p1 = new WPos(edge1.v1.position.X, edge1.v1.position.Z);
				var l1_p2 = new WPos(edge1.v2.position.X, edge1.v2.position.Z);

				var l2_p1 = new WPos(edge2.v1.position.X, edge2.v1.position.Z);
				var l2_p2 = new WPos(edge2.v2.position.X, edge2.v2.position.Z);

				var isIntersecting = AreLinesIntersecting(l1_p1, l1_p2, l2_p1, l2_p2, true);

				return isIntersecting;
			}

			public static bool AreLinesIntersecting(WPos l1_p1, WPos l1_p2, WPos l2_p1, WPos l2_p2, bool shouldIncludeEndPoints)
			{
				// To avoid floating point precision issues we can add a small value
				const float epsilon = 0.00001f;

				float denominator = (l2_p2.Y - l2_p1.Y) * (l1_p2.X - l1_p1.X) - (l2_p2.X - l2_p1.X) * (l1_p2.Y - l1_p1.Y);

				// Make sure the denominator is > 0, if not the lines are parallel
				if (denominator != 0f)
				{
					var u_a = ((l2_p2.X - l2_p1.X) * (l1_p1.Y - l2_p1.Y) - (l2_p2.Y - l2_p1.Y) * (l1_p1.X - l2_p1.X)) / denominator;
					var u_b = ((l1_p2.X - l1_p1.X) * (l1_p1.Y - l2_p1.Y) - (l1_p2.Y - l1_p1.Y) * (l1_p1.X - l2_p1.X)) / denominator;

					// Are the line segments intersecting if the end points are the same
					if (shouldIncludeEndPoints)
					{
						// Is intersecting if u_a and u_b are between 0 and 1 or exactly 0 or 1
						if (u_a >= 0f + epsilon && u_a <= 1f - epsilon && u_b >= 0f + epsilon && u_b <= 1f - epsilon)
							return true;
					}
					else
					{
						// Is intersecting if u_a and u_b are between 0 and 1
						if (u_a > 0f + epsilon && u_a < 1f - epsilon && u_b > 0f + epsilon && u_b < 1f - epsilon)
							return true;
					}
				}

				return false;
			}
		}

		// Alternative 1. Triangulate with some algorithm - then flip edges until we have a delaunay triangulation
		public static List<Triangle> TriangulateByFlippingEdges(List<WPos> sites)
		{
			// Step 1. Triangulate the points with some algorithm
			// WPos to vertex
			var vertices = new List<Vertex>();

			for (var i = 0; i < sites.Count; i++)
				vertices.Add(new Vertex(sites[i]));

			// Triangulate the convex hull of the sites
			var triangles = IncrementalTriangulationAlgorithm.TriangulatePoints(vertices);

			// Step 2. Change the structure from triangle to half-edge to make it faster to flip edges
			var halfEdges = TransformFromTriangleToHalfEdge(triangles);

			// Step 3. Flip edges until we have a delaunay triangulation
			var safety = 0;

			var flippedEdges = 0;

			while (true)
			{
				safety++;

				if (safety > 100000)
				{
					Console.WriteLine("Stuck in endless loop");
					break;
				}

				var hasFlippedEdge = false;

				// Search through all edges to see if we can flip an edge
				for (var i = 0; i < halfEdges.Count; i++)
				{
					var thisEdge = halfEdges[i];

					// Is this edge sharing an edge, otherwise its a border, and then we cant flip the edge
					if (thisEdge.oppositeEdge == null)
						continue;

					//The vertices belonging to the two triangles, c-a are the edge vertices, b belongs to this triangle
					var a = thisEdge.v;
					var b = thisEdge.nextEdge.v;
					var c = thisEdge.prevEdge.v;
					var d = thisEdge.oppositeEdge.nextEdge.v;

					var aPos = a.GetPos2D_XZ();
					var bPos = b.GetPos2D_XZ();
					var cPos = c.GetPos2D_XZ();
					var dPos = d.GetPos2D_XZ();

					// Use the circle test to test if we need to flip this edge
					if (IsPointInsideOutsideOrOnCircle(aPos, bPos, cPos, dPos) < 0f)
					{
						// Are these the two triangles that share this edge forming a convex quadrilateral?
						// Otherwise the edge cant be flipped
						if (IsQuadrilateralConvex(aPos, bPos, cPos, dPos))
						{
							// If the new triangle after a flip is not better, then dont flip
							// This will also stop the algoritm from ending up in an endless loop
							if (IsPointInsideOutsideOrOnCircle(bPos, cPos, dPos, aPos) < 0f)
								continue;

							// Flip the edge
							flippedEdges++;
							hasFlippedEdge = true;
							FlipEdge(thisEdge);
						}
					}
				}

				// We have searched through all edges and havent found an edge to flip, so we have a Delaunay triangulation!
				if (!hasFlippedEdge)
					break;
			}

			// Dont have to convert from half edge to triangle because the algorithm will modify the objects, which belongs to the
			// original triangles, so the triangles have the data we need
			return triangles;
		}

		// Flip an edge
		public static void FlipEdge(HalfEdge one)
		{
			// The data we need

			// This edge's triangle
			var two = one.nextEdge;
			var three = one.prevEdge;

			// The opposite edge's triangle
			var four = one.oppositeEdge;
			var five = one.oppositeEdge.nextEdge;
			var six = one.oppositeEdge.prevEdge;

			// The vertices
			var a = one.v;
			var b = one.nextEdge.v;
			var c = one.prevEdge.v;
			var d = one.oppositeEdge.nextEdge.v;

			// Flip

			// Change vertex
			a.halfEdge = one.nextEdge;
			c.halfEdge = one.oppositeEdge.nextEdge;

			// Change half-edge
			// Half-edge - half-edge connections
			one.nextEdge = three;
			one.prevEdge = five;

			two.nextEdge = four;
			two.prevEdge = six;

			three.nextEdge = five;
			three.prevEdge = one;

			four.nextEdge = six;
			four.prevEdge = two;

			five.nextEdge = one;
			five.prevEdge = three;

			six.nextEdge = two;
			six.prevEdge = four;

			// Half-edge - vertex connection
			one.v = b;
			two.v = b;
			three.v = c;
			four.v = d;
			five.v = d;
			six.v = a;

			// Half-edge - triangle connection
			var t1 = one.t;
			var t2 = four.t;

			one.t = t1;
			three.t = t1;
			five.t = t1;

			two.t = t2;
			four.t = t2;
			six.t = t2;

			// Opposite-edges are not changing!

			// Triangle connection
			t1.v1 = b.ToWPos();
			t1.v2 = c.ToWPos();
			t1.v3 = d.ToWPos();

			t2.v1 = b.ToWPos();
			t2.v2 = d.ToWPos();
			t2.v3 = a.ToWPos();

			t1.halfEdge = three;
			t2.halfEdge = four;
		}
	}

	public static class ConstrainedDelaunay
	{
		// From the report "An algorithm for generating constrained delaunay triangulations" by Sloan
		public static List<Triangle> GenerateTriangulation(List<WPos> points, List<WPos> constraints)
		{
			// Start by generating a delaunay triangulation with all points, including the constraints
			points.AddRange(constraints);

			// This delaunay triangulation algorithm is not the same as in the report, but it makes no difference
			var delaunayTriangulation = NavMesh.TriangulateByFlippingEdges(points);

			// Modify the triangulation by adding the constraints to the delaunay triangulation
			var constrainedDelaunayTriangulation = AddConstraints(delaunayTriangulation, constraints);

			return constrainedDelaunayTriangulation;
		}

		// Clamp list indices
		// Will even work if index is larger/smaller than listSize, so can loop multiple times
		public static int ClampListIndex(int index, int listSize)
		{
			index = (index % listSize + listSize) % listSize;
			return index;
		}

		// Add the constraints to the delaunay triangulation
		private static List<Triangle> AddConstraints(List<Triangle> triangulation, List<WPos> constraints)
		{
			// The steps numbering is from the report
			// Step 1. Loop over each constrained edge. For each of these edges, do steps 2-4
			for (var i = 0; i < constraints.Count; i++)
			{
				// Let each constrained edge be defined by the vertices:
				var v_i = constraints[i];
				var v_j = constraints[ClampListIndex(i + 1, constraints.Count)];

				// Check if this constraint already exists in the triangulation, if so we are happy and dont need to worry about this edge
				if (IsEdgePartOfTriangulation(triangulation, v_i, v_j))
					continue;

				// Step 2. Find all edges in the current triangulation that intersects with this constraint
				var intersectingEdges = FindIntersectingEdges(triangulation, v_i, v_j);

				// Step 3. Remove intersecting edges by adding new edges
				var newEdges = RemoveIntersectingEdges(v_i, v_j, intersectingEdges);

				// Step 4. Restore delaunay triangulation (if you want to)
				RestoreDelaunayTriangulation(v_i, v_j, newEdges);
			}

			// Step 5. Remove superfluous triangles (if you need to)
			RemoveSuperfluousTriangles(triangulation, constraints);

			return triangulation;
		}

		// Remove edges that intersects with a constraint and add new edges
		// The idea here is that all possible triangulations for a set of points can be found
		// by systematically swapping the diagonal in each convex quadrilateral formed by a pair of triangles
		// So we will test all possible arrangements and will always find a triangulation which includes the constrained edge
		private static List<HalfEdge> RemoveIntersectingEdges(WPos v_i, WPos v_j, List<HalfEdge> intersectingEdges)
		{
			var newEdges = new List<HalfEdge>();

			var safety = 0;

			// While some edges still cross the constrained edge, do steps 3.1 and 3.2
			while (intersectingEdges.Count > 0)
			{
				safety++;

				if (safety > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when fixing constrained edges");
					break;
				}

				// Step 3.1. Remove an edge from the list of edges that intersects the constrained edge
				var e = intersectingEdges[0];

				intersectingEdges.RemoveAt(0);

				// The vertices belonging to the two triangles
				var v_k = e.v.position;
				var v_l = e.prevEdge.v.position;
				var v_third_pos = e.nextEdge.v.position;
				// The vertex belonging to the opposite triangle and isn't shared by the current edge
				var v_opposite_pos = e.oppositeEdge.nextEdge.v.position;

				// Step 3.2. If the two triangles that share the edge v_k and v_l do not form a convex quadtrilateral then place
				// the edge back on the list of intersecting edges and go to step 3.1
				if (!NavMesh.IsQuadrilateralConvex(v_k.XZ(), v_l.XZ(), v_third_pos.XZ(), v_opposite_pos.XZ()))
					intersectingEdges.Add(e);
				else
				{
					// Flip the edge like we did when we created the delaunay triangulation so use the code from that class
					NavMesh.FlipEdge(e);

					// The new diagonal is defined by the vertices
					var v_m = e.v.position;
					var v_n = e.prevEdge.v.position;

					// If this new diagonal intersects the constrained edge, add it to the list of intersecting edges
					if (IsEdgeCrossingEdge(v_i, v_j, v_m, v_n))
						intersectingEdges.Add(e);
					else // Place it in the list of newly created edges
						newEdges.Add(e);
				}
			}

			return newEdges;
		}

		// Try to restore the delaunay triangulation by flipping newly created edges
		// This process is similar to when we created the original delaunay triangulation
		// This step can maybe be skipped if you just want a triangulation and Ive noticed its often not flipping any triangles
		private static void RestoreDelaunayTriangulation(WPos v_i, WPos v_j, List<HalfEdge> newEdges)
		{
			var safety = 0;
			var flippedEdges = 0;

			// Repeat 4.1 - 4.3 until no further swaps take place
			while (true)
			{
				safety++;

				if (safety > 100000)
				{
					Console.WriteLine("Stuck in endless loop when delaunay after fixing constrained edges");
					break;
				}

				var hasFlippedEdge = false;

				// Step 4.1. Loop over each edge in the list of newly created edges
				for (var j = 0; j < newEdges.Count; j++)
				{
					var e = newEdges[j];

					// Step 4.2. Let the newly created edge be defined by the vertices
					var v_k = e.v.position;
					var v_l = e.prevEdge.v.position;

					// If this edge is equal to the constrained edge v_i and v_j, then skip to step 4.1
					// because we are not allowed to flip a constrained edge
					if ((v_k == v_i && v_l == v_j) || (v_l == v_i && v_k == v_j))
						continue;

					// Step 4.3. If the two triangles that share edge v_k and v_l don't satisfy the delaunay criterion,
					// so that a vertex of one of the triangles is inside the circumcircle of the other triangle, flip the edge
					// The third vertex of the triangle belonging to this edge
					var v_third_pos = e.nextEdge.v.position;

					// The vertice belonging to the triangle on the opposite side of the edge and this vertex is not a part of the edge
					var v_opposite_pos = e.oppositeEdge.nextEdge.v.position;

					var circleTestValue = NavMesh.IsPointInsideOutsideOrOnCircle(v_k.XZ(), v_l.XZ(), v_third_pos.XZ(), v_opposite_pos.XZ());

					if (circleTestValue < 0f)
					{
						// Are these the two triangles that share this edge forming a convex quadrilateral? Otherwise the edge cant be flipped
						if (NavMesh.IsQuadrilateralConvex(v_k.XZ(), v_l.XZ(), v_third_pos.XZ(), v_opposite_pos.XZ()))
						{
							// If the new triangle after a flip is not better, then dont flip
							if (NavMesh.IsPointInsideOutsideOrOnCircle(v_opposite_pos.XZ(), v_l.XZ(), v_third_pos.XZ(), v_k.XZ()) <= circleTestValue)
								continue;

							// Flip the edge
							hasFlippedEdge = true;
							NavMesh.FlipEdge(e);
							flippedEdges++;
						}
					}
				}

				// We have searched through all edges and havent found an edge to flip, so we cant improve anymore
				if (!hasFlippedEdge)
				{
					Console.WriteLine("Found a constrained delaunay triangulation in " + flippedEdges + " flips");
					break;
				}
			}
		}

		// Remove all triangles that are inside the constraint
		// This assumes the vertices in the constraint are ordered clockwise
		private static void RemoveSuperfluousTriangles(List<Triangle> triangulation, List<WPos> constraints)
		{
			// This assumes we have at least 3 vertices in the constraint because we cant delete triangles inside a line
			if (constraints.Count < 3)
				return;

			// Start at a triangle with an edge that shares an edge with the first constraint edge in the list
			// Since both are clockwise we know we are "inside" of the constraint, so this is a triangle we should delete
			Triangle borderTriangle = null;

			var constrained_p1 = constraints[0];
			var constrained_p2 = constraints[1];

			for (var i = 0; i < triangulation.Count; i++)
			{
				var e1 = triangulation[i].halfEdge;
				var e2 = e1.nextEdge;
				var e3 = e2.nextEdge;

				// Is any of these edges a constraint?
				if (e1.v.position == constrained_p2 && e1.prevEdge.v.position == constrained_p1)
				{
					borderTriangle = triangulation[i];
					break;
				}

				if (e2.v.position == constrained_p2 && e2.prevEdge.v.position == constrained_p1)
				{
					borderTriangle = triangulation[i];
					break;
				}

				if (e3.v.position == constrained_p2 && e3.prevEdge.v.position == constrained_p1)
				{
					borderTriangle = triangulation[i];
					break;
				}
			}

			if (borderTriangle == null)
				return;

			// Find all triangles within the constraint by using a flood fill algorithm
			// Add these triangles should be deleted
			var trianglesToBeDeleted = new List<Triangle>();
			var neighborsToCheck = new List<Triangle>
			{
				// Start at the triangle we know is within the constraints
				borderTriangle
			};

			var safety = 0;

			while (true)
			{
				safety++;

				if (safety > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when deleteing superfluous triangles");
					break;
				}

				// Stop if we are out of neighbors
				if (neighborsToCheck.Count == 0)
					break;

				// Pick the first triangle in the list and investigate its neighbors
				var t = neighborsToCheck[0];

				neighborsToCheck.RemoveAt(0);

				trianglesToBeDeleted.Add(t);

				var e1 = t.halfEdge;
				var e2 = e1.nextEdge;
				var e3 = e2.nextEdge;

				// If the neighbor is not an outer border meaning no neighbor exists
				// If we have not already visited the neighbor
				// If the edge between the neighbor and this triangle is not a constraint
				// Then its a valid neighbor and we should flood to it
				if (
					e1.oppositeEdge != null &&
					!trianglesToBeDeleted.Contains(e1.oppositeEdge.t) &&
					!neighborsToCheck.Contains(e1.oppositeEdge.t) &&
					!IsAnEdgeAConstraint(e1.v.position, e1.prevEdge.v.position, constraints))
				{
					neighborsToCheck.Add(e1.oppositeEdge.t);
				}

				if (
					e2.oppositeEdge != null &&
					!trianglesToBeDeleted.Contains(e2.oppositeEdge.t) &&
					!neighborsToCheck.Contains(e2.oppositeEdge.t) &&
					!IsAnEdgeAConstraint(e2.v.position, e2.prevEdge.v.position, constraints))
				{
					neighborsToCheck.Add(e2.oppositeEdge.t);
				}

				if (
					e3.oppositeEdge != null &&
					!trianglesToBeDeleted.Contains(e3.oppositeEdge.t) &&
					!neighborsToCheck.Contains(e3.oppositeEdge.t) &&
					!IsAnEdgeAConstraint(e3.v.position, e3.prevEdge.v.position, constraints))
				{
					neighborsToCheck.Add(e3.oppositeEdge.t);
				}
			}

			// Delete the triangles
			for (var i = 0; i < trianglesToBeDeleted.Count; i++)
			{
				var t = trianglesToBeDeleted[i];

				// Remove from the list of all triangles
				triangulation.Remove(t);

				// In the half-edge data structure there's an edge going in the opposite direction
				// on the other side of this triangle with a reference to this edge, so we have to remove these
				var t_e1 = t.halfEdge;
				var t_e2 = t_e1.nextEdge;
				var t_e3 = t_e2.nextEdge;

				if (t_e1.oppositeEdge != null)
					t_e1.oppositeEdge.oppositeEdge = null;

				if (t_e2.oppositeEdge != null)
					t_e2.oppositeEdge.oppositeEdge = null;

				if (t_e3.oppositeEdge != null)
					t_e3.oppositeEdge.oppositeEdge = null;
			}
		}

		// Is an edge between p1 and p2 a constraint?
		private static bool IsAnEdgeAConstraint(WPos p1, WPos p2, List<WPos> constraints)
		{
			for (var i = 0; i < constraints.Count; i++)
			{
				var c_p1 = constraints[i];
				var c_p2 = constraints[ClampListIndex(i + 1, constraints.Count)];

				if ((p1 == c_p1 && p2 == c_p2) || (p2 == c_p1 && p1 == c_p2))
					return true;
			}

			return false;
		}

		// Find all edges of the current triangulation that intersects with the constraint edge between p1 and p2
		private static List<HalfEdge> FindIntersectingEdges(List<Triangle> triangulation, WPos p1, WPos p2)
		{
			var intersectingEdges = new List<HalfEdge>();

			// Step 1. Begin at a triangle connected to the first vertex in the constraint edge
			Triangle t = null;

			for (var i = 0; i < triangulation.Count; i++)
			{
				// The edges the triangle consists of
				var e1 = triangulation[i].halfEdge;
				var e2 = e1.nextEdge;
				var e3 = e2.nextEdge;

				// Does one of these edges include the first vertex in the constraint edge
				if (e1.v.position == p1 || e2.v.position == p1 || e3.v.position == p1)
				{
					t = triangulation[i];

					break;
				}
			}


			// Step2. Walk around p1 until we find a triangle with an edge that intersects with the edge p1-p2
			var safety = 0;

			// This is the last edge on the previous triangle we crossed so we know which way to rotatet
			HalfEdge lastEdge = null;

			// When we rotate we might pick the wrong start direction if the edge is on the border, so we can't rotate all the way around
			// If that happens we have to restart and rotate in the other direction
			var startTriangle = t;

			var restart = false;

			while (true)
			{
				safety++;

				if (safety > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when finding the start triangle when finding intersecting edges");

					break;
				}

				// Check if the current triangle is intersecting with the constraint
				var e1 = t.halfEdge;
				var e2 = e1.nextEdge;
				var e3 = e2.nextEdge;

				// The only edge that can intersect with the constraint is the edge that doesnt include p1, so find it
				HalfEdge e_doesnt_include_p1;
				if (e1.v.position != p1 && e1.prevEdge.v.position != p1)
					e_doesnt_include_p1 = e1;
				else if (e2.v.position != p1 && e2.prevEdge.v.position != p1)
					e_doesnt_include_p1 = e2;
				else
					e_doesnt_include_p1 = e3;

				// Is the edge that doesn't include p1 intersecting with the constrained edge?
				if (IsEdgeCrossingEdge(e_doesnt_include_p1.v.position, e_doesnt_include_p1.prevEdge.v.position, p1, p2))
					break; // We have found the triangle where we should begin the walk

				// We have not found the triangle where we should begin the walk, so we should rotate to another triangle which includes p1

				// Find the two edges that include p1 so we can rotate across one of them
				var includes_p1 = new List<HalfEdge>();

				if (e1 != e_doesnt_include_p1)
					includes_p1.Add(e1);

				if (e2 != e_doesnt_include_p1)
					includes_p1.Add(e2);

				if (e3 != e_doesnt_include_p1)
					includes_p1.Add(e3);

				// This is the first rotation we do from the triangle we found at the start, so we rotate in a direction
				if (lastEdge == null)
				{
					// But if we are on the border of the triangulation we cant just pick a direction because one of the
					// directions might not be valid and end up outside of the triangulation
					// This problem could be solved if we add a "supertriangle" covering all points
					lastEdge = includes_p1[0];

					// Dont go in this direction because then we are outside of the triangulation
					// Sometimes we may have picked the wrong direction when we rotate from the first triangle
					// and rotated around towards a triangle that's at the border, if so we have to restart and rotate
					// in the other direction
					if (lastEdge.oppositeEdge == null || restart)
						lastEdge = includes_p1[1];

					// The triangle we rotate to
					t = lastEdge.oppositeEdge.t;
				}
				else
				{
					// Move in the direction that doesnt include the last edge
					if (includes_p1[0].oppositeEdge != lastEdge)
						lastEdge = includes_p1[0];
					else
						lastEdge = includes_p1[1];

					// If we have hit a border edge, we should have rotated in the other direction when we started at the first triangle
					// So we have to jump back
					if (lastEdge.oppositeEdge == null)
					{
						restart = true;
						t = startTriangle;
						lastEdge = null;
					}
					else
						t = lastEdge.oppositeEdge.t; // The triangle we rotate to
				}
			}


			// Step3. March from one triangle to the next in the general direction of p2
			// This means we always move across the edge of the triangle that intersects with the constraint
			var safety2 = 0;

			lastEdge = null;

			while (true)
			{
				safety2++;

				if (safety2 > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when finding intersecting edges");
					break;
				}

				// The three edges belonging to the current triangle
				var e1 = t.halfEdge;
				var e2 = e1.nextEdge;
				var e3 = e2.nextEdge;

				// Does this triangle include the last vertex on the constraint edge? If so we have found all edges that intersects
				if (e1.v.position == p2 || e2.v.position == p2 || e3.v.position == p2)
					break;

				// Find which edge that intersects with the constraint
				// More than one edge maight intersect, so we have to check if it's not the edge we are coming from
				else
				{
					// Save the edge that intersects in case the triangle intersects with two edges
					if (e1.oppositeEdge != lastEdge && IsEdgeCrossingEdge(e1.v.position, e1.prevEdge.v.position, p1, p2))
						lastEdge = e1;
					else if (e2.oppositeEdge != lastEdge && IsEdgeCrossingEdge(e2.v.position, e2.prevEdge.v.position, p1, p2))
						lastEdge = e2;
					else
						lastEdge = e3;

					// Jump to the next triangle by crossing the edge that intersects with the constraint
					t = lastEdge.oppositeEdge.t;

					// Save the intersecting edge
					intersectingEdges.Add(lastEdge);
				}
			}

			return intersectingEdges;
		}

		// Check if an edge is intersecting with the constraint edge between p1 and p2
		// If so, add it to the list if the edge doesnt exist in the list
		private static void TryAddEdgeToIntersectingEdges(HalfEdge e, WPos p1, WPos p2, List<HalfEdge> intersectingEdges)
		{
			// The position the edge is going to
			var e_p1 = e.v.position;

			// The position the edge is coming from
			var e_p2 = e.prevEdge.v.position;

			// Is this edge intersecting with the constraint?
			if (IsEdgeCrossingEdge(e_p1, e_p2, p1, p2))
			{
				// Add it to the list if it isnt already in the list
				for (var i = 0; i < intersectingEdges.Count; i++)
				{
					// In the half-edge data structure, theres another edge on the opposite side going in the other direction
					// so we have to check both because we want unique edges
					if (intersectingEdges[i] == e || intersectingEdges[i].oppositeEdge == e)
					{
						// The edge is already in the list
						return;
					}
				}

				// The edge is not in the list so add it
				intersectingEdges.Add(e);
			}
		}

		// Is an edge crossing another edge?
		private static bool IsEdgeCrossingEdge(WPos e1_p1, WPos e1_p2, WPos e2_p1, WPos e2_p2)
		{
			// We will here run into floating point precision issues so we have to be careful
			// To solve that you can first check the end points
			// and modify the line-line intersection algorithm to include a small epsilon

			// First check if the edges are sharing a point, if so they are not crossing
			if (e1_p1 == e2_p1 || e1_p1 == e2_p2 || e1_p2 == e2_p1 || e1_p2 == e2_p2)
				return false;

			// Then check if the lines are intersecting
			if (!NavMesh.IncrementalTriangulationAlgorithm.AreLinesIntersecting(e1_p1, e1_p2, e2_p1, e2_p2, false))
				return false;

			return true;
		}

		// Is an edge (between p1 and p2) a part of an edge in the triangulation?
		private static bool IsEdgePartOfTriangulation(List<Triangle> triangulation, WPos p1, WPos p2)
		{
			for (var i = 0; i < triangulation.Count; i++)
			{
				// The vertices positions of the current triangle
				var t_p1 = triangulation[i].v1;
				var t_p2 = triangulation[i].v2;
				var t_p3 = triangulation[i].v3;

				// Check if any of the triangle's edges have the same coordinates as the constrained edge
				// We have no idea about direction so we have to check both directions
				if ((t_p1 == p1 && t_p2 == p2) || (t_p1 == p2 && t_p2 == p1))
					return true;

				if ((t_p2 == p1 && t_p3 == p2) || (t_p2 == p2 && t_p3 == p1))
					return true;

				if ((t_p3 == p1 && t_p1 == p2) || (t_p3 == p2 && t_p1 == p1))
					return true;
			}

			return false;
		}
	}
}

using Mono.Nat;
using OpenRA.Mods.Common.Traits;
using OpenRA.Traits;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;

namespace OpenRA.Mods.Common
{
	public class ConstrainedDelauneyTriangulation
	{
		//From the report "An algorithm for generating constrained delaunay triangulations" by Sloan
		public static List<Triangle> GenerateTriangulation(List<WPos> points, List<WPos> constraints)
		{
			//Start by generating a delaunay triangulation with all points, including the constraints
			points.AddRange(constraints);

			//This delaunay triangulation algorithm is not the same as in the report, but it makes no difference
			List<Triangle> delaunayTriangulation = NavMesh.TriangulateByFlippingEdges(points);

			//Modify the triangulation by adding the constraints to the delaunay triangulation
			List<Triangle> constrainedDelaunayTriangulation = AddConstraints(delaunayTriangulation, constraints);

			return constrainedDelaunayTriangulation;
		}

		// Clamp list indices
		// Will even work if index is larger/smaller than listSize, so can loop multiple times
		public static int ClampListIndex(int index, int listSize)
		{
			index = (index % listSize + listSize) % listSize;
			return index;
		}

		//Add the constraints to the delaunay triangulation
		private static List<Triangle> AddConstraints(List<Triangle> triangulation, List<WPos> constraints)
		{
			//The steps numbering is from the report
			//Step 1. Loop over each constrained edge. For each of these edges, do steps 2-4 
			for (int i = 0; i < constraints.Count; i++)
			{
				//Let each constrained edge be defined by the vertices:
				var v_i = constraints[i];
				var v_j = constraints[ClampListIndex(i + 1, constraints.Count)];

				//Check if this constraint already exists in the triangulation, if so we are happy and dont need to worry about this edge
				if (IsEdgePartOfTriangulation(triangulation, v_i, v_j))
				{
					continue;
				}

				//Step 2. Find all edges in the current triangulation that intersects with this constraint
				List<HalfEdge> intersectingEdges = FindIntersectingEdges(triangulation, v_i, v_j);

				//Step 3. Remove intersecting edges by adding new edges
				List<HalfEdge> newEdges = RemoveIntersectingEdges(v_i, v_j, intersectingEdges);

				//Step 4. Restore delaunay triangulation (if you want to)
				RestoreDelaunayTriangulation(v_i, v_j, newEdges);
			}

			//Step 5. Remove superfluous triangles (if you need to)
			RemoveSuperfluousTriangles(triangulation, constraints);

			return triangulation;
		}

		//Remove edges that intersects with a constraint and add new edges
		//The idea here is that all possible triangulations for a set of points can be found 
		//by systematically swapping the diagonal in each convex quadrilateral formed by a pair of triangles
		//So we will test all possible arrangements and will always find a triangulation which includes the constrained edge
		private static List<HalfEdge> RemoveIntersectingEdges(WPos v_i, WPos v_j, List<HalfEdge> intersectingEdges)
		{
			List<HalfEdge> newEdges = new List<HalfEdge>();

			int safety = 0;

			//While some edges still cross the constrained edge, do steps 3.1 and 3.2
			while (intersectingEdges.Count > 0)
			{
				safety += 1;

				if (safety > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when fixing constrained edges");

					break;
				}

				//Step 3.1. Remove an edge from the list of edges that intersects the constrained edge
				HalfEdge e = intersectingEdges[0];

				intersectingEdges.RemoveAt(0);

				//The vertices belonging to the two triangles
				var v_k = e.v.position;
				var v_l = e.prevEdge.v.position;
				var v_third_pos = e.nextEdge.v.position;
				//The vertex belonging to the opposite triangle and isn't shared by the current edge
				var v_opposite_pos = e.oppositeEdge.nextEdge.v.position;

				//Step 3.2. If the two triangles that share the edge v_k and v_l do not form a convex quadtrilateral then place
				//the edge back on the list of intersecting edges and go to step 3.1
				if (!NavMesh.IsQuadrilateralConvex(v_k.XZ(), v_l.XZ(), v_third_pos.XZ(), v_opposite_pos.XZ()))
				{
					intersectingEdges.Add(e);

					continue;
				}
				else
				{
					//Flip the edge like we did when we created the delaunay triangulation so use the code from that class
					NavMesh.FlipEdge(e);

					//The new diagonal is defined by the vertices
					var v_m = e.v.position;
					var v_n = e.prevEdge.v.position;

					//If this new diagonal intersects the constrained edge, add it to the list of intersecting edges
					if (IsEdgeCrossingEdge(v_i, v_j, v_m, v_n))
					{
						intersectingEdges.Add(e);
					}
					//Place it in the list of newly created edges
					else
					{
						newEdges.Add(e);
					}
				}
			}

			return newEdges;
		}

		//Try to restore the delaunay triangulation by flipping newly created edges
		//This process is similar to when we created the original delaunay triangulation
		//This step can maybe be skipped if you just want a triangulation and Ive noticed its often not flipping any triangles
		private static void RestoreDelaunayTriangulation(WPos v_i, WPos v_j, List<HalfEdge> newEdges)
		{
			int safety = 0;

			int flippedEdges = 0;

			//Repeat 4.1 - 4.3 until no further swaps take place
			while (true)
			{
				safety += 1;

				if (safety > 100000)
				{
					Console.WriteLine("Stuck in endless loop when delaunay after fixing constrained edges");

					break;
				}

				bool hasFlippedEdge = false;

				//Step 4.1. Loop over each edge in the list of newly created edges
				for (int j = 0; j < newEdges.Count; j++)
				{
					HalfEdge e = newEdges[j];

					//Step 4.2. Let the newly created edge be defined by the vertices
					var v_k = e.v.position;
					var v_l = e.prevEdge.v.position;

					//If this edge is equal to the constrained edge v_i and v_j, then skip to step 4.1
					//because we are not allowed to flip a constrained edge
					if ((v_k == v_i && v_l == v_j) || (v_l == v_i && v_k == v_j))
					{
						continue;
					}

					//Step 4.3. If the two triangles that share edge v_k and v_l don't satisfy the delaunay criterion,
					//so that a vertex of one of the triangles is inside the circumcircle of the other triangle, flip the edge
					//The third vertex of the triangle belonging to this edge
					var v_third_pos = e.nextEdge.v.position;
					//The vertice belonging to the triangle on the opposite side of the edge and this vertex is not a part of the edge
					var v_opposite_pos = e.oppositeEdge.nextEdge.v.position;

					float circleTestValue = NavMesh.IsPointInsideOutsideOrOnCircle(v_k.XZ(), v_l.XZ(), v_third_pos.XZ(), v_opposite_pos.XZ());

					if (circleTestValue < 0f)
					{
						//Are these the two triangles that share this edge forming a convex quadrilateral? Otherwise the edge cant be flipped
						if (NavMesh.IsQuadrilateralConvex(v_k.XZ(), v_l.XZ(), v_third_pos.XZ(), v_opposite_pos.XZ()))
						{
							//If the new triangle after a flip is not better, then dont flip
							if (NavMesh.IsPointInsideOutsideOrOnCircle(v_opposite_pos.XZ(), v_l.XZ(), v_third_pos.XZ(), v_k.XZ()) <= circleTestValue)
							{
								continue;
							}

							//Flip the edge
							hasFlippedEdge = true;

							NavMesh.FlipEdge(e);

							flippedEdges += 1;
						}
					}
				}

				//We have searched through all edges and havent found an edge to flip, so we cant improve anymore
				if (!hasFlippedEdge)
				{
					Console.WriteLine("Found a constrained delaunay triangulation in " + flippedEdges + " flips");

					break;
				}
			}
		}

		//Remove all triangles that are inside the constraint
		//This assumes the vertices in the constraint are ordered clockwise
		private static void RemoveSuperfluousTriangles(List<Triangle> triangulation, List<WPos> constraints)
		{
			//This assumes we have at least 3 vertices in the constraint because we cant delete triangles inside a line
			if (constraints.Count < 3)
			{
				return;
			}

			//Start at a triangle with an edge that shares an edge with the first constraint edge in the list 
			//Since both are clockwise we know we are "inside" of the constraint, so this is a triangle we should delete
			Triangle borderTriangle = null;

			var constrained_p1 = constraints[0];
			var constrained_p2 = constraints[1];

			for (int i = 0; i < triangulation.Count; i++)
			{
				HalfEdge e1 = triangulation[i].halfEdge;
				HalfEdge e2 = e1.nextEdge;
				HalfEdge e3 = e2.nextEdge;

				//Is any of these edges a constraint?
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
			{
				return;
			}


			//Find all triangles within the constraint by using a flood fill algorithm
			//Add these triangles should be deleted
			List<Triangle> trianglesToBeDeleted = new List<Triangle>();

			List<Triangle> neighborsToCheck = new List<Triangle>();

			//Start at the triangle we know is within the constraints
			neighborsToCheck.Add(borderTriangle);

			int safety = 0;

			while (true)
			{
				safety += 1;

				if (safety > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when deleteing superfluous triangles");

					break;
				}

				//Stop if we are out of neighbors
				if (neighborsToCheck.Count == 0)
				{
					break;
				}

				//Pick the first triangle in the list and investigate its neighbors
				Triangle t = neighborsToCheck[0];

				neighborsToCheck.RemoveAt(0);

				trianglesToBeDeleted.Add(t);

				HalfEdge e1 = t.halfEdge;
				HalfEdge e2 = e1.nextEdge;
				HalfEdge e3 = e2.nextEdge;

				//If the neighbor is not an outer border meaning no neighbor exists
				//If we have not already visited the neighbor
				//If the edge between the neighbor and this triangle is not a constraint
				//Then its a valid neighbor and we should flood to it
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


			//Delete the triangles
			for (int i = 0; i < trianglesToBeDeleted.Count; i++)
			{
				Triangle t = trianglesToBeDeleted[i];

				//Remove from the list of all triangles
				triangulation.Remove(t);

				//In the half-edge data structure there's an edge going in the opposite direction
				//on the other side of this triangle with a reference to this edge, so we have to remove these
				HalfEdge t_e1 = t.halfEdge;
				HalfEdge t_e2 = t_e1.nextEdge;
				HalfEdge t_e3 = t_e2.nextEdge;

				if (t_e1.oppositeEdge != null)
				{
					t_e1.oppositeEdge.oppositeEdge = null;
				}
				if (t_e2.oppositeEdge != null)
				{
					t_e2.oppositeEdge.oppositeEdge = null;
				}
				if (t_e3.oppositeEdge != null)
				{
					t_e3.oppositeEdge.oppositeEdge = null;
				}
			}
		}

		//Is an edge between p1 and p2 a constraint?
		private static bool IsAnEdgeAConstraint(WPos p1, WPos p2, List<WPos> constraints)
		{
			for (int i = 0; i < constraints.Count; i++)
			{
				var c_p1 = constraints[i];
				var c_p2 = constraints[ClampListIndex(i + 1, constraints.Count)];

				if ((p1 == c_p1 && p2 == c_p2) || (p2 == c_p1 && p1 == c_p2))
				{
					return true;
				}
			}

			return false;
		}

		//Find all edges of the current triangulation that intersects with the constraint edge between p1 and p2
		private static List<HalfEdge> FindIntersectingEdges(List<Triangle> triangulation, WPos p1, WPos p2)
		{
			List<HalfEdge> intersectingEdges = new List<HalfEdge>();

			//While the above is working, a faster (but more complicated) way is to do a triangle walk which is suggested in the report

			//Step 1. Begin at a triangle connected to the first vertex in the constraint edge
			Triangle t = null;

			for (int i = 0; i < triangulation.Count; i++)
			{
				//The edges the triangle consists of
				HalfEdge e1 = triangulation[i].halfEdge;
				HalfEdge e2 = e1.nextEdge;
				HalfEdge e3 = e2.nextEdge;

				//Does one of these edges include the first vertex in the constraint edge
				if (e1.v.position == p1 || e2.v.position == p1 || e3.v.position == p1)
				{
					t = triangulation[i];

					break;
				}
			}


			//Step2. Walk around p1 until we find a triangle with an edge that intersects with the edge p1-p2
			int safety = 0;

			//This is the last edge on the previous triangle we crossed so we know which way to rotatet
			HalfEdge lastEdge = null;

			//When we rotate we might pick the wrong start direction if the edge is on the border, so we can't rotate all the way around
			//If that happens we have to restart and rotate in the other direction
			Triangle startTriangle = t;

			bool restart = false;

			while (true)
			{
				safety += 1;

				if (safety > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when finding the start triangle when finding intersecting edges");

					break;
				}

				//Check if the current triangle is intersecting with the constraint
				HalfEdge e1 = t.halfEdge;
				HalfEdge e2 = e1.nextEdge;
				HalfEdge e3 = e2.nextEdge;

				//The only edge that can intersect with the constraint is the edge that doesnt include p1, so find it
				HalfEdge e_doesnt_include_p1 = null;

				if (e1.v.position != p1 && e1.prevEdge.v.position != p1)
				{
					e_doesnt_include_p1 = e1;
				}
				else if (e2.v.position != p1 && e2.prevEdge.v.position != p1)
				{
					e_doesnt_include_p1 = e2;
				}
				else
				{
					e_doesnt_include_p1 = e3;
				}

				//Is the edge that doesn't include p1 intersecting with the constrained edge?
				if (IsEdgeCrossingEdge(e_doesnt_include_p1.v.position, e_doesnt_include_p1.prevEdge.v.position, p1, p2))
				{
					//We have found the triangle where we should begin the walk
					break;
				}

				//We have not found the triangle where we should begin the walk, so we should rotate to another triangle which includes p1

				//Find the two edges that include p1 so we can rotate across one of them
				List<HalfEdge> includes_p1 = new List<HalfEdge>();

				if (e1 != e_doesnt_include_p1)
				{
					includes_p1.Add(e1);
				}
				if (e2 != e_doesnt_include_p1)
				{
					includes_p1.Add(e2);
				}
				if (e3 != e_doesnt_include_p1)
				{
					includes_p1.Add(e3);
				}

				//This is the first rotation we do from the triangle we found at the start, so we rotate in a direction
				if (lastEdge == null)
				{
					//But if we are on the border of the triangulation we cant just pick a direction because one of the 
					//directions might not be valid and end up outside of the triangulation
					//This problem could be solved if we add a "supertriangle" covering all points

					lastEdge = includes_p1[0];

					//Dont go in this direction because then we are outside of the triangulation
					//Sometimes we may have picked the wrong direction when we rotate from the first triangle 
					//and rotated around towards a triangle that's at the border, if so we have to restart and rotate
					//in the other direction
					if (lastEdge.oppositeEdge == null || restart)
					{
						lastEdge = includes_p1[1];
					}

					//The triangle we rotate to
					t = lastEdge.oppositeEdge.t;
				}
				else
				{
					//Move in the direction that doesnt include the last edge
					if (includes_p1[0].oppositeEdge != lastEdge)
					{
						lastEdge = includes_p1[0];
					}
					else
					{
						lastEdge = includes_p1[1];
					}

					//If we have hit a border edge, we should have rotated in the other direction when we started at the first triangle
					//So we have to jump back
					if (lastEdge.oppositeEdge == null)
					{
						restart = true;

						t = startTriangle;

						lastEdge = null;
					}
					else
					{
						//The triangle we rotate to
						t = lastEdge.oppositeEdge.t;
					}
				}
			}


			//Step3. March from one triangle to the next in the general direction of p2
			//This means we always move across the edge of the triangle that intersects with the constraint
			int safety2 = 0;

			lastEdge = null;

			while (true)
			{
				safety2 += 1;

				if (safety2 > 10000)
				{
					Console.WriteLine("Stuck in infinite loop when finding intersecting edges");

					break;
				}

				//The three edges belonging to the current triangle
				HalfEdge e1 = t.halfEdge;
				HalfEdge e2 = e1.nextEdge;
				HalfEdge e3 = e2.nextEdge;

				//Does this triangle include the last vertex on the constraint edge? If so we have found all edges that intersects
				if (e1.v.position == p2 || e2.v.position == p2 || e3.v.position == p2)
				{
					break;
				}
				//Find which edge that intersects with the constraint
				//More than one edge maight intersect, so we have to check if it's not the edge we are coming from
				else
				{
					//Save the edge that intersects in case the triangle intersects with two edges
					if (e1.oppositeEdge != lastEdge && IsEdgeCrossingEdge(e1.v.position, e1.prevEdge.v.position, p1, p2))
					{
						lastEdge = e1;
					}
					else if (e2.oppositeEdge != lastEdge && IsEdgeCrossingEdge(e2.v.position, e2.prevEdge.v.position, p1, p2))
					{
						lastEdge = e2;
					}
					else
					{
						lastEdge = e3;
					}

					//Jump to the next triangle by crossing the edge that intersects with the constraint
					t = lastEdge.oppositeEdge.t;

					//Save the intersecting edge
					intersectingEdges.Add(lastEdge);
				}
			}

			return intersectingEdges;
		}

		//Check if an edge is intersecting with the constraint edge between p1 and p2
		//If so, add it to the list if the edge doesnt exist in the list
		private static void TryAddEdgeToIntersectingEdges(HalfEdge e, WPos p1, WPos p2, List<HalfEdge> intersectingEdges)
		{
			//The position the edge is going to
			var e_p1 = e.v.position;
			//The position the edge is coming from
			var e_p2 = e.prevEdge.v.position;

			//Is this edge intersecting with the constraint?
			if (IsEdgeCrossingEdge(e_p1, e_p2, p1, p2))
			{
				//Add it to the list if it isnt already in the list
				for (int i = 0; i < intersectingEdges.Count; i++)
				{
					//In the half-edge data structure, theres another edge on the opposite side going in the other direction
					//so we have to check both because we want unique edges
					if (intersectingEdges[i] == e || intersectingEdges[i].oppositeEdge == e)
					{
						//The edge is already in the list
						return;
					}
				}

				//The edge is not in the list so add it
				intersectingEdges.Add(e);
			}
		}

		public static bool AreLinesIntersecting(WPos l1_p1, WPos l1_p2, WPos l2_p1, WPos l2_p2, bool shouldIncludeEndPoints)
		{
			//To avoid floating point precision issues we can add a small value
			float epsilon = 0.00001f;

			bool isIntersecting = false;

			float denominator = (l2_p2.Y - l2_p1.Y) * (l1_p2.X - l1_p1.X) - (l2_p2.X - l2_p1.X) * (l1_p2.Y - l1_p1.Y);

			//Make sure the denominator is > 0, if not the lines are parallel
			if (denominator != 0f)
			{
				float u_a = ((l2_p2.X - l2_p1.X) * (l1_p1.Y - l2_p1.Y) - (l2_p2.Y - l2_p1.Y) * (l1_p1.X - l2_p1.X)) / denominator;
				float u_b = ((l1_p2.X - l1_p1.X) * (l1_p1.Y - l2_p1.Y) - (l1_p2.Y - l1_p1.Y) * (l1_p1.X - l2_p1.X)) / denominator;

				//Are the line segments intersecting if the end points are the same
				if (shouldIncludeEndPoints)
				{
					//Is intersecting if u_a and u_b are between 0 and 1 or exactly 0 or 1
					if (u_a >= 0f + epsilon && u_a <= 1f - epsilon && u_b >= 0f + epsilon && u_b <= 1f - epsilon)
					{
						isIntersecting = true;
					}
				}
				else
				{
					//Is intersecting if u_a and u_b are between 0 and 1
					if (u_a > 0f + epsilon && u_a < 1f - epsilon && u_b > 0f + epsilon && u_b < 1f - epsilon)
					{
						isIntersecting = true;
					}
				}
			}

			return isIntersecting;
		}

		//Is an edge crossing another edge? 
		private static bool IsEdgeCrossingEdge(WPos e1_p1, WPos e1_p2, WPos e2_p1, WPos e2_p2)
		{
			//We will here run into floating point precision issues so we have to be careful
			//To solve that you can first check the end points 
			//and modify the line-line intersection algorithm to include a small epsilon

			//First check if the edges are sharing a point, if so they are not crossing
			if (e1_p1 == e2_p1 || e1_p1 == e2_p2 || e1_p2 == e2_p1 || e1_p2 == e2_p2)
			{
				return false;
			}

			//Then check if the lines are intersecting
			if (!AreLinesIntersecting(e1_p1.XZ(), e1_p2.XZ(), e2_p1.XZ(), e2_p2.XZ(), false))
			{
				return false;
			}

			return true;
		}

		//Is an edge (between p1 and p2) a part of an edge in the triangulation?
		private static bool IsEdgePartOfTriangulation(List<Triangle> triangulation, WPos p1, WPos p2)
		{
			for (int i = 0; i < triangulation.Count; i++)
			{
				//The vertices positions of the current triangle
				var t_p1 = triangulation[i].v1;
				var t_p2 = triangulation[i].v2;
				var t_p3 = triangulation[i].v3;

				//Check if any of the triangle's edges have the same coordinates as the constrained edge
				//We have no idea about direction so we have to check both directions
				if ((t_p1 == p1 && t_p2 == p2) || (t_p1 == p2 && t_p2 == p1))
				{
					return true;
				}
				if ((t_p2 == p1 && t_p3 == p2) || (t_p2 == p2 && t_p3 == p1))
				{
					return true;
				}
				if ((t_p3 == p1 && t_p1 == p2) || (t_p3 == p2 && t_p1 == p1))
				{
					return true;
				}
			}

			return false;
		}
	}
}

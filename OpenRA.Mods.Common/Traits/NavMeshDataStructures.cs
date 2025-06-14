using System.Collections;
using System.Collections.Generic;

namespace OpenRA.Mods.Common.Traits
{
	public class Vertex
	{
		public WPos position;

		//The outgoing halfedge (a halfedge that starts at this vertex)
		//Doesnt matter which edge we connect to it
		public HalfEdge halfEdge;

		public Vertex(WPos position)
		{
			this.position = position;
		}

		//Get 2d pos of this vertex
		public WPos GetPos2D_XZ()
		{
			var pos_2d_xz = new WPos(position.X, position.Z, 0);

			return pos_2d_xz;
		}

		public WPos ToWPos() => position;
	}

	public class HalfEdge
	{
		//The vertex the edge points to
		public Vertex v;

		//The face this edge is a part of
		public Triangle t;

		//The next edge
		public HalfEdge nextEdge;
		//The previous
		public HalfEdge prevEdge;
		//The edge going in the opposite direction
		public HalfEdge oppositeEdge;

		//This structure assumes we have a vertex class with a reference to a half edge going from that vertex
		//and a face (triangle) class with a reference to a half edge which is a part of this face
		public HalfEdge(Vertex v)
		{
			this.v = v;
		}

		public HalfEdge(WPos p)
		{
			this.v = new Vertex(p);
		}
	}

	public class Triangle
	{
		//Corners
		public WPos v1;
		public WPos v2;
		public WPos v3;

		public List<WPos> AsList() => new() { v1, v2, v3 };

		//If we are using the half edge mesh structure, we just need one half edge
		public HalfEdge halfEdge;

		public Triangle(Vertex v1, Vertex v2, Vertex v3)
		{
			this.v1 = v1.ToWPos();
			this.v2 = v2.ToWPos();
			this.v3 = v3.ToWPos();
		}

		public Triangle(WPos v1, WPos v2, WPos v3)
		{
			this.v1 = v1;
			this.v2 = v2;
			this.v3 = v3;
		}

		public Triangle(HalfEdge halfEdge)
		{
			this.halfEdge = halfEdge;
		}

		//Change orientation of triangle from cw -> ccw or ccw -> cw
		public void ChangeOrientation()
		{
			WPos temp = this.v1;

			this.v1 = this.v2;

			this.v2 = temp;
		}
	}

	//And edge between two vertices
	public class Edge
	{
		public Vertex v1;
		public Vertex v2;

		//Is this edge intersecting with another edge?
		public bool isIntersecting = false;

		public Edge(Vertex v1, Vertex v2)
		{
			this.v1 = v1;
			this.v2 = v2;
		}

		public Edge(WPos v1, WPos v2)
		{
			this.v1 = new Vertex(v1);
			this.v2 = new Vertex(v2);
		}

		//Get vertex in 2d space (assuming x, z)
		public WPos GetVertex2D(Vertex v)
		{
			return new WPos(v.position.X, v.position.Z);
		}

		//Flip edge
		public void FlipEdge()
		{
			Vertex temp = v1;

			v1 = v2;

			v2 = temp;
		}
	}

	public class Plane
	{
		public WPos pos;

		public WPos normal;

		public Plane(WPos pos, WPos normal)
		{
			this.pos = pos;

			this.normal = normal;
		}
	}
}

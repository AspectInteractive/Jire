using System;
using System.Collections.Generic;
using System.Linq;

// Fixed point math implementation (16.16 format)
public struct Fixed : IComparable<Fixed>, IEquatable<Fixed>
{
    private const int SHIFT = 16;
    private const long SCALE = 1L << SHIFT;
    private readonly long value;

    public static readonly Fixed Zero = new Fixed(0L, true);
    public static readonly Fixed One = new Fixed(SCALE, true);
    public static readonly Fixed MaxValue = new Fixed(long.MaxValue, true);

    private Fixed(long rawValue, bool dummy)
    {
        value = rawValue;
    }

    public Fixed(int intValue)
    {
        value = (long)intValue << SHIFT;
    }

    public Fixed(float floatValue)
    {
        value = (long)(floatValue * SCALE);
    }

    public static Fixed FromRaw(long rawValue) => new Fixed(rawValue, true);

    public static Fixed operator +(Fixed a, Fixed b) => FromRaw(a.value + b.value);
    public static Fixed operator -(Fixed a, Fixed b) => FromRaw(a.value - b.value);
    public static Fixed operator *(Fixed a, Fixed b) => FromRaw((a.value * b.value) >> SHIFT);
    public static Fixed operator /(Fixed a, Fixed b) => FromRaw((a.value << SHIFT) / b.value);
    public static Fixed operator -(Fixed a) => FromRaw(-a.value);

    public static bool operator <(Fixed a, Fixed b) => a.value < b.value;
    public static bool operator >(Fixed a, Fixed b) => a.value > b.value;
    public static bool operator <=(Fixed a, Fixed b) => a.value <= b.value;
    public static bool operator >=(Fixed a, Fixed b) => a.value >= b.value;
    public static bool operator ==(Fixed a, Fixed b) => a.value == b.value;
    public static bool operator !=(Fixed a, Fixed b) => a.value != b.value;

    public Fixed Abs() => value < 0 ? FromRaw(-value) : this;
    public float ToFloat() => (float)value / SCALE;
    public int ToInt() => (int)(value >> SHIFT);

    public Fixed Sqrt()
    {
        if (value <= 0) return Zero;

        // Newton-Raphson method
        Fixed x = this;
        Fixed prev;
        for (int i = 0; i < 10; i++)
        {
            prev = x;
            x = (x + this / x) / new Fixed(2);
            if ((x - prev).Abs() < FromRaw(1)) break;
        }
        return x;
    }

    public int CompareTo(Fixed other) => value.CompareTo(other.value);
    public bool Equals(Fixed other) => value == other.value;
    public override bool Equals(object obj) => obj is Fixed other && Equals(other);
    public override int GetHashCode() => value.GetHashCode();
    public override string ToString() => ToFloat().ToString("F3");
}

// 2D Vector with fixed point coordinates
public struct FixedVector2 : IEquatable<FixedVector2>
{
    public Fixed X { get; }
    public Fixed Y { get; }

    public FixedVector2(Fixed x, Fixed y)
    {
        X = x;
        Y = y;
    }

    public FixedVector2(float x, float y) : this(new Fixed(x), new Fixed(y)) { }

    public static FixedVector2 operator +(FixedVector2 a, FixedVector2 b) => new FixedVector2(a.X + b.X, a.Y + b.Y);
    public static FixedVector2 operator -(FixedVector2 a, FixedVector2 b) => new FixedVector2(a.X - b.X, a.Y - b.Y);
    public static FixedVector2 operator *(FixedVector2 v, Fixed scalar) => new FixedVector2(v.X * scalar, v.Y * scalar);

    public Fixed Dot(FixedVector2 other) => X * other.X + Y * other.Y;
    public Fixed LengthSquared() => X * X + Y * Y;
    public Fixed Length() => LengthSquared().Sqrt();
    public Fixed DistanceTo(FixedVector2 other) => (this - other).Length();

    public FixedVector2 Normalize()
    {
        Fixed len = Length();
        return len > Fixed.Zero ? new FixedVector2(X / len, Y / len) : new FixedVector2(Fixed.Zero, Fixed.Zero);
    }

    public bool Equals(FixedVector2 other) => X == other.X && Y == other.Y;
    public override bool Equals(object obj) => obj is FixedVector2 other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(X, Y);
    public override string ToString() => $"({X}, {Y})";
}

// Navigation mesh triangle
public class NavTriangle
{
    public int Id { get; }
    public FixedVector2[] Vertices { get; }
    public List<int> Neighbors { get; }
    public FixedVector2 Center { get; }

    public NavTriangle(int id, FixedVector2 v0, FixedVector2 v1, FixedVector2 v2)
    {
        Id = id;
        Vertices = new[] { v0, v1, v2 };
        Neighbors = new List<int>();
        Center = new FixedVector2(
            (v0.X + v1.X + v2.X) / new Fixed(3),
            (v0.Y + v1.Y + v2.Y) / new Fixed(3)
        );
    }

    // Check if point is inside triangle using barycentric coordinates
    public bool ContainsPoint(FixedVector2 point)
    {
        var v0 = Vertices[2] - Vertices[0];
        var v1 = Vertices[1] - Vertices[0];
        var v2 = point - Vertices[0];

        var dot00 = v0.Dot(v0);
        var dot01 = v0.Dot(v1);
        var dot02 = v0.Dot(v2);
        var dot11 = v1.Dot(v1);
        var dot12 = v1.Dot(v2);

        var invDenom = Fixed.One / (dot00 * dot11 - dot01 * dot01);
        var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return u >= Fixed.Zero && v >= Fixed.Zero && u + v <= Fixed.One;
    }

    // Get the edge midpoint connecting to a neighbor triangle
    public FixedVector2 GetConnectionPoint(int neighborId)
    {
        int neighborIndex = Neighbors.IndexOf(neighborId);
        if (neighborIndex == -1) return Center;

        // Return midpoint of the shared edge
        var v1 = Vertices[neighborIndex];
        var v2 = Vertices[(neighborIndex + 1) % 3];
        return new FixedVector2((v1.X + v2.X) / new Fixed(2), (v1.Y + v2.Y) / new Fixed(2));
    }
}

// A* pathfinding node
public class PathNode : IComparable<PathNode>
{
    public int TriangleId { get; }
    public Fixed GCost { get; set; }
    public Fixed HCost { get; set; }
    public Fixed FCost => GCost + HCost;
    public PathNode Parent { get; set; }

    public PathNode(int triangleId)
    {
        TriangleId = triangleId;
        GCost = Fixed.Zero;
        HCost = Fixed.Zero;
    }

    public int CompareTo(PathNode other)
    {
        int result = FCost.CompareTo(other.FCost);
        return result != 0 ? result : HCost.CompareTo(other.HCost);
    }
}

// Main navigation mesh class
public class NavMesh
{
    private readonly Dictionary<int, NavTriangle> triangles;
    private readonly Dictionary<FixedVector2, int> spatialHash;

    public NavMesh()
    {
        triangles = new Dictionary<int, NavTriangle>();
        spatialHash = new Dictionary<FixedVector2, int>();
    }

    public void AddTriangle(NavTriangle triangle)
    {
        triangles[triangle.Id] = triangle;

        // Simple spatial hashing for faster point queries
        var hashKey = new FixedVector2(
            new Fixed((int)(triangle.Center.X.ToFloat() / 10) * 10),
            new Fixed((int)(triangle.Center.Y.ToFloat() / 10) * 10)
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

    // Find which triangle contains the given point
    public int FindTriangleContaining(FixedVector2 point)
    {
        // Try spatial hash first
        var hashKey = new FixedVector2(
            new Fixed((int)(point.X.ToFloat() / 10) * 10),
            new Fixed((int)(point.Y.ToFloat() / 10) * 10)
        );

        if (spatialHash.TryGetValue(hashKey, out int candidateId))
        {
            if (triangles[candidateId].ContainsPoint(point))
                return candidateId;
        }

        // Fallback: check all triangles
        foreach (var triangle in triangles.Values)
        {
            if (triangle.ContainsPoint(point))
                return triangle.Id;
        }

        return -1; // Not found
    }

    // A* pathfinding between two points
    public List<FixedVector2> FindPath(FixedVector2 start, FixedVector2 end)
    {
        int startTriangle = FindTriangleContaining(start);
        int endTriangle = FindTriangleContaining(end);

        if (startTriangle == -1 || endTriangle == -1)
            return new List<FixedVector2>(); // Invalid start or end point

        if (startTriangle == endTriangle)
            return new List<FixedVector2> { start, end }; // Direct path

        var openSet = new SortedSet<PathNode>();
        var allNodes = new Dictionary<int, PathNode>();
        var closedSet = new HashSet<int>();

        var startNode = new PathNode(startTriangle);
        allNodes[startTriangle] = startNode;
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            var current = openSet.Min;
            openSet.Remove(current);
            closedSet.Add(current.TriangleId);

            if (current.TriangleId == endTriangle)
            {
                return ReconstructPath(current, start, end);
            }

            var currentTriangle = triangles[current.TriangleId];
            foreach (int neighborId in currentTriangle.Neighbors)
            {
                if (closedSet.Contains(neighborId) || !triangles.ContainsKey(neighborId))
                    continue;

                var neighborTriangle = triangles[neighborId];
                var tentativeGCost = current.GCost + currentTriangle.Center.DistanceTo(neighborTriangle.Center);

                if (!allNodes.TryGetValue(neighborId, out var neighbor))
                {
                    neighbor = new PathNode(neighborId);
                    allNodes[neighborId] = neighbor;
                }

                if (tentativeGCost < neighbor.GCost || !openSet.Contains(neighbor))
                {
                    neighbor.GCost = tentativeGCost;
                    neighbor.HCost = neighborTriangle.Center.DistanceTo(triangles[endTriangle].Center);
                    neighbor.Parent = current;

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }

        return new List<FixedVector2>(); // No path found
    }

    private List<FixedVector2> ReconstructPath(PathNode endNode, FixedVector2 start, FixedVector2 end)
    {
        var path = new List<FixedVector2> { end };
        var current = endNode;

        while (current.Parent != null)
        {
            var currentTriangle = triangles[current.TriangleId];
            var parentTriangle = triangles[current.Parent.TriangleId];
            var connectionPoint = currentTriangle.GetConnectionPoint(current.Parent.TriangleId);
            path.Add(connectionPoint);
            current = current.Parent;
        }

        path.Add(start);
        path.Reverse();
        return path;
    }

    // Helper method to create a simple test nav mesh
    public static NavMesh CreateTestMesh()
    {
        var navMesh = new NavMesh();

        // Create a simple diamond-shaped nav mesh
        var tri1 = new NavTriangle(1, new FixedVector2(0, 0), new FixedVector2(5, 5), new FixedVector2(0, 10));
        var tri2 = new NavTriangle(2, new FixedVector2(0, 0), new FixedVector2(5, -5), new FixedVector2(5, 5));
        var tri3 = new NavTriangle(3, new FixedVector2(5, 5), new FixedVector2(10, 0), new FixedVector2(5, -5));
        var tri4 = new NavTriangle(4, new FixedVector2(5, 5), new FixedVector2(10, 10), new FixedVector2(10, 0));

        navMesh.AddTriangle(tri1);
        navMesh.AddTriangle(tri2);
        navMesh.AddTriangle(tri3);
        navMesh.AddTriangle(tri4);

        // Set up neighbor relationships
        navMesh.SetNeighbors(1, 2);
        navMesh.SetNeighbors(2, 1, 3);
        navMesh.SetNeighbors(3, 2, 4);
        navMesh.SetNeighbors(4, 3);

        return navMesh;
    }
}

// Example usage
public class NavMeshExample
{
    public static void Main()
    {
        var navMesh = NavMesh.CreateTestMesh();

        var start = new FixedVector2(1, 1);
        var end = new FixedVector2(9, 1);

        var path = navMesh.FindPath(start, end);

        Console.WriteLine($"Path from {start} to {end}:");
        foreach (var point in path)
        {
            Console.WriteLine($"  -> {point}");
        }
    }
}
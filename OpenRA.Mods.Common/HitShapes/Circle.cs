#region Copyright & License Information
/*
 * Copyright (c) The OpenRA Developers and Contributors
 * This file is part of OpenRA, which is free software. It is made
 * available to you under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. For more
 * information, see COPYING.
 */
#endregion

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using DiscordRPC;
using OpenRA.Graphics;
using OpenRA.Mods.Common.Graphics;
using OpenRA.Mods.Common.Traits;
using OpenRA.Mods.Common.Traits.BotModules.Squads;
using OpenRA.Primitives;
using static OpenRA.Mods.Common.Traits.MobileOffGridOverlay;

#pragma warning disable SA1005 // Single line comments should begin with single space
#pragma warning disable SA1515 // Single-line comment should be preceded by blank line
#pragma warning disable SA1108 // Block statements should not contain embedded comments
#pragma warning disable SA1513 // Closing brace should be followed by blank line

namespace OpenRA.Mods.Common.HitShapes
{
	public class CircleShape : IHitShape
	{
		public WDist OuterRadius => Radius;

		[FieldLoader.Require]
		public readonly WDist Radius = new(426);

		[Desc("Defines the top offset relative to the actor's center.")]
		public readonly int VerticalTopOffset = 0;

		[Desc("Defines the bottom offset relative to the actor's center.")]
		public readonly int VerticalBottomOffset = 0;

		public CircleShape() { }

		public CircleShape(WDist radius) { Radius = radius; }

		public void Initialize()
		{
			if (VerticalTopOffset < VerticalBottomOffset)
				throw new YamlException("VerticalTopOffset must be equal to or higher than VerticalBottomOffset.");
		}
		bool IHitShape.IsOverlapping(IHitShape shape, WPos selfCenter, WPos otherCenter)
		{
			if (shape is CircleShape circleShape)
				return IsOverlapping(circleShape, selfCenter, otherCenter);

			// If it is called with an overlapping shape that is not a circle, we have not yet implemented this
			throw new NotImplementedException();
		}

		internal bool IsOverlapping(CircleShape otherCircle, WPos selfCenter, WPos otherCircleCenter)
			=> (Radius + otherCircle.Radius).Length > (otherCircleCenter - selfCenter).Length;

		public WDist DistanceFromEdge(in WVec v)
		{
			return new WDist(Math.Max(0, v.Length - Radius.Length));
		}

		public WDist DistanceFromEdge(WPos pos, WPos origin, WRot orientation)
		{
			if (pos.Z > origin.Z + VerticalTopOffset)
				return DistanceFromEdge(pos - (origin + new WVec(0, 0, VerticalTopOffset)));

			if (pos.Z < origin.Z + VerticalBottomOffset)
				return DistanceFromEdge(pos - (origin + new WVec(0, 0, VerticalBottomOffset)));

			return DistanceFromEdge(pos - new WPos(origin.X, origin.Y, pos.Z));
		}

		static List<WPos> GetPointsSurroundingCircleUnit(int2 selfCenter, WDist unitRadius) =>
			GetPointsSurroundingCircleUnit(selfCenter, unitRadius, WAngle.Zero);

		static List<WPos> GetPointsSurroundingCircleUnit(int2 selfCenter, WDist unitRadius, WAngle initialRotation, int angleIncAmount = 128)
		{
			var radiusVec = new WVec(unitRadius, WRot.FromYaw(initialRotation));
			var points = new List<WPos>(); // Points surrounding circle
			for (var a = 0; a < 1024; a += angleIncAmount) // 128 = 45 degrees,
				points.Add(new WPos(selfCenter.X, selfCenter.Y, 0) + radiusVec.Rotate(new WRot(WAngle.Zero, WAngle.Zero, new WAngle(a))));
			return points;
		}

		WPos[] IHitShape.GetCorners(int2 selfCenter) => GetPointsSurroundingCircleUnit(selfCenter, Radius).ToArray();

		public static Fix64 Sq(Fix64 i) { return i * i; }
		public static Fix64 Sqrt(Fix64 i) { return Fix64.Sqrt(i); }
		public static double Sq(double i) { return i * i; }
		public static double Sqrt(double i) { return (double)Math.Sqrt(i); }
		public static int Sq(int i) { return Exts.ISqr(i); }
		public static int Sqrt(int i) { return Exts.ISqrt(i); }

		public bool PosIsInsideCircle(WPos circleCenter, WPos checkPos) { return PosIsInsideCircle(circleCenter, Radius.Length, checkPos); }
		public static bool PosIsInsideCircle(WPos circleCenter, int radius, WPos checkPos)
		{
			var xDelta = circleCenter.X - checkPos.X;
			var yDelta = circleCenter.Y - checkPos.Y;
			var delta = (long)Sq(xDelta) + Sq(yDelta);
			return Sqrt(delta) < radius;
		}

		public static int GetSliceCount(int angleToCutCircleSlices)
		{ return angleToCutCircleSlices != 0 ? (int)((Fix64)360 / (Fix64)angleToCutCircleSlices) : -1; }

		public Fix64 GetSliceAngle(int sliceIndex, int angleToCutCircleSlices)
		{
			var nSlices = GetSliceCount(angleToCutCircleSlices);
			var oneSliceAngle = (Fix64)2.0 * Fix64.Pi / (Fix64)nSlices;
			return oneSliceAngle * (Fix64)sliceIndex;
		}

		public int CalcCircleSliceIndex(WPos circleCenter, WPos checkPos, int angleToCutCircleSlices)
		{ return CalcCircleSliceIndex(circleCenter, Radius.Length, checkPos, angleToCutCircleSlices); }
		public static int CalcCircleSliceIndex(WPos circleCenter, int radius, WPos checkPos, int angleToCutCircleSlices)
		{
			var nSlices = GetSliceCount(angleToCutCircleSlices);
			if (nSlices > 0 && PosIsInsideCircle(circleCenter, radius, checkPos))
			{
				var angle = Fix64.Atan2((Fix64)(circleCenter.Y - checkPos.Y), (Fix64)(circleCenter.X - checkPos.X));
				if (angle < Fix64.Zero)
					angle = Fix64.Pi - angle;
				var sliceAngle = (Fix64)2.0 * Fix64.Pi / (Fix64)nSlices;
				return (int)(angle / sliceAngle);
			}
			else
				return -1;
		}

		public static bool PointIsWithinLineSegment(WPos checkPoint, WPos lineP1, WPos lineP2)
		{
			// if we knew lineP1.X < lineP2.X, then we could use lineP1.X <= checkPoint.X && checkPoint.X <= lineP2.X
			var deltaVec = lineP2 - lineP1;
			var slope = Fix64.Abs(new Fix64(deltaVec.Y) / new Fix64(deltaVec.X));
			Func<WPos, int> getC; // get a coordinate
			if (slope <= new Fix64(1))
				getC = p => p.X;
			else
				getC = p => p.Y;
			return getC(checkPoint) < getC(lineP1) ^ getC(checkPoint) <= getC(lineP2); // if this does not work use checkPoint.X or checkPoint.Y etc.
		}

		bool IHitShape.LineIntersectsOrIsInside(WPos circleCenter, WPos p1, WPos p2) => LineIsCollidingLogic(circleCenter, p1, p2);

		bool LineIsCollidingLogic(WPos circleCenter, WPos p1, WPos p2)
		{
			if (PosIsInsideCircle(circleCenter, Radius.Length, p1) ||
				PosIsInsideCircle(circleCenter, Radius.Length, p2) ||
				IntersectingPosesFromLine(circleCenter, Radius.Length, p1, p2).Count > 0)
				return true;
			return false;
		}

		static int TriangleArea(WPos a, WPos b, WPos c)
		{
			var ab = b - a;
			var ac = c - a;
			var crossProduct = ab.X * ac.Y - ab.Y * ac.X;
			return (int)((Fix64)Math.Abs(crossProduct) / (Fix64)2);
		}
		public static List<WPos?> CircleCircleIntersections(WPos selfCenter, WDist selfRadius, WPos otherCenter, WDist otherRadius)
			=> CircleCircleIntersections((Fix64)selfCenter.X, (Fix64)selfCenter.Y, (Fix64)selfRadius.Length,
										 (Fix64)otherCenter.X, (Fix64)otherCenter.Y, (Fix64)otherRadius.Length);

		public static bool DoesCircleAlongLineCollideWithCircle(WPos lineStart, WPos lineEnd, WDist selfRadius, WPos otherCenter, WDist otherRadius)
		{
			// Calculate denominator first to check for division by zero
			var denominator = (Fix64)((lineEnd.X - lineStart.X) * (lineEnd.X - lineStart.X) +
									 (lineEnd.Y - lineStart.Y) * (lineEnd.Y - lineStart.Y));

			// If denominator is zero, the start and end points are the same
			// In this case, just check if the single point overlaps with the circle
			if (denominator == Fix64.Zero)
			{
				var dx = otherCenter.X - lineStart.X;
				var dy = otherCenter.Y - lineStart.Y;
				var distanceSquared = (Fix64)(dx * dx + dy * dy);
				var radiusSum = (Fix64)(selfRadius.Length + otherRadius.Length);
				return distanceSquared <= radiusSum * radiusSum;
			}

			var t = Math.Max(0, Math.Min(1, (int)(
				((Fix64)(otherCenter.X - lineStart.X) * (Fix64)(lineEnd.X - lineStart.X) +
				 (Fix64)(otherCenter.Y - lineStart.Y) * (Fix64)(lineEnd.Y - lineStart.Y)) /
				denominator)));

			var closestX = (Fix64)lineStart.X + (Fix64)t * (Fix64)(lineEnd.X - lineStart.X);
			var closestY = (Fix64)lineStart.Y + (Fix64)t * (Fix64)(lineEnd.Y - lineStart.Y);
			var closestDistance = Fix64.Sqrt(((Fix64)otherCenter.X - closestX) * ((Fix64)otherCenter.X - closestX) +
										   ((Fix64)otherCenter.Y - closestY) * ((Fix64)otherCenter.Y - closestY));

			return (int)closestDistance <= selfRadius.Length + otherRadius.Length;
		}

		private static Fix64 ClosestDistance(WPos lineStart, WPos lineEnd, WPos point)
		{
			var dx = lineEnd.X - lineStart.X;
			var dy = lineEnd.Y - lineStart.Y;

			if (dx == 0 && dy == 0)
				return Fix64.Sqrt((Fix64)((point.X - lineStart.X) * (point.X - lineStart.X) +
										 (point.Y - lineStart.Y) * (point.Y - lineStart.Y)));

			var t = Fix64.Max(Fix64.Zero, Fix64.Min(Fix64.One,
				((Fix64)(point.X - lineStart.X) * (Fix64)dx + (Fix64)(point.Y - lineStart.Y) * (Fix64)dy) /
				(Fix64)(dx * dx + dy * dy)));

			var closestX = (Fix64)lineStart.X + t * (Fix64)dx;
			var closestY = (Fix64)lineStart.Y + t * (Fix64)dy;

			return Fix64.Sqrt((Fix64)((point.X - (int)closestX) * (point.X - (int)closestX) +
									  (point.Y - (int)closestY) * (point.Y - (int)closestY)));
		}

		public static bool CheckOverlap(WPos lineStart, WPos lineEnd, WDist selfRadius, WPos otherCenter, WDist otherRadius)
		{
			var distance = ClosestDistance(lineStart, lineEnd, otherCenter);
			return distance <= (Fix64)selfRadius.Length + (Fix64)otherRadius.Length;
		}


		public static bool CapsuleIntersectsSquare(WPos squareTopLeft, WDist squareWidth, WDist capsuleRadius, WPos startPos, WPos endPos)
		{
			static float Distance(WPos a, WPos b)
			{
				float dx = a.X - b.X;
				float dy = a.Y - b.Y;
				return (float)Math.Sqrt(dx * dx + dy * dy);
			}

			static WPos ClosestPointOnLineSegment(WPos lineStart, WPos lineEnd, WPos point)
			{
				static float Dot(WVec a, WVec b) => a.X * b.X + a.Y * b.Y;

				var lineDirection = lineEnd - lineStart;
				var lineLength = lineDirection.Length;
				lineDirection /= lineLength;

				var vector = point - lineStart;
				var d = Dot(vector, lineDirection);

				if (d <= 0)
					return lineStart;
				else if (d >= lineLength)
					return lineEnd;
				else
					return lineStart + new WVec((int)(lineDirection.X * d), (int)(lineDirection.Y * d), 0);
			}

			static float DistanceFromLineSegment(WPos lineStart, WPos lineEnd, WPos point)
			{
				var closestPoint = ClosestPointOnLineSegment(lineStart, lineEnd, point);
				return Distance(point, closestPoint);
			}

			static WPos ClosestPointOnRectangle(WPos lineStart, WPos lineEnd, WPos rectTopLeft, WPos rectBotRight)
			{
				var closestPoint = ClosestPointOnLineSegment(lineStart, lineEnd, rectTopLeft);
				var closestDistance = Distance(closestPoint, ClosestPointOnLineSegment(lineStart, lineEnd, closestPoint));

				// Check the top edge
				var topEdgePoint = ClosestPointOnLineSegment(lineStart, lineEnd, ClosestPointOnLineSegment(rectTopLeft, new WPos(rectBotRight.X, rectTopLeft.Y, 0), lineStart));
				var topEdgeDistance = Distance(topEdgePoint, ClosestPointOnLineSegment(lineStart, lineEnd, topEdgePoint));
				if (topEdgeDistance < closestDistance)
				{
					closestPoint = topEdgePoint;
					closestDistance = topEdgeDistance;
				}

				// Check the right edge
				var rightEdgePoint = ClosestPointOnLineSegment(lineStart, lineEnd, ClosestPointOnLineSegment(new WPos(rectBotRight.X, rectTopLeft.Y, 0), rectBotRight, lineStart));
				var rightEdgeDistance = Distance(rightEdgePoint, ClosestPointOnLineSegment(lineStart, lineEnd, rightEdgePoint));
				if (rightEdgeDistance < closestDistance)
				{
					closestPoint = rightEdgePoint;
					closestDistance = rightEdgeDistance;
				}

				// Check the bottom edge
				var botEdgePoint = ClosestPointOnLineSegment(lineStart, lineEnd, ClosestPointOnLineSegment(rectBotRight, new WPos(rectTopLeft.X, rectBotRight.Y, 0), lineStart));
				var botEdgeDistance = Distance(botEdgePoint, ClosestPointOnLineSegment(lineStart, lineEnd, botEdgePoint));
				if (botEdgeDistance < closestDistance)
				{
					closestPoint = botEdgePoint;
					closestDistance = botEdgeDistance;
				}

				// Check the left edge
				var leftEdgePoint = ClosestPointOnLineSegment(lineStart, lineEnd, ClosestPointOnLineSegment(new WPos(rectTopLeft.X, rectBotRight.Y, 0), rectTopLeft, lineStart));
				var leftEdgeDistance = Distance(leftEdgePoint, ClosestPointOnLineSegment(lineStart, lineEnd, leftEdgePoint));
				if (leftEdgeDistance < closestDistance)
				{
					closestPoint = leftEdgePoint;
					closestDistance = leftEdgeDistance;
				}

				return closestPoint;
			}

			var squareBotRight = new WPos(squareTopLeft.X + squareWidth.Length, squareTopLeft.Y + squareWidth.Length, 0);

			// Calculate the closest point on the square to the capsule's line segment
			var closestPoint = ClosestPointOnRectangle(startPos, endPos, squareTopLeft, squareBotRight);

			// Calculate the distance between the closest point and the capsule's line segment
			var distance = DistanceFromLineSegment(startPos, endPos, closestPoint);

			// Check if the distance is less than or equal to the capsule's radius
			return distance <= capsuleRadius.Length;
		}

		public static bool WillCircleCollideWithSquare(WPos selfPos, WDist selfRadius, WVec velocity, WPos squareTopLeft, WDist squareWidth)
		{
			// 1. Check if start position overlaps
			if (IsCircleCollidingWithSquare(selfPos, selfRadius, squareTopLeft, squareWidth))
				return true;

			// 2. Calculate end position and check if it overlaps
			var endPos = selfPos + velocity;
			if (IsCircleCollidingWithSquare(endPos, selfRadius, squareTopLeft, squareWidth))
				return true;

			// 3. Get square corners
			var squareRight = squareTopLeft.X + squareWidth.Length;
			var squareBottom = squareTopLeft.Y + squareWidth.Length;
			var corners = new[]
			{
				squareTopLeft,                                         // Top-left
				new WPos(squareRight, squareTopLeft.Y, 0),           // Top-right
				new WPos(squareRight, squareBottom, 0),              // Bottom-right
				new WPos(squareTopLeft.X, squareBottom, 0),          // Bottom-left
			};

			// 4. Check each edge of the square
			for (var i = 0; i < corners.Length; i++)
			{
				var start = corners[i];
				var end = corners[(i + 1) % corners.Length];

				// Use closest point approach to check if movement path comes close enough to edge
				var closestDistance = ClosestDistance(selfPos, endPos, start);
				if (closestDistance <= (Fix64)selfRadius.Length)
					return true;

				// Also check if the movement path crosses this edge
				if (WPos.DoTwoLinesIntersect(selfPos, endPos, start, end))
					return true;
			}

			// 5. Special case: check if movement path goes through square without touching edges
			// This can happen if movement is fast enough to "tunnel" through
			var movementDirection = endPos - selfPos;
			var toSquareCenter = new WPos(squareTopLeft.X + squareWidth.Length / 2,
										 squareTopLeft.Y + squareWidth.Length / 2, 0) - selfPos;

			var projLength = (Fix64)(movementDirection.X * toSquareCenter.X + movementDirection.Y * toSquareCenter.Y) /
							(Fix64)(movementDirection.X * movementDirection.X + movementDirection.Y * movementDirection.Y);

			if (projLength > Fix64.Zero && projLength < Fix64.One)
			{
				var closestPoint = selfPos + new WVec(
					(int)(movementDirection.X * (int)projLength),
					(int)(movementDirection.Y * (int)projLength),
					0);

				if (IsCircleCollidingWithSquare(closestPoint, selfRadius, squareTopLeft, squareWidth))
					return true;
			}

			return false;
		}

		///// <summary>
		///// Detects if a moving circle will collide with a static square
		///// </summary>
		///// <param name="circle">The circle at starting position</param>
		///// <param name="velocity">Movement vector of the circle (only X,Y used)</param>
		///// <param name="square">The static square defined by top-left corner and width</param>
		///// <param name="collisionTime">Time of collision (0-1 range, where 1 = end of movement)</param>
		///// <param name="collisionPoint">Point where collision occurs</param>
		///// <returns>True if collision will occur during movement</returns>
		//public static bool WillCircleCollideWithSquare(WPos selfPos, WDist selfRadius, WVec velocity, WPos squareTopLeft, WDist squareWidth)
		//	//, out Fix64 collisionTime, out WPos collisionPoint)
		//{
		//	var squareLeft = squareTopLeft.X;
		//	var squareRight = squareTopLeft.X + squareWidth.Length;
		//	var squareTop = squareTopLeft.Y;
		//	var squareBottom = squareTopLeft.Y + squareWidth.Length;

		//	//collisionTime = Fix64.MaxValue;
		//	//collisionPoint = WPos.Zero;

		//	// Early exit if circle is stationary
		//	var velocityLengthSq = (Fix64)(velocity.X * velocity.X + velocity.Y * velocity.Y);
		//	if (velocityLengthSq < Fix64.FromRaw(655)) // Very small threshold in fixed point
		//		return IsCircleCollidingWithSquare(selfPos, selfRadius, squareTopLeft, squareWidth);

		//	// Expand square by circle radius to treat circle as a point
		//	var eSquareTopLeft = new WPos(squareLeft - selfRadius.Length, squareTop - selfRadius.Length, 0);
		//	var eSquareWidth = new WDist(squareWidth.Length + 2 * selfRadius.Length);

		//	// Check if ray intersects with expanded square
		//	if (!RayIntersectsRectangle(selfPos, velocity, eSquareTopLeft, eSquareWidth, out Fix64 t))
		//		return false;

		//	// If intersection time is beyond our movement (t > 1), no collision
		//	if (t > Fix64.One || t < Fix64.Zero)
		//		return false;

		//	// Calculate the actual collision point and details
		//	var futureCirclePos = new WPos(
		//		selfPos.X + velocity.X * (int)t,
		//		selfPos.Y + velocity.Y * (int)t,
		//		0);

		//	// Check if it's a corner collision (more complex case)
		//	if (IsCornerCollision(futureCirclePos, squareTopLeft, squareWidth))
		//	{
		//		// Recalculate for precise corner collision
		//		if (CalculateCornerCollision(selfPos, selfRadius, velocity, squareTopLeft, squareWidth, out Fix64 cornerTime, out WPos cornerPoint) &&
		//			cornerTime >= Fix64.Zero && cornerTime <= Fix64.One)
		//		{
		//			//collisionTime = cornerTime;
		//			//collisionPoint = cornerPoint;
		//			return true;
		//		}
		//	}

		//	//collisionTime = t;
		//	//collisionPoint = futureCirclePos;
		//	return true;
		//}

		/// <summary>
		/// Checks if circle is currently colliding with square
		/// </summary>
		public static bool IsCircleCollidingWithSquare(WPos circlePos, WDist circleRadius, WPos squareTopLeft, WDist squareWidth)
		{
			var squareLeft = squareTopLeft.X;
			var squareRight = squareTopLeft.X + squareWidth.Length;
			var squareTop = squareTopLeft.Y;
			var squareBottom = squareTopLeft.Y + squareWidth.Length;

			// Find closest point on square to circle center
			var closestX = Fix64.Max((Fix64)squareLeft, Fix64.Min((Fix64)circlePos.X, (Fix64)squareRight));
			var closestY = Fix64.Max((Fix64)squareTop, Fix64.Min((Fix64)circlePos.Y, (Fix64)squareBottom));

			// Check if distance is less than radius
			var distanceX = (Fix64)circlePos.X - closestX;
			var distanceY = (Fix64)circlePos.Y - closestY;
			var distanceSquared = distanceX * distanceX + distanceY * distanceY;

			return distanceSquared <= (Fix64)circleRadius.Length * (Fix64)circleRadius.Length;
		}

		public static bool CheckOverlapSquare(WPos startPos, WPos endPos, WDist selfRadius, WPos squareTopLeft, WDist squareWidth)
		{
			// 1. Check if either start or end position overlaps with the square
			if (IsCircleCollidingWithSquare(startPos, selfRadius, squareTopLeft, squareWidth) ||
				IsCircleCollidingWithSquare(endPos, selfRadius, squareTopLeft, squareWidth))
				return true;

			// 2. Get the four corners of the square
			var squarePoints = new[]
			{
				squareTopLeft, // Top-left
				new WPos(squareTopLeft.X + squareWidth.Length, squareTopLeft.Y, 0), // Top-right
				new WPos(squareTopLeft.X + squareWidth.Length, squareTopLeft.Y + squareWidth.Length, 0), // Bottom-right
				new WPos(squareTopLeft.X, squareTopLeft.Y + squareWidth.Length, 0) // Bottom-left
			};

			// 3. Check if the movement path intersects any of the square edges
			for (var i = 0; i < squarePoints.Length; i++)
			{
				var p1 = squarePoints[i];
				var p2 = squarePoints[(i + 1) % squarePoints.Length];

				// Use the existing line collision check
				if (CheckOverlap(startPos, endPos, selfRadius, p1, new WDist(1)) ||
					DoesCircleAlongLineCollideWithCircle(startPos, endPos, selfRadius, p1, new WDist(1)))
					return true;

				// Check if movement path intersects square edge
				if (WPos.DoTwoLinesIntersect(startPos, endPos, p1, p2))
					return true;
			}

			return false;
		}

		static List<WPos?> CircleCircleIntersections(Fix64 x1, Fix64 y1, Fix64 r1, Fix64 x2, Fix64 y2, Fix64 r2)
		{
			var intersections = new List<WPos?>();
			var dx = x2 - x1;
			var dy = y2 - y1;
			var d = Fix64.Sqrt(dx * dx + dy * dy);

			if (d > r1 + r2 || d < Fix64.Abs(r1 - r2))
			{
				intersections.Add(null);
				intersections.Add(null);
				return intersections;
			}

			var a = (r1 * r1 - r2 * r2 + d * d) / ((Fix64)2 * d);
			var h = Fix64.Sqrt(r1 * r1 - a * a);
			var xm = x1 + a * dx / d;
			var ym = y1 + a * dy / d;
			var xs1 = xm + h * dy / d;
			var xs2 = xm - h * dy / d;
			var ys1 = ym - h * dx / d;
			var ys2 = ym + h * dx / d;

			intersections.Add(new WPos((int)xs1, (int)ys1, 0));
			intersections.Add(new WPos((int)xs2, (int)ys2, 0));

			return intersections;
		}

		bool IHitShape.LineIsColliding(WPos circleCenter, WPos p1, WPos p2) => LineIsCollidingLogic(circleCenter, p1, p2);

#pragma warning disable SA1312
		static List<WPos> IntersectingPosesFromLine(WPos circleCenter, int radius, WPos p1, WPos p2)
		{
			var poses = new List<WPos>();
			var p1InCircle = PosIsInsideCircle(circleCenter, radius, p1);
			var p2InCircle = PosIsInsideCircle(circleCenter, radius, p2);
			var p1X = (Fix64)p1.X;
			var p1Y = (Fix64)p1.Y;
			var p2X = (Fix64)p2.X;
			var p2Y = (Fix64)p2.Y;
			if (!(p1InCircle && p2InCircle))
			{
				if (p1X == p2X) // since we cannot have a slope for a vertical line, we add 1, or 0.1% the width of a cell to the ray cast
					p2X += 1;
				if (p1Y == p2Y)
					p2Y += 1;
				var a1 = (p2Y - p1Y) / (p2X - p1X); // could be an issue for floating point
				var b1 = (p2X * p1Y - p1X * p2Y) / (p2X - p1X);
				var a2 = (Fix64)(-1) / a1;
				var b2 = (Fix64)circleCenter.Y + (Fix64)circleCenter.X / a1;
				var Px = (b2 - b1) / (a1 - a2); // could be an issue for floating point
				var Py = (a1 * b2 - b1 * a2) / (a1 - a2);
				var LenCP = (new WPos((int)Px, (int)Py, 0) - circleCenter).Length;
				if (LenCP <= radius) // true if there is an intersection between the circle and the infinite line
				{
					var LenIPsq = Fix64.Abs(Sq((Fix64)radius) - Sq((Fix64)LenCP));
					// var A = Sq(a1) + 1;
					// var B = 2 * (a1 * (b1 - (int)Py) - 1);
					// var C = Sq((int)Px) + Sq(b1 - (int)Py) - LenIPsq;
					var A = Sq(a1) + 1;
					var B = (Fix64)(-2) * Px + (Fix64)2 * a1 * b1 - (Fix64)2 * a1 * Py;
					var C = Sq((double)Px) - 2 * (double)b1 * (double)Py + Sq((double)b1) + Sq((double)Py) - (double)LenIPsq;
					var discr = Sq((double)B) - 4 * (double)A * (double)C; // discriminant
					if (discr > 0) // No roots found if this is less than 0
					{
						var sqrtDiscr = (Fix64)(double)Sqrt(discr);
						var root1 = (-B + sqrtDiscr) / ((Fix64)2 * A);
						var root2 = (-B - sqrtDiscr) / ((Fix64)2 * A);
						var I1 = new WPos((int)root1, (int)(a1 * root1 + b1), 0);
						var I2 = new WPos((int)root2, (int)(a1 * root2 + b1), 0);
						var newP2 = new WPos((int)p2X, (int)p2Y, p2.Z);
						var I1isOnLine = PointIsWithinLineSegment(I1, p1, newP2);
						var I2isOnLine = PointIsWithinLineSegment(I2, p1, newP2);
						// Check that line segment is long enough to intersect, and that it is the nearest point
						if (I1isOnLine)
						{
							if (I2isOnLine && !p1InCircle && !p2InCircle) // this guarantees both points are valid
								if ((p1 - I1).Length <= (p1 - I2).Length)
								{
									poses.Add(I1);
									poses.Add(I2);
								}
								else // I2 is closer, so add it first
								{
									poses.Add(I2);
									poses.Add(I1);
								}
							else // I2 is not valid, so we only add I1
								poses.Add(I1);
						}
						else if (I2isOnLine) // I1 is not valid, so we only add I2
							poses.Add(I2);
					}
				}
			}
			return poses;
		}
#pragma warning restore SA1312 // Variable names should begin with lower-case letter

		bool IHitShape.IntersectsWithHitShape(int2 selfCenter, int2 secondCenter, HitShape hitShape)
		{
			if (hitShape.Info.Type is RectangleShape rect)
				return IntersectsWithHitShape(selfCenter, secondCenter, rect);
			else if (hitShape.Info.Type is CircleShape circ)
				return IntersectsWithHitShape(selfCenter, secondCenter, circ);
			else if (hitShape.Info.Type is PolygonShape poly)
				return IntersectsWithHitShape(selfCenter, secondCenter, poly);
			else if (hitShape.Info.Type is CapsuleShape caps)
				return IntersectsWithHitShape(selfCenter, secondCenter, caps);
			else
				return false;
		}

		bool IntersectsWithHitShape(int2 selfCenter, int2 rectCenter, RectangleShape rectHitShape) { return false; } // to be implemented
		bool IntersectsWithHitShape(int2 selfCenter, int2 circCenter, CircleShape circHitShape)
		{
			var circ1Radius = Radius.Length;
			var circ2Radius = circHitShape.Radius.Length;

			return Math.Abs((selfCenter.X - circCenter.X) * (selfCenter.X - circCenter.X) +
							(selfCenter.Y - circCenter.Y) * (selfCenter.Y - circCenter.Y)) <
					(circ1Radius + circ2Radius) * (circ1Radius + circ2Radius);
		}

		bool IntersectsWithHitShape(int2 selfCenter, int2 circleCenter, PolygonShape polygonHitShape) { return false; } // to be implemented
		bool IntersectsWithHitShape(int2 selfCenter, int2 circleCenter, CapsuleShape capsuleHitShape) { return false; } // to be implemented

		IEnumerable<IRenderable> IHitShape.RenderDebugOverlay(HitShape hs, WorldRenderer wr, WPos origin, WRot orientation)
		{
			var shapeColor = hs.IsTraitDisabled ? Color.LightGray : Color.Yellow;

			var corners = hs.Info.Type.GetCorners(origin.XYToInt2());
			foreach (var corner in corners)
			{
				var cornerWPos = new WPos(corner.X, corner.Y, origin.Z);
				yield return new CircleAnnotationRenderable(cornerWPos, new WDist(32), 2, shapeColor);
			}

			yield return new CircleAnnotationRenderable(origin + new WVec(0, 0, VerticalTopOffset), Radius, 1, shapeColor);
			yield return new CircleAnnotationRenderable(origin + new WVec(0, 0, VerticalBottomOffset), Radius, 1, shapeColor);
		}
	}
}

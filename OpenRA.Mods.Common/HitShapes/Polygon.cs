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
using System.Linq;
using OpenRA.Graphics;
using OpenRA.Mods.Common.Graphics;
using OpenRA.Mods.Common.Traits;
using OpenRA.Primitives;

namespace OpenRA.Mods.Common.HitShapes
{
	public class PolygonShape : IHitShape
	{
		public WDist OuterRadius { get; private set; }

		[FieldLoader.Require]
		public readonly int2[] Points;

		[Desc("Defines the top offset relative to the actor's center.")]
		public readonly int VerticalTopOffset = 0;

		[Desc("Defines the bottom offset relative to the actor's center.")]
		public readonly int VerticalBottomOffset = 0;

		[Desc("Rotates shape by an angle relative to actor facing. Mostly required for buildings on isometric terrain.",
			"Mobile actors do NOT need this!")]
		public readonly WAngle LocalYaw = WAngle.Zero;

		WVec[] combatOverlayVertsTop;
		WVec[] combatOverlayVertsBottom;
		int[] squares;

		public PolygonShape() { }

		public PolygonShape(int2[] points) { Points = points; }

		public void Initialize()
		{
			if (VerticalTopOffset < VerticalBottomOffset)
				throw new YamlException("VerticalTopOffset must be equal to or higher than VerticalBottomOffset.");

			OuterRadius = new WDist(Points.Max(x => x.Length));
			combatOverlayVertsTop = Points.Select(p => new WVec(p.X, p.Y, VerticalTopOffset)).ToArray();
			combatOverlayVertsBottom = Points.Select(p => new WVec(p.X, p.Y, VerticalBottomOffset)).ToArray();
			squares = new int[Points.Length];
			squares[0] = (Points[0] - Points[^1]).LengthSquared;
			for (var i = 1; i < Points.Length; i++)
				squares[i] = (Points[i] - Points[i - 1]).LengthSquared;
		}

		static int DistanceSquaredFromLineSegment(int2 c, int2 a, int2 b, int ab2)
		{
			var ac = c - a;
			var ac2 = ac.LengthSquared;
			var bc2 = (c - b).LengthSquared;

			// c is closest to point a
			if (ac2 + ab2 <= bc2)
				return ac2;

			// c is closest to point b
			if (bc2 + ab2 <= ac2)
				return bc2;

			// c is closest to its unknown orthogonal projection (p) onto the line spanned by b with a as the origin
			// Cast to a long for the calculations to avoid overflows
			var ab = b - a;
			var ap2 = ac.X * ab.X + ac.Y * ab.Y;
			var ap = new int2((int)((long)ab.X * ap2 / ab2), (int)((long)ab.Y * ap2 / ab2));

			// Length of vector pc squared.
			return (ac - ap).LengthSquared;
		}

		public WDist DistanceFromEdge(in WVec v)
		{
			var p = new int2(v.X, v.Y);
			var z = Math.Abs(v.Z);
			if (Points.PolygonContains(p))
				return new WDist(z);

			var min2 = DistanceSquaredFromLineSegment(p, Points[^1], Points[0], squares[0]);
			for (var i = 1; i < Points.Length; i++)
			{
				var d2 = DistanceSquaredFromLineSegment(p, Points[i - 1], Points[i], squares[i]);
				if (d2 < min2)
					min2 = d2;
			}

			return new WDist(Exts.ISqrt(min2 + z * z));
		}

		public WDist DistanceFromEdge(WPos pos, WPos origin, WRot orientation)
		{
			orientation += WRot.FromYaw(LocalYaw);

			if (pos.Z > origin.Z + VerticalTopOffset)
				return DistanceFromEdge((pos - (origin + new WVec(0, 0, VerticalTopOffset))).Rotate(-orientation));

			if (pos.Z < origin.Z + VerticalBottomOffset)
				return DistanceFromEdge((pos - (origin + new WVec(0, 0, VerticalBottomOffset))).Rotate(-orientation));

			return DistanceFromEdge((pos - new WPos(origin.X, origin.Y, pos.Z)).Rotate(-orientation));
		}

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

		WPos[] IHitShape.GetCorners(int2 selfCenter) { return new WPos[0]; }
		bool IntersectsWithHitShape(int2 selfCenter, int2 rectCenter, RectangleShape rectHitShape) { return false; } // to be implemented
		bool IntersectsWithHitShape(int2 selfCenter, int2 circleCenter, CircleShape circleHitShape) { return false; } // to be implemented
		bool IntersectsWithHitShape(int2 selfCenter, int2 polygonCenter, PolygonShape polygonHitShape) { return false; } // to be implemented
		bool IntersectsWithHitShape(int2 selfCenter, int2 capsuleCenter, CapsuleShape capsuleHitShape) { return false; } // to be implemented

		IEnumerable<IRenderable> IHitShape.RenderDebugOverlay(HitShape hs, WorldRenderer wr, WPos actorPos, WRot orientation)
		{
			orientation += WRot.FromYaw(LocalYaw);
			var vertsTop = combatOverlayVertsTop.Select(v => actorPos + v.Rotate(orientation)).ToArray();
			var vertsBottom = combatOverlayVertsBottom.Select(v => actorPos + v.Rotate(orientation)).ToArray();

			var shapeColor = hs.IsTraitDisabled ? Color.LightGray : Color.Yellow;

			yield return new PolygonAnnotationRenderable(vertsTop, actorPos, 1, shapeColor);
			yield return new PolygonAnnotationRenderable(vertsBottom, actorPos, 1, shapeColor);
			yield return new CircleAnnotationRenderable(wr.World, actorPos, OuterRadius, 1, hs.IsTraitDisabled ? Color.Gray : Color.LimeGreen);
		}

		public bool LineIntersectsOrIsInside(WPos selfCenter, WPos p1, WPos p2)
		{
			throw new NotImplementedException();
		}

		public bool LineIsColliding(WPos selfCEnter, WPos p1, WPos p2)
		{
			throw new NotImplementedException();
		}
	}
}

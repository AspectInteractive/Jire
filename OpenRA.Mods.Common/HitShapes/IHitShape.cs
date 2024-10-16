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

using System.Collections.Generic;
using OpenRA.Graphics;
using OpenRA.Mods.Common.Traits;

namespace OpenRA.Mods.Common.HitShapes
{
	public interface IHitShape
	{
		WDist OuterRadius { get; }

		WDist DistanceFromEdge(in WVec v);
		WDist DistanceFromEdge(WPos pos, WPos origin, WRot orientation);
		WPos[] GetCorners(int2 selfCenter);
		bool IntersectsWithHitShape(int2 selfCenter, int2 secondCenter, HitShape hitShape);

		void Initialize();
		IEnumerable<IRenderable> RenderDebugOverlay(HitShape hs, WorldRenderer wr, WPos origin, WRot orientation);
		bool LineIntersectsOrIsInside(WPos selfCenter, WPos p1, WPos p2);
		bool LineIsColliding(WPos selfCEnter, WPos p1, WPos p2);
	}
}

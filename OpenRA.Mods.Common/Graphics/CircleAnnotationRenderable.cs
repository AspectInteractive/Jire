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

using OpenRA.Graphics;
using OpenRA.Primitives;

namespace OpenRA.Mods.Common.Graphics
{
	public class CircleAnnotationRenderable : IRenderable, IFinalizedRenderable
	{
		World world;
		const int CircleSegments = 32;
		static readonly WVec[] FacingOffsets = Exts.MakeArray(CircleSegments, i => new WVec(1024, 0, 0).Rotate(WRot.FromFacing(i * 256 / CircleSegments)));
		readonly WDist radius;
		readonly int width;
		readonly Color color;
		readonly bool filled;

		public CircleAnnotationRenderable(World world, WPos centerPosition, WDist radius, int width, Color color, bool filled = false)
		{
			this.world = world;
			Pos = centerPosition;
			this.radius = radius;
			this.width = width;
			this.color = color;
			this.filled = filled;
		}

		public void AddOrUpdateScreenMap()
		{
			var length = (radius * 2).Length;
			world.ScreenMap.AddOrUpdate(this, Pos, new Size(length, length));
		}

		public void RemoveFromScreenMap() => world.ScreenMap.Remove(this);

		public WPos Pos { get; }
		public int ZOffset => 0;
		public bool IsDecoration => true;

		public IRenderable WithZOffset(int newOffset) { return new CircleAnnotationRenderable(world, Pos, radius, width, color, filled); }
		public IRenderable OffsetBy(in WVec vec) { return new CircleAnnotationRenderable(world, Pos + vec, radius, width, color, filled); }
		public IRenderable AsDecoration() { return this; }

		public IFinalizedRenderable PrepareRender(WorldRenderer wr) { return this; }
		public void Render(WorldRenderer wr)
		{
			var cr = Game.Renderer.RgbaColorRenderer;
			if (filled)
			{
				var offset = new WVec(radius.Length, radius.Length, 0);
				var tl = wr.Viewport.WorldToViewPx(wr.ScreenPosition(Pos - offset));
				var br = wr.Viewport.WorldToViewPx(wr.ScreenPosition(Pos + offset));

				cr.FillEllipse(tl, br, color);
			}
			else
			{
				var r = radius.Length;
				var a = wr.Viewport.WorldToViewPx(wr.ScreenPosition(Pos + r * FacingOffsets[CircleSegments - 1] / 1024));
				for (var i = 0; i < CircleSegments; i++)
				{
					var b = wr.Viewport.WorldToViewPx(wr.ScreenPosition(Pos + r * FacingOffsets[i] / 1024));
					cr.DrawLine(a, b, width, color);
					a = b;
				}
			}
		}

		public void RenderDebugGeometry(WorldRenderer wr) { }
		public Rectangle ScreenBounds(WorldRenderer wr) { return Rectangle.Empty; }
		public void Dispose() => RemoveFromScreenMap();
	}
}

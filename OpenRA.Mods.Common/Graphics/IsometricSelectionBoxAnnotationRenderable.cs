#region Copyright & License Information
/*
 * Copyright 2007-2022 The OpenRA Developers (see AUTHORS)
 * This file is part of OpenRA, which is free software. It is made
 * available to you under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. For more
 * information, see COPYING.
 */
#endregion

using System.Linq;
using OpenRA.Graphics;
using OpenRA.Primitives;

namespace OpenRA.Mods.Common.Graphics
{
	public class IsometricSelectionBoxAnnotationRenderable : IRenderable, IFinalizedRenderable
	{
		static readonly float2 TLOffset = new float2(-12, -6);
		static readonly float2 TROffset = new float2(12, -6);
		static readonly float2 TOffset = new float2(0, -13);
		static readonly float2[] Offsets =
		{
			-TROffset, -TLOffset, -TOffset,
			TROffset, -TOffset, -TLOffset,
			-TLOffset, TOffset, TROffset,
			TLOffset, TROffset, TOffset,
			-TROffset, TOffset, TLOffset,
			TLOffset, -TOffset, -TROffset
		};

		readonly WPos pos;
		readonly Polygon bounds;
		readonly Color color;
		readonly int layer;

		public IsometricSelectionBoxAnnotationRenderable(Actor actor, in Polygon bounds, Color color, int layer = 0)
		{
			pos = actor.CenterPosition;
			this.bounds = bounds;
			this.color = color;
			this.layer = layer;
		}

		public IsometricSelectionBoxAnnotationRenderable(WPos pos, in Polygon bounds, Color color, int layer = 0)
		{
			this.pos = pos;
			this.bounds = bounds;
			this.color = color;
			this.layer = layer;
		}

		public WPos Pos => pos;

		public int ZOffset => 0;
		public int Layer => layer;
		public bool IsDecoration => true;

		public IRenderable WithZOffset(int newOffset) { return this; }
		public IRenderable OffsetBy(in WVec vec) { return new IsometricSelectionBoxAnnotationRenderable(pos + vec, bounds, color, layer); }
		public IRenderable AsDecoration() { return this; }

		public IFinalizedRenderable PrepareRender(WorldRenderer wr) { return this; }

		public void Render(WorldRenderer wr)
		{
			var screen = bounds.Vertices.Select(v => wr.Viewport.WorldToViewPx(v).ToFloat2()).ToArray();

			var tl = new float2(-12, -6);
			var tr = new float2(12, -6);
			var t = new float2(0, -13);

			var cr = Game.Renderer.RgbaColorRenderer;
			for (var i = 0; i < 6; i++)
			{
				cr.DrawLine(new float3[] { screen[i] + Offsets[3 * i], screen[i], screen[i] + Offsets[3 * i + 1] }, 1, color, true);
				cr.DrawLine(new float3[] { screen[i], screen[i] + Offsets[3 * i + 2] }, 1, color, true);
			}
		}

		public void RenderDebugGeometry(WorldRenderer wr) { }
		public Rectangle ScreenBounds(WorldRenderer wr) { return Rectangle.Empty; }
	}
}

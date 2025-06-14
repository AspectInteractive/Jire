#region Copyright & License Information
/*
 * Copyright 2007-2021 The OpenRA Developers (see AUTHORS)
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
using OpenRA.Mods.Common.Commands;
using OpenRA.Mods.Common.Graphics;
using OpenRA.Primitives;
using OpenRA.Traits;

namespace OpenRA.Mods.Common.Traits
{
	[TraitLocation(SystemActors.World | SystemActors.EditorWorld)]
	[Desc("Renders a debug overlay of the Nav Mesh Pathfinder. Attach this to the world actor.")]
	public class NavMeshPathfinderOverlayInfo : TraitInfo<NavMeshPathfinderOverlay> { }

	public class NavMeshPathfinderOverlay : IRenderAnnotations, IWorldLoaded, IChatCommand
	{
		World world;
		public readonly List<Command> Comms;
		readonly List<(List<WPos>, Color C, string Key)> linesWithColors = new();
		readonly List<((WPos Pos, WDist Dist), Color C, string Key)> circlesWithColors = new();
		readonly List<(WPos Pos, Color C, string Key)> pointsWithColors = new();
		readonly List<string> enabledOverlays = new();
		bool NoFiltering => enabledOverlays.Count == 0;
		readonly List<(Actor Unit, List<WPos> Path, Color? C)> paths = new();
		readonly List<(List<WPos> Triangle, Color? C)> triangles = new();
		readonly List<(List<WPos> Line, string Key)> lines = new();

		public struct OverlayKeyStrings
		{
			public const string Path = "path";  // toggles the theta path
			public const string Triangles = "triangles"; // toggles the triangulation grid
			public const string HeatMap = "heatmap"; // toggles the theta path heat map
			public const string Circles = "circles"; // toggles the circles and slices in the theta PF execution
			public const string Test = "test"; // toggles the circles and slices in the theta PF execution manager
		}

		// Set this to true to display annotations showing the cost at each cell
		private readonly bool showCosts = true;

		public bool Enabled;
		private float currHue = Color.Blue.ToAhsv().H; // 0.0 - 1.0
		private float pointHue = Color.Red.ToAhsv().H; // 0.0 - 1.0
		private float circleHue = Color.LightGreen.ToAhsv().H; // 0.0 - 1.0
		private float pathHue = Color.Yellow.ToAhsv().H; // 0.0 - 1.0
		private float lineHue = Color.LightBlue.ToAhsv().H; // 0.0 - 1.0
		private float currSat = 1.0F; // 0.0 - 1.0
		private float currLight = 0.7F; // 0.0 - 1.0 with 1.0 being brightest
		private float lineColorIncrement = 0.05F;
		public Action<string> ToggleVisibility;

		readonly List<string> validChatCommandArgs =
			new()
			{
				OverlayKeyStrings.Path,
				OverlayKeyStrings.Triangles,
				OverlayKeyStrings.HeatMap,
				OverlayKeyStrings.Circles,
				OverlayKeyStrings.Test,
			};

		public NavMeshPathfinderOverlay()
		{
			Comms = new List<Command>()
			{
				new("navmesh", "toggles the nav mesh pathfinder overlay.", true),
				new("navmeshall", "toggles all nav mesh pathfinder overlays.", true)
			};
		}

		void IWorldLoaded.WorldLoaded(World w, WorldRenderer wr)
		{
			world = w;
			var console = w.WorldActor.TraitOrDefault<ChatCommands>();
			var help = w.WorldActor.TraitOrDefault<HelpCommand>();

			if (console == null || help == null)
				return;

			foreach (var comm in Comms)
			{
				console.RegisterCommand(comm.Name, this);
				if (comm.InHelp)
					help.RegisterHelp(comm.Name, comm.Desc);
			}

			ToggleVisibility = arg => DevCommands.Visibility(arg, w);
		}

		void IChatCommand.InvokeCommand(string name, string arg)
		{
			if (Comms.Any(comm => comm.Name == name))
			{
				// If an argument is passed, we always enable and then toggle the argument overlay
				if (validChatCommandArgs.Any(validArg => arg == validArg))
				{
					Enabled = true;
					// If the overlay does not exist and cannot be removed, then we add it
					if (!enabledOverlays.Remove(arg))
						enabledOverlays.Add(arg);
				}
				else // if no argument is passed, we toggle Theta
				{
					ToggleVisibility("");
					Enabled ^= true;
				}
			}
		}

		public static void GenericLinkedPointsFunc<T1>(List<T1> pointList, int pointListLen, Action<T1, T1> funcOnLinkedPoints)
		{
			for (var i = 0; i < pointListLen - 1; i++)
			{
				var currItem = pointList[i];
				var nextItem = pointList[i + 1];
				funcOnLinkedPoints(currItem, nextItem);
			}
		}
		public static void GenericLinkedPointsFunc<T1, T2>(List<T1> pointList, int pointListLen, Func<T1, T2> pointUnpacker,
														Action<T2, T2> funcOnLinkedPoints)
		{
			for (var i = 0; i < pointListLen - 1; i++)
			{
				var currItem = pointUnpacker(pointList[i]);
				var nextItem = pointUnpacker(pointList[i + 1]);
				funcOnLinkedPoints(currItem, nextItem);
			}
		}

		public static List<LineAnnotationRenderableWithZIndex> GetPathRenderableSet(List<WPos> path, int lineThickness, Color lineColor, int endPointRadius,
																	int endPointThickness, Color endPointColor)
		{
			var linesToRender = new List<LineAnnotationRenderableWithZIndex>();
			void FuncOnLinkedPoints(WPos wpos1, WPos wpos2) => linesToRender.Add(new LineAnnotationRenderableWithZIndex(wpos1, wpos2,
																			lineThickness, lineColor, lineColor,
																			(endPointRadius, endPointThickness, endPointColor)));
			GenericLinkedPointsFunc(path, path.Count, FuncOnLinkedPoints);
			return linesToRender;
		}

		IEnumerable<IRenderable> IRenderAnnotations.RenderAnnotations(Actor self, WorldRenderer wr)
		{
			if (!Enabled)
				yield break;

			var pointRadius = 100;
			var pointThickness = 3;
			var lineThickness = 3;
			var endPointRadius = 100;
			var endPointThickness = 3;
			var fontName = "TinyBold";
			var font = Game.Renderer.Fonts[fontName];
			Color lineColor;

			CircleAnnotationRenderable PointRenderFunc(WPos p, Color color)	=> new(p, new WDist(pointRadius), pointThickness, color, true);
			CircleAnnotationRenderable CircleRenderFunc((WPos, WDist) c, Color color) => new(c.Item1, c.Item2, pointThickness, color, false);

			// Render Points
			foreach (var (point, color, _) in pointsWithColors.Where(o => NoFiltering || enabledOverlays.Contains(o.Key)))
				yield return PointRenderFunc(point, color);

			// Render Circles
			foreach (var (circle, color, _) in circlesWithColors.Where(o => NoFiltering || enabledOverlays.Contains(o.Key)))
				yield return CircleRenderFunc(circle, color);

			// Render Paths
			lineColor = Color.FromAhsv(pathHue, currSat, currLight);
			if (NoFiltering || enabledOverlays.Contains(OverlayKeyStrings.Path))
			{
				foreach (var (unit, path, color) in paths)
				{
					if (self.World.Selection.Contains(unit)) // Do not render the path if the unit is not selected
					{
						var linesToRender = GetPathRenderableSet(path, lineThickness, color ?? lineColor, endPointRadius, endPointThickness, lineColor);
						foreach (var line in linesToRender)
							yield return line;
					}
				}
			}

			// Render Triangles
			lineColor = Color.FromAhsv(pathHue, currSat, currLight);
			if (NoFiltering || enabledOverlays.Contains(OverlayKeyStrings.Triangles))
			{
				foreach (var (triangle, color) in triangles)
				{
					var linesToRender = GetPathRenderableSet(triangle, lineThickness, color ?? lineColor, endPointRadius, endPointThickness, lineColor);
					foreach (var line in linesToRender)
						yield return line;
				}
			}

			// Render Lines
			lineColor = Color.FromAhsv(lineHue, currSat, currLight);
			foreach (var (line, _) in lines.Where(o => NoFiltering || enabledOverlays.Contains(o.Key)))
			{
				var linesToRender = GetPathRenderableSet(line, lineThickness, lineColor, endPointRadius, endPointThickness, lineColor);
				foreach (var l in linesToRender)
					yield return l;
			}

			// Render LinesWithColours
			foreach (var (line, color, _) in linesWithColors.Where(o => NoFiltering || enabledOverlays.Contains(o.Key)))
			{
				var linesToRender = GetPathRenderableSet(line, lineThickness, color, endPointRadius, endPointThickness, color);
				foreach (var l in linesToRender)
					yield return l;
			}
		}

		public void AddPath(Actor unit, List<WPos> path, Color? color = null) { paths.Add((unit, path, color)); }
		public void RemovePath(List<WPos> path)	{ paths.RemoveAll(p => p.Path == path); }
		public void AddTriangle(List<WPos> triangle, Color? color = null)
		{
			if (triangle.Count < 3)
				throw new ArgumentException("Triangle must have at least 3 points.");

			triangle.Add(triangle[0]); // Add the first point to the end so that the triangle loops around.

			triangles.Add((triangle, color));
		}

		public void RemoveTriangle(List<WPos> triangle) { triangles.RemoveAll(t => t.Triangle == triangle); }
		public void AddLine(List<WPos> line, string key) { lines.Add((line, key)); }
		public void RemoveLine(List<WPos> line) { lines.RemoveAll(l => l.Line == line); }
		public void AddLineWithColor(List<WPos> line, Color color, string key) { linesWithColors.Add((line, color, key)); }
		public void RemoveLineWithColor(List<WPos> line) { linesWithColors.RemoveAll(lwc => lwc.Item1 == line); }
		public void AddCircle((WPos Pos, WDist Dist) circle, string key) { circlesWithColors.Add((circle, Color.FromAhsv(circleHue, currSat, currLight), key)); }
		public void AddCircleWithColor((WPos Pos, WDist Dist) circle, Color color, string key) { circlesWithColors.Add((circle, color, key)); }
		public void RemoveCircle((WPos Pos, WDist Dist) circle) { circlesWithColors.RemoveAll(c => c.Item1 == circle); }

		public void ClearPaths() { paths.Clear(); }
		public void ClearTriangles() { triangles.Clear(); }
		public void ClearLines() { lines.Clear(); }
		public void ClearLinesWithColors() { linesWithColors.Clear(); }
		public void ClearPoints() { pointsWithColors.Clear(); }
		public void ClearCircles() { circlesWithColors.Clear(); }
		public void ClearRadiuses() { circlesWithColors.Clear(); }

		bool IRenderAnnotations.SpatiallyPartitionable => false;
	}
}

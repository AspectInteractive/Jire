using System;
using System.Collections.Generic;
using System.Linq;
using OpenRA.Mods.Common.Activities;
using OpenRA.Mods.Common.HitShapes;
using OpenRA.Mods.Common.Traits;
using OpenRA.Primitives;
using OpenRA.Traits;
using static OpenRA.Mods.Common.Traits.MobileOffGrid;

namespace OpenRA.Mods.Common.Pathfinder
{
	public abstract class BaseOffGridPathSearch : IDisposable
	{
		// Common properties
		public Actor Self { get; protected set; }
		public WPos Source { get; protected set; }
		public WPos Dest { get; protected set; }
		public int CurrDelayToRun { get; set; } = -1; // This is the number of ticks required to elapse before this PF runs
		public List<Actor> ActorsSharingPF { get; set; } = new();
		public bool Running { get; set; }
		public bool PathFound { get; set; } = false;

		protected static readonly List<PathPos> EmptyPath = new(0);
		public List<PathPos> path = new();

		public struct PathPos
		{
			public CCPos ccPos;
			public WPos wPos;

			public PathPos(WPos wPos, CCPos ccPos)
			{
				this.wPos = wPos;
				this.ccPos = ccPos;
			}
			public PathPos(WPos wPos)
			{
				this.wPos = wPos;
				ccPos = CCPos.Zero;
			}
		}

		// Common fields
		protected readonly World world;
		protected readonly Locomotor locomotor;
		protected readonly MobileOffGrid mobileOffGrid;
		protected bool disposed;

		// Abstract methods that implementations must provide
		public abstract void Expand(int maxExpansions);
		public abstract List<WPos> FindPath(WPos start, WPos goal);

		protected BaseOffGridPathSearch(Actor self, WPos source, WPos dest, int currDelayToRun = 2)
		{
			CurrDelayToRun = currDelayToRun;
			Self = self;
			Source = source;
			Dest = dest;
			world = self.World;
			mobileOffGrid = self.TraitsImplementing<MobileOffGrid>().FirstOrDefault(Exts.IsTraitEnabled);
			locomotor = self.World.WorldActor.TraitsImplementing<Locomotor>().FirstEnabledTraitOrDefault();
		}

		// Common utility methods
		public static bool IsCellBlocked(Actor self, Locomotor locomotor, CPos? cell, BlockedByActor check = BlockedByActor.None)
		{
			if (cell == null)
				return true;

			return CellIsBlockedCache(self, locomotor, (CPos)cell, check);
		}

		public bool IsPathObservable(WPos sourcePos, WPos destPos, IHitShape unitHitShape, bool useUnitRadius, int neighbours)
			=> IsPathObservable(world, Self, locomotor, sourcePos, destPos, unitHitShape, useUnitRadius, neighbours);

		public static bool IsPathObservable(World world, Actor self, Locomotor locomotor, WPos rootPos, WPos destPos,
			IHitShape unitHitShape, bool useUnitRadius, int neighbours)
		{
			if (useUnitRadius)
			{
				foreach (var (source, dest) in MobileOffGrid.GenSDPairs(rootPos, destPos - rootPos, unitHitShape))
				{
					var cellsUnderneathLine = GetAllCellsUnderneathALine(world, source, dest, neighbours);
					if (AreCellsIntersectingPath(world, self, locomotor, cellsUnderneathLine, rootPos, destPos))
						return false;
				}
			}
			else
			{
				var cellsUnderneathLine = GetAllCellsUnderneathALine(world, rootPos, destPos, neighbours);
				return !AreCellsIntersectingPath(world, self, locomotor, cellsUnderneathLine, rootPos, destPos);
			}

			return true;
		}

		public static List<CPos> GetAllCellsUnderneathALine(World world, WPos a0, WPos a1, int neighboursToCount = 0)
		{
			var ca0 = world.Map.CellContaining(a0);
			var ca1 = world.Map.CellContaining(a1);
			var x0 = ca0.X;
			var y0 = ca0.Y;
			var x1 = ca1.X;
			var y1 = ca1.Y;
			var dx = Math.Abs(x1 - x0);
			var dy = -Math.Abs(y1 - y0);
			var sx = x0 < x1 ? 1 : -1;
			var sy = y0 < y1 ? 1 : -1;
			var err = dx + dy;

			var result = neighboursToCount > 0 ? null : new List<CPos>(Math.Max(dx, -dy) + 1);
			var resultSet = neighboursToCount > 0 ? new HashSet<CPos>() : null;

			int minX = 0, minY = 0, maxX = world.Map.MapSize.X - 1, maxY = world.Map.MapSize.Y - 1;

			void AddCell(int x, int y)
			{
				if (x < minX || x > maxX || y < minY || y > maxY)
					return;

				var cp = new CPos(x, y);
				if (neighboursToCount > 0)
				{
					for (var nx = Math.Max(x - neighboursToCount, minX); nx <= Math.Min(x + neighboursToCount, maxX); nx++)
						for (var ny = Math.Max(y - neighboursToCount, minY); ny <= Math.Min(y + neighboursToCount, maxY); ny++)
							resultSet.Add(new CPos(nx, ny));
				}
				else
				{
					result.Add(cp);
				}
			}

			while (true)
			{
				AddCell(x0, y0);

				if (x0 == x1 && y0 == y1)
					break;

				var e2 = 2 * err;
				if (e2 >= dy)
				{
					err += dy;
					x0 += sx;
				}
				if (e2 <= dx)
				{
					err += dx;
					y0 += sy;
				}
			}

			if (neighboursToCount > 0)
				return resultSet.ToList();
			else
				return result;
		}

		public static bool AreCellsIntersectingPath(World world, Actor self, Locomotor locomotor,
			List<CPos> cells, WPos sourcePos, WPos destPos)
		{
			foreach (var cell in cells)
			{
				if (IsCellBlocked(self, locomotor, cell) && world.Map.AnyCellEdgeIntersectsWithLine(cell, sourcePos, destPos))
				{
					return true;
				}
			}
			return false;
		}

		public static CCPos GetNearestUnblockedCCPos(World world, Actor self, Locomotor locomotor, WPos pos, int maxExpansions = 10, bool showDebug = false)
		{
			var overlay = self.World.WorldActor.TraitsImplementing<ThetaStarPathfinderOverlay>().FirstEnabledTraitOrDefault();
			bool CCIsUnblocked(CCPos cc) => !CCIsBlocked(world, self, locomotor, cc);
			var i = 0;

			var candidateCCs = GetAllNeighbourCCPosByDist(world, pos).ToList();

			if (showDebug && overlay != null)
				foreach (var cc in candidateCCs)
					overlay.AddCircleWithColor((world.Map.WPosFromCCPos(cc), new WDist(512)), Color.RandomColor(), ThetaStarPathfinderOverlay.OverlayKeyStrings.Test);

			while (candidateCCs.Count > 0 && i < maxExpansions)
			{
				var unblockedCandidates = candidateCCs.Where(c => CCIsUnblocked(c));
				if (unblockedCandidates.Any())
					return unblockedCandidates.First();

				var newCandidateCCs = new List<CCPos>();
				foreach (var c in candidateCCs)
					newCandidateCCs.AddRange(GetCCNeighbours(c).Where(c => CcinMap(c, world)));

				if (showDebug && overlay != null)
					foreach (var cc in newCandidateCCs)
						overlay.AddCircleWithColor((world.Map.WPosFromCCPos(cc), new WDist(512)), Color.RandomColor(), ThetaStarPathfinderOverlay.OverlayKeyStrings.Test);

				candidateCCs = newCandidateCCs;
				i++;
			}

			return ClosestCCPosInMap(new CCPos(-1, -1), world);
		}

		public static IEnumerable<CCPos> GetAllNeighbourCCPosByDist(World world, WPos pos)
		{
			var cellContainingPos = world.Map.CellContaining(pos);
			return new List<(CCPos CC, int Dist)>
			{
				(Map.TopLeftCCPos(cellContainingPos), (pos - world.Map.TopLeftOfCell(cellContainingPos)).HorizontalLength),
				(Map.TopRightCCPos(cellContainingPos), (pos - world.Map.TopRightOfCell(cellContainingPos)).HorizontalLength),
				(Map.BottomLeftCCPos(cellContainingPos), (pos - world.Map.BottomLeftOfCell(cellContainingPos)).HorizontalLength),
				(Map.BottomRightCCPos(cellContainingPos), (pos - world.Map.BottomRightOfCell(cellContainingPos)).HorizontalLength),
			}.Where(x => CcinMap(x.CC, world))
			.OrderBy(x => x.Dist)
			.Select(x => x.CC);
		}

		static List<CCPos> GetCCNeighbours(CCPos cc)
		{
			return new List<CCPos>()
			{
				new(cc.X, cc.Y - 1, cc.Layer),
				new(cc.X - 1, cc.Y - 1, cc.Layer),
				new(cc.X + 1, cc.Y - 1, cc.Layer),
				new(cc.X, cc.Y + 1, cc.Layer),
				new(cc.X - 1, cc.Y + 1, cc.Layer),
				new(cc.X + 1, cc.Y + 1, cc.Layer),
				new(cc.X - 1, cc.Y, cc.Layer),
				new(cc.X + 1, cc.Y, cc.Layer),
			};
		}

		public static bool CcinMap(CCPos ccPos, World world)
		{
			return ccPos.X >= 0 && ccPos.X <= world.Map.MapSize.X
				&& ccPos.Y >= 0 && ccPos.Y <= world.Map.MapSize.Y;
		}

		static CCPos ClosestCCPosInMap(CCPos ccPos, World world)
		{
			return new CCPos(Math.Max(Math.Min(ccPos.X, world.Map.MapSize.X), 0),
							 Math.Max(Math.Min(ccPos.Y, world.Map.MapSize.Y), 0));
		}

		static bool CCIsBlocked(World world, Actor self, Locomotor locomotor, CCPos cc, BlockedByActor check = BlockedByActor.Immovable,
			CPos? includeDest = null)
		{
			var TL = world.Map.CellTopLeftOfCCPos(cc);
			var TR = world.Map.CellTopRightOfCCPos(cc);
			var BL = world.Map.CellBottomLeftOfCCPos(cc);
			var BR = world.Map.CellBottomRightOfCCPos(cc);

			var TLBlocked = IsCellBlocked(self, locomotor, TL, check);
			var TRBlocked = IsCellBlocked(self, locomotor, TR, check);
			var BLBlocked = IsCellBlocked(self, locomotor, BL, check);
			var BRBlocked = IsCellBlocked(self, locomotor, BR, check);

			return (TLBlocked && BRBlocked && !TRBlocked && !BLBlocked) ||
				   (TRBlocked && BLBlocked && !TLBlocked && !BRBlocked) ||
				   (TLBlocked && TRBlocked && BLBlocked) ||
				   (TLBlocked && TRBlocked && BRBlocked) ||
				   (BLBlocked && BRBlocked && TLBlocked) ||
				   (BLBlocked && BRBlocked && TRBlocked) ||
				   (includeDest != null &&
					(includeDest == TL || includeDest == TR || includeDest == BL || includeDest == BR));
		}

		// Common rendering method
		public void RenderPathIfOverlay(List<WPos> path)
		{
			var overlay = world.WorldActor.TraitsImplementing<ThetaStarPathfinderOverlay>().FirstEnabledTraitOrDefault();
			if (overlay?.Enabled == true && path.Count > 1)
			{
				overlay.AddPath(Self, path);
			}
		}

		protected enum CellSurroundingCorner : byte { TopLeft, TopRight, BottomLeft, BottomRight }

		protected bool CellSurroundingCCPosIsBlocked(CCPos ccPos, CellSurroundingCorner cellSurroundingCorner, BlockedByActor check = BlockedByActor.Immovable)
			=> CellSurroundingCCPosIsBlocked(Self.World, Self, locomotor, ccPos, cellSurroundingCorner, check);
		protected static bool CellSurroundingCCPosIsBlocked(World world, Actor self, Locomotor locomotor,
														  CCPos ccPos, CellSurroundingCorner cellSurroundingCorner, BlockedByActor check = BlockedByActor.Immovable)
		{
			switch (cellSurroundingCorner)
			{
				case CellSurroundingCorner.TopLeft:
					return IsCellBlocked(self, locomotor, world.Map.CellTopLeftOfCCPos(ccPos), check);
				case CellSurroundingCorner.TopRight:
					return IsCellBlocked(self, locomotor, world.Map.CellTopRightOfCCPos(ccPos), check);
				case CellSurroundingCorner.BottomLeft:
					return IsCellBlocked(self, locomotor, world.Map.CellBottomLeftOfCCPos(ccPos), check);
				case CellSurroundingCorner.BottomRight:
					return IsCellBlocked(self, locomotor, world.Map.CellBottomRightOfCCPos(ccPos), check);
				default:
					return false;
			}
		}

		// This will pad the ccPos in a path with a set amount of padding based on the actor's radius
		// Four cases: Note that only one direction is shown below, but other directions are simply mirrors
		//
		// XO                    OX                   XO                    XO
		// XX - move point TR    OO - move point BL   XO - move point R     OX - should not happen (to be tested)

		public WPos PadCC(CCPos cc) { return PadCC(Self.World, Self, locomotor, mobileOffGrid, cc); }
		public static WPos PadCC(World world, Actor self, Locomotor locomotor, MobileOffGrid mobileOG, CCPos cc)
		{
			var ccPos = world.Map.WPosFromCCPos(cc);
			var unitRadius = mobileOG.UnitRadius.Length;

			var topLeftBlocked = CellSurroundingCCPosIsBlocked(world, self, locomotor, cc, CellSurroundingCorner.TopLeft);
			var topRightBlocked = CellSurroundingCCPosIsBlocked(world, self, locomotor, cc, CellSurroundingCorner.TopRight);
			var botLeftBlocked = CellSurroundingCCPosIsBlocked(world, self, locomotor, cc, CellSurroundingCorner.BottomLeft);
			var botRightBlocked = CellSurroundingCCPosIsBlocked(world, self, locomotor, cc, CellSurroundingCorner.BottomRight);

			var blockedList = new List<bool>() { topLeftBlocked, topRightBlocked, botLeftBlocked, botRightBlocked };
			var areBlocked = blockedList.Where(c => c == true).ToList();

			if (areBlocked.Count == 1 || areBlocked.Count == 3)
			{
				if (topLeftBlocked || (topLeftBlocked && botLeftBlocked && topRightBlocked))
					return new WPos(ccPos.X + unitRadius, ccPos.Y + unitRadius, ccPos.Z);
				if (topRightBlocked || (topRightBlocked && topLeftBlocked && botRightBlocked))
					return new WPos(ccPos.X - unitRadius, ccPos.Y + unitRadius, ccPos.Z);
				if (botLeftBlocked || (botLeftBlocked && topLeftBlocked && botRightBlocked))
					return new WPos(ccPos.X + unitRadius, ccPos.Y - unitRadius, ccPos.Z);
				if (botRightBlocked || (botRightBlocked && botLeftBlocked && topRightBlocked))
					return new WPos(ccPos.X - unitRadius, ccPos.Y - unitRadius, ccPos.Z);
			}
			else if (areBlocked.Count == 2)
			{
				if (topLeftBlocked && topRightBlocked)
					return new WPos(ccPos.X, ccPos.Y + unitRadius, ccPos.Z);
				if (topLeftBlocked && botLeftBlocked)
					return new WPos(ccPos.X + unitRadius, ccPos.Y, ccPos.Z);
				if (botLeftBlocked && botRightBlocked)
					return new WPos(ccPos.X, ccPos.Y - unitRadius, ccPos.Z);
				if (topRightBlocked && botRightBlocked)
					return new WPos(ccPos.X - unitRadius, ccPos.Y, ccPos.Z);
				return ccPos; // This is the case where the 2 blocked cells are diagonally adjacent, and should never happen.
			}

			return ccPos;
		}

		public virtual void Dispose()
		{
			if (!disposed)
			{
				disposed = true;
			}
		}
	}
}

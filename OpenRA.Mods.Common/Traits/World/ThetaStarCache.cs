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
using OpenRA.Graphics;
using OpenRA.Traits;
using static OpenRA.Mods.Common.Pathfinder.ThetaStarPathSearch;

namespace OpenRA.Mods.Common.Traits
{
	[TraitLocation(SystemActors.World)]
	[Desc("Stores a cache of paths from previously run Theta Star pathfinders, for use by future run pathfinders",
		  "Attach this to the world actor.")]
	public class ThetaStarCacheInfo : TraitInfo<ThetaStarCache> { }

	public class ThetaStarCache : IWorldLoaded
	{
		public class CCStateWithGoal
		{
			public CCState ccState;
			public WPos goalPos;
			public int finalGval;
			public int FinalHval => finalGval - ccState.Gval;

			public CCStateWithGoal(CCState ccState, WPos goalPos, int finalGval)
			{
				this.ccState = ccState;
				this.goalPos = goalPos;
				this.finalGval = finalGval;
			}
		}

		World world;
		//Dictionary<(CCPos, WPos), CCStateWithGoal> ccStateCache = new Dictionary<(CCPos, WPos), CCStateWithGoal>();
		Dictionary<(CCPos, CCPos), CCStateWithGoal> ccStateCache = new();
		private Queue<(CCPos, CCPos)> keys = new();
		private int capacity = 10000;

		public ThetaStarCache()
		{
		}

		void IWorldLoaded.WorldLoaded(World w, WorldRenderer wr)
		{
			world = w;
		}

		//public void AddWithAllParents(CCState ccState, WPos goalPos, int finalGval)
		//{
		//	Add(ccState, goalPos, finalGval);
		//	var parentState = ccState.ParentState;
		//	while (parentState != null)
		//	{
		//		Add(parentState, goalPos, finalGval);
		//		parentState = parentState.ParentState;
		//	}
		//}

		public void Add(CCState ccState, CCPos goalCCPos, WPos goalPos, int finalGval)
		{
			if (ccStateCache.Count >= capacity)
			{
				var oldestKey = keys.Dequeue();
				ccStateCache.Remove(oldestKey);
			}

			var ccKey = (ccState.CC, goalCCPos);
			ccStateCache[ccKey] = new CCStateWithGoal(ccState, goalPos, finalGval);
			keys.Enqueue(ccKey);
		}

		public bool CheckIfInCache(CCState ccState, CCPos goalCCPos)
		{
			var inCache = ccStateCache.ContainsKey((ccState.CC, goalCCPos));
			//if (inCache)
			//	Console.WriteLine($"CheckIfInCache: {inCache}");
			return inCache;
		}

		public CCStateWithGoal Get(CCState ccState, CCPos goalCCPos)
		{
			var ccKey = (ccState.CC, goalCCPos);
			if (!ccStateCache.ContainsKey(ccKey))
				return null;
			return ccStateCache[ccKey];
		}
	}
}

using System;
using System.Collections.Generic;
using OpenRA.Mods.Common.HitShapes;
using OpenRA.Mods.Common.Traits;
using OpenRA.Traits;

namespace OpenRA.Mods.Common.Pathfinder
{
    public class NavMeshPathSearch : BaseOffGridPathSearch, IDisposable
    {
        private readonly NavMesh navMesh; // Changed from WorldNavMesh to NavMesh
        private bool disposed;

        public NavMeshPathSearch(Actor self, WPos source, WPos dest, int currDelayToRun = 2)
            : base(self, source, dest, currDelayToRun)
        {
            // Get the nav mesh trait
            navMesh = self.World.WorldActor.Trait<NavMesh>();
        }

		// TO BE IMPLEMENTED
		public override List<WPos> FindPath(WPos start, WPos goal)
        {
			// Use the nav mesh for pathfinding
			//return navMesh.FindPath(start, goal);

			return new();
        }

        public override void Expand(int maxExpansions)
        {
            // For nav mesh, expansion is not incremental like A*
            Running = false;
        }

		// TO BE IMPLEMENTED
        public bool IsPathObservable(WPos sourcePos, WPos destPos, IHitShape unitHitShape, bool useUnitRadius, int neighbours)
        {
			//var startTriangle = navMesh.FindTriangleContaining(sourcePos);
			//var endTriangle = navMesh.FindTriangleContaining(destPos);

			//return startTriangle != -1 && startTriangle == endTriangle;

			return false;
        }

        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
            }
        }
    }
}

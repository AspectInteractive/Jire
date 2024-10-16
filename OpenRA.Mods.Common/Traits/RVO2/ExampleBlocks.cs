/*
 * Blocks.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/*
 * Example file showing a demo with 100 agents split in four groups initially
 * positioned in four corners of the environment. Each agent attempts to move to
 * other side of the environment through a narrow passage generated by four
 * obstacles. There is no roadmap to guide the agents around the obstacles.
 */

#pragma warning disable format

#define RVOCS_OUTPUT_TIME_AND_POSITIONS
#define RVOCS_SEED_RANDOM_NUMBER_GENERATOR

using System;
using System.Collections.Generic;
using static OpenRA.Mods.Common.Traits.MobileOffGridOverlay;

namespace RVO
{
    public class Blocks
    {
        /* Store the goals of the agents. */
        readonly IList<Vector2> goals;
        bool firstRun = true;
		Blocks blocks;

        /** Random number generator. */
        readonly Random random;

        public Blocks()
        {
            goals = new List<Vector2>();

#if RVOCS_SEED_RANDOM_NUMBER_GENERATOR
            random = new Random();
#else
            random = new Random(0);
#endif
        }

		void setupAgents(AgentPreset agentPreset)
		{
			/*
             * Add agents, specifying their start position, and store their
             * goals on the opposite side of the environment.
             */
			for (int i = 0; i < 5; ++i)
			{
				for (int j = 0; j < 5; ++j)
				{
					Simulator.Instance.addAgent(new Vector2(55.0f + i * 10.0f, 55.0f + j * 10.0f) * 150, agentPreset,
						goal: new Vector2(-75.0f, -75.0f) * 150);

					Simulator.Instance.addAgent(new Vector2(-55.0f - i * 10.0f, 55.0f + j * 10.0f) * 150, agentPreset,
						goal: new Vector2(75.0f, -75.0f) * 150);

					Simulator.Instance.addAgent(new Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f) * 150, agentPreset,
						goal: new Vector2(-75.0f, 75.0f) * 150);

					Simulator.Instance.addAgent(new Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f) * 150, agentPreset,
						goal: new Vector2(75.0f, 75.0f) * 150);
				}
			}
		}
		

		void setupObstacles()
		{
			IList<Vector2> obstacle1 = new List<Vector2>
			{
				new Vector2(-10.0f, 40.0f) * 150,
				new Vector2(-40.0f, 40.0f) * 150,
				new Vector2(-40.0f, 10.0f) * 150,
				new Vector2(-10.0f, 10.0f) * 150
			};
			Simulator.Instance.addObstacle(obstacle1);

			IList<Vector2> obstacle2 = new List<Vector2>
			{
				new Vector2(10.0f, 40.0f) * 150,
				new Vector2(10.0f, 10.0f) * 150,
				new Vector2(40.0f, 10.0f) * 150,
				new Vector2(40.0f, 40.0f) * 150
			};
			Simulator.Instance.addObstacle(obstacle2);

			IList<Vector2> obstacle3 = new List<Vector2>
			{
				new Vector2(10.0f, -40.0f) * 150,
				new Vector2(40.0f, -40.0f) * 150,
				new Vector2(40.0f, -10.0f) * 150,
				new Vector2(10.0f, -10.0f) * 150
			};
			Simulator.Instance.addObstacle(obstacle3);

			IList<Vector2> obstacle4 = new List<Vector2>
			{
				new Vector2(-10.0f, -40.0f) * 150,
				new Vector2(-10.0f, -10.0f) * 150,
				new Vector2(-40.0f, -10.0f) * 150,
				new Vector2(-40.0f, -40.0f) * 150
			};
			Simulator.Instance.addObstacle(obstacle4);
		}

		void setupScenario()
        {
            /* Specify the global time step of the simulation. */
            Simulator.Instance.setTimeStep(0.25f * 150);

			/*
             * Specify the default parameters for agents that are subsequently
             * added.
             */

			var agentPreset = new AgentPreset(
				neighborDist: 15.0f * 150,
				maxNeighbors: 10 * 150,
				timeHorizon: 5.0f * 150,
				timeHorizonObst: 5.0f * 150,
				radius: 2.0f * 150,
				maxSpeed: 2.0f * 150,
				velocity: new Vector2(0.0f, 0.0f));

			setupAgents(agentPreset);
			setupObstacles();

			/*
             * Process the obstacles so that they are accounted for in the
             * simulation.
             */
			Simulator.Instance.processObstacles();
        }

		public IEnumerable<Vector2> getAgentPositions()
		{
			if (Simulator.Instance.getNumAgents() == 0)
				yield break;

			for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
				yield return Simulator.Instance.getAgentPosition(i);
		}

		void setPreferredVelocities()
		{
			for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
				Simulator.Instance.setAgentPrefVelocity(i, Simulator.Instance.agents_[i].GoalVectorNorm, true);
		}

		bool reachedGoal()
		{
			/* Check if all agents have reached their goals. */
			for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
				if (!Simulator.Instance.agents_[i].ReachedGoal)
					return false;
			return true;
		}

        public void Tick()
        {
			if (firstRun)
			{
				blocks = new();

				/* Set up the scenario. */
				blocks.setupScenario();

				firstRun = false;
			}

			/* Perform (and manipulate) the simulation. */
			if (!blocks.reachedGoal())
			{
				// blocks.updateVisualization();
				blocks.setPreferredVelocities();
				Simulator.Instance.doStep();
			}
		}
    }
}

#pragma warning restore format

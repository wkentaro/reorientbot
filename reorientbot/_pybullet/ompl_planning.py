import itertools

import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
import pybullet as p
import pybullet_planning as pp


class pbValidityChecker(ob.StateValidityChecker):
    def __init__(
        self,
        si,
        ri,
        obstacles=None,
        min_distances=None,
        min_distances_start_goal=None,
    ):
        super().__init__(si)
        self.ri = ri
        self.ndof = len(self.ri.joints)
        self.obstacles = obstacles or []
        self.min_distances = min_distances or {}
        self.min_distances_start_goal = min_distances_start_goal or {}
        self.lower, self.upper = ri.get_bounds()
        self.lower = np.asarray(self.lower)
        self.upper = np.asarray(self.upper)

        self.start = None
        self.goal = None

    def isValid(self, state):
        if not self.check_joint_limits(state):
            return False

        j = [state[i] for i in range(self.ndof)]

        if self.min_distances_start_goal:
            if self.start is not None and np.allclose(j, self.start):
                min_distances = self.min_distances_start_goal
            elif self.goal is not None and np.allclose(j, self.goal):
                min_distances = self.min_distances_start_goal
            else:
                min_distances = self.min_distances
        else:
            min_distances = self.min_distances

        with pp.WorldSaver():
            self.ri.setj(j)

            is_valid = self.check_self_collision(
                min_distances=min_distances
            ) and self.check_collision(
                self.obstacles, min_distances=min_distances
            )

        return is_valid

    def check_self_collision(self, min_distances=None):
        min_distances = min_distances or {}

        is_colliding = False

        links = pp.get_links(self.ri.robot)
        for link_a, link_b in itertools.combinations(links, 2):
            link_name_a = pp.get_link_name(self.ri.robot, link_a)
            link_name_b = pp.get_link_name(self.ri.robot, link_b)

            assert link_b > link_a
            if link_b - link_a == 1:
                continue

            distance = 0

            # XXX: specific configurations for panda_drl.urdf
            # panda_link5: front arm
            # panda_link6: arm head
            # panda_link7: wrist
            # panda_link8: palm tip
            if (
                link_name_a == "panda_link7"
                and link_name_b == "panda_suction_gripper"
            ):
                continue
            if (
                link_name_a == "panda_link5"
                and link_name_b == "panda_suction_gripper"
            ):
                continue

            # XXX: specific configurations for panda_suction.urdf
            # panda_link7: wrist
            if link_name_a == "panda_link7" and link_name_b == "baseLink":
                continue

            is_colliding_i = (
                len(
                    p.getClosestPoints(
                        bodyA=self.ri.robot,
                        linkIndexA=link_a,
                        bodyB=self.ri.robot,
                        linkIndexB=link_b,
                        distance=distance,
                    )
                )
                > 0
            )
            # if is_colliding_i:
            #     from loguru import logger
            #
            #     logger.warning(
            #         f"{link_a}:{link_name_a} and {link_b}:{link_name_b} "
            #         f"is self-colliding with distance of {distance}"
            #     )
            is_colliding |= is_colliding_i

        for attachment in self.ri.attachments:
            assert attachment.parent == self.ri.robot
            min_distance = min_distances.get((attachment.child, -1), 0)
            for link in links:
                if link == attachment.parent_link:
                    continue
                is_colliding |= (
                    len(
                        p.getClosestPoints(
                            bodyA=attachment.child,
                            linkIndexA=-1,
                            bodyB=self.ri.robot,
                            linkIndexB=link,
                            distance=min_distance,
                        )
                    )
                    > 0
                )

        return not is_colliding

    def check_collision(self, ids_to_check, min_distances=None):
        min_distances = min_distances or {}

        if len(ids_to_check) == 0:
            return True

        is_colliding = False

        for link in pp.get_links(self.ri.robot):
            min_distance = min_distances.get((self.ri.robot, link), 0)
            is_colliding |= (
                len(
                    p.getClosestPoints(
                        bodyA=self.ri.robot,
                        linkIndexA=link,
                        bodyB=ids_to_check[-1],
                        linkIndexB=-1,
                        distance=min_distance,
                    )
                )
                > 0
            )

        for attachment in self.ri.attachments:
            min_distance = min_distances.get((attachment.child, -1), 0)
            is_colliding |= (
                len(
                    p.getClosestPoints(
                        attachment.child,
                        ids_to_check[-1],
                        distance=min_distance,
                    )
                )
                > 0
            )

        if is_colliding:
            return False
        else:
            return self.check_collision(
                ids_to_check[0:-1], min_distances=min_distances
            )

    def check_joint_limits(self, state):
        for i in range(self.ndof):
            if state[i] > self.upper[i] or state[i] < self.lower[i]:
                return False
        return True

    def sample_state(self):
        q = (
            np.random.random(self.ndof) * (self.upper - self.lower)
            + self.lower
        )
        if self.isValid(q):
            return q
        else:
            return self.sample_state()


class PbPlanner:
    def __init__(
        self,
        ri,
        obstacles=None,
        min_distances=None,
        min_distances_start_goal=None,
        planner="RRTConnect",
        planner_range=0,
    ):
        ndof = len(ri.joints)

        lower, upper = ri.get_bounds()
        bounds = ob.RealVectorBounds(ndof)
        for i in range(ndof):
            bounds.setLow(i, lower[i])
            bounds.setHigh(i, upper[i])

        self.space = ob.RealVectorStateSpace(ndof)
        self.space.setBounds(bounds)

        self.si = ob.SpaceInformation(self.space)

        self.validityChecker = pbValidityChecker(
            self.si,
            ri=ri,
            obstacles=obstacles,
            min_distances=min_distances,
            min_distances_start_goal=min_distances_start_goal,
        )
        self.si.setStateValidityChecker(self.validityChecker)
        self.si.setup()

        self.planner = planner
        self.planner_range = planner_range

    def plan(self, start_q, goal_q):
        log_level = ou.getLogLevel()
        ou.setLogLevel(ou.LOG_WARN)

        # start and goal configs
        start = ob.State(self.space)
        for i in range(len(start_q)):
            start[i] = start_q[i]

        goal = ob.State(self.space)
        for i in range(len(start_q)):
            goal[i] = goal_q[i]

        # setup and solve
        pdef = ob.ProblemDefinition(self.si)
        pdef.setStartAndGoalStates(start, goal)
        pdef.setOptimizationObjective(
            ob.PathLengthOptimizationObjective(self.si)
        )
        optimizingPlanner = getattr(og, self.planner)(self.si)
        optimizingPlanner.setProblemDefinition(pdef)
        optimizingPlanner.setRange(self.planner_range)
        optimizingPlanner.setup()
        solved = optimizingPlanner.solve(solveTime=1)

        if solved:
            path = pdef.getSolutionPath()
            simplifier = og.PathSimplifier(self.si)
            simplifier.simplifyMax(path)
        else:
            # logger.warning("No solution found")
            path = None

        ou.setLogLevel(log_level)

        return path

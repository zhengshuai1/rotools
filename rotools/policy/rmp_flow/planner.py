import numpy as np

from typing import Sized
from attr import attrs, attrib, Factory
from scipy.integrate import solve_ivp

from _root import RMPRoot
from _leaf import GoalAttractorUni


def _init_state_factory(planner):
    return np.zeros((2, planner.dim))


def _init_goal_factory(planner):
    return np.zeros((2, planner.dim))


@attrs
class RMPPlanner(Sized):
    """Planner class using Riemannian Motion Policy."""

    def __len__(self):
        return self.dim

    dim = attrib(type=int)
    _root = attrib(factory=lambda: RMPRoot('root'))
    _leaf = None

    def _dynamics(self, t, state):
        """the function to solve, iteratively used in solve_ivp.

        :param t:
        :param state: ndarray 2*dim [x1, x2, ..., xdot1, xdot2, ...]
        :return:
        """
        state = state.reshape(2, -1)
        x = state[0]
        x_dot = state[1]
        x_ddot = self._root.solve(x, x_dot)  # RMP is used here!
        state_dot = np.concatenate((x_dot, x_ddot), axis=None)
        return state_dot

    def plan(self, state, goal, obstacle=None, span=None):
        """

        :param state: initial state as Sequence[x, ..., xdot, ...]
        :param goal: target state as List[float]
        :param obstacle:
        :param span: the total execution time interval in second
        :return: Trajectory positions, velocities, timestamps
        """
        if span is None:
            span = [0, 5]  # default plan a 5s trajectory

        self._leaf = GoalAttractorUni('goal_attractor', self._root, goal)
        sol = solve_ivp(self._dynamics, span, state)
        if sol:
            positions = sol.y[:self.dim, :]  # shape (dof, N), N is the way point number
            velocities = sol.y[self.dim:, :]
            # accelerations =
            timestamps = sol.t
            return timestamps, positions, velocities  # accelerations
        else:
            return None

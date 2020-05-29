import numpy as np

from typing import Sized
from attr import attrs, attrib, Factory
from scipy.integrate import solve_ivp
from rmp_root import RMPRoot
from rmp_leaf import GoalAttractorUni


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
    state = attrib(
        default=Factory(factory=_init_state_factory, takes_self=True),
        type=np.ndarray,
    )
    goal = attrib(
        default=Factory(factory=_init_goal_factory, takes_self=True),
        type=np.ndarray,
    )
    _root = attrib(factory=lambda: RMPRoot('root'))
    _leaf = None

    def init(self, state, goal):
        self.state = state
        self._leaf = GoalAttractorUni('goal_attractor', self._root, goal)
        self.goal = goal

    def update_goal(self, goal):
        self.goal = goal

    def _dynamics(self, t, state):
        """the function to solve, iteratively used in solve_ivp.

        :param t:
        :param state: ndarray [[x, xdot], ...]
        :return:
        """
        state = state.reshape(2, -1)
        x = state[0]
        x_dot = state[1]
        x_ddot = self._root.solve(x, x_dot)  # RMP is used here!
        state_dot = np.concatenate((x_dot, x_ddot), axis=None)
        return state_dot

    def plan(self, span):
        """

        :param span:
        :return:
        """
        sol = solve_ivp(self._dynamics, span, self.state)
        return sol


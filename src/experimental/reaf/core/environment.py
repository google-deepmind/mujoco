# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The Robotics Environment Authoring Framework (REAF) Environment class."""

import abc
from collections.abc import Mapping
import enum
from typing import Generic

from absl import logging
import dm_env
from dm_env import specs
from gdm_robotics.interfaces import environment as gdmr_env
from gdm_robotics.interfaces import types as gdmr_types
import numpy as np
from reaf.core import action_space_adapter as reaf_action_space_adapter
from reaf.core import data_acquisition_and_control_layer as reaf_dacl
from reaf.core import default_observation_space_adapter
from reaf.core import logger as reaf_logger
from reaf.core import observation_space_adapter as reaf_observation_space_adapter
from reaf.core import pass_through_action_space_adapter
from reaf.core import task_logic_layer as reaf_tll
import tree


class ActionSpecEnforcementOption(enum.StrEnum):
  """Options for action spec enforcement."""

  CLIP_TO_SPEC = "clip_to_spec"
  IGNORE = "ignore"
  WARNING = "warning"
  RAISE_ERROR = "raise_error"


class EnvironmentReset(abc.ABC, Generic[gdmr_env.ResetOptions]):
  """Support for general resets adhering to the GDM environment API."""

  @abc.abstractmethod
  def do_reset(
      self,
      config: gdmr_env.ResetOptions,
  ) -> None:
    """Resets the environment."""

  def default_reset_configuration(self) -> gdmr_env.ResetOptions:
    """Returns the default reset configuration."""
    return gdmr_env.Options()


class EndOfEpisodeHandler:
  """Handler called after the last episode step."""

  def on_end_of_episode_stepping(self, final_timestep: dm_env.TimeStep) -> None:
    """Called when the episode has ended stepping.

    This will be called at the end of every episode, after all other triggers
    have been resolved. Episodes can end either due to truncation or
    termination, i.e. `timestep.step_type` is `StepType.LAST`, or due to an
    early call to `Environment.reset()`. To verify whether it has indeed
    ended due to truncation or termination, the implementer should test
    `timestep.last()`.

    Note that the first reset after environment construction will not trigger
    this handler, but it will be triggered before resolving any subsequent
    environment resets, either implicit or explicit.

    Args:
      final_timestep: The final timestep of the episode that ended stepping.
    """


class EnvironmentCloser(abc.ABC):
  """Handler called when the environment is closed."""

  @abc.abstractmethod
  def close(self) -> None:
    """Releases resources when the environment is closed.

    This method is called automatically when exiting the environment's
    context manager (`with` statement).
    """


class Environment(gdmr_env.Environment):
  """The Robotics Environment Authoring Framework (REAF) Environment class."""

  def __init__(
      self,
      *,
      data_acquisition_and_control_layer: reaf_dacl.DataAcquisitionAndControlLayer,
      task_logic_layer: reaf_tll.TaskLogicLayer,
      environment_reset: EnvironmentReset,
      action_space_adapter: (
          reaf_action_space_adapter.ActionSpaceAdapter | None
      ) = None,
      observation_space_adapter: (
          reaf_observation_space_adapter.ObservationSpaceAdapter | None
      ) = None,
      end_of_episode_handler: EndOfEpisodeHandler | None = None,
      environment_closer: EnvironmentCloser | None = None,
      action_spec_enforcement_option: ActionSpecEnforcementOption = ActionSpecEnforcementOption.RAISE_ERROR,
  ):
    """Creates an environment.

    Args:
      data_acquisition_and_control_layer: The layer for communicating with the
        specific robotic setup.
      task_logic_layer: The layer in charge of defining the task.
      environment_reset: The `EnvironmentReset` specifying the function to be
        called at environment reset and the default environment reset
        configuration.
      action_space_adapter: Adapter from the agent action space to the flattened
        commands accepted by the task layer. If None the
        PassThroughActionSpaceAdapter is used, meaning the entirety of the
        commands dictionary is exposed to the agent.
      observation_space_adapter: Adapter from the computed features to the
        observations that are exposed to the agent. If None the
        DefaultObservationSpaceAdapter is used, meaning all the features are
        exposed to the agent as observations.
      end_of_episode_handler: Called at the end of an episode, after the last
        step.
      environment_closer: Specifies the handler to be called when the
        environment is closed. This is called automatically on exit if the
        environment is used as a context manager. If None, no action is
        performed at close.
      action_spec_enforcement_option: How to enforce the action spec. If
        `CLIP_TO_SPEC`, the action will be clipped to the spec. If `WARNING`, an
        warning logged if the action is outside the spec. If `RAISE_ERROR`, an
        error will be raised if the action is outside the  spec. If `IGNORE`,
        the action will be passed through. Default is `RAISE_ERROR`.
    """

    self._data_acquisition_and_control_layer = (
        data_acquisition_and_control_layer
    )
    self._task_logic_layer = task_logic_layer
    self._end_of_episode_handler = (
        end_of_episode_handler or EndOfEpisodeHandler()
    )
    self._environment_reset = environment_reset
    self._environment_closer = environment_closer
    self._action_spec_enforcement_option = action_spec_enforcement_option

    # Before assigning the adapters, validate the specs on the task logic layer
    # and the DACL.
    self._validate_dacl_and_ttl_specs()

    ttl_commands_spec = self._task_logic_layer.commands_spec(
        self._data_acquisition_and_control_layer.commands_spec()
    )
    ttl_features_spec = self._task_logic_layer.features_spec(
        self._data_acquisition_and_control_layer.measurements_spec()
    )

    if action_space_adapter is None:
      action_space_adapter = (
          pass_through_action_space_adapter.PassThroughActionSpaceAdapter(
              commands_spec=ttl_commands_spec
          )
      )
    self._action_space_adapter = action_space_adapter

    if observation_space_adapter is None:
      observation_space_adapter = (
          default_observation_space_adapter.DefaultObservationSpaceAdapter(
              task_features_spec=ttl_features_spec,
              selected_features=None,
              renamed_features=None,
              observation_type_mapper=None,
          )
      )
    self._observation_space_adapter = observation_space_adapter

    # Now we can validate the adapters.
    self._validate_adapters_specs()

    self._last_timestep: dm_env.TimeStep | None = None
    self._should_finalize_episode = False
    self._timestep_spec = gdmr_types.TimeStepSpec(
        step_type=gdmr_types.STEP_TYPE_SPEC,
        reward=self._task_logic_layer.reward_spec(),
        discount=self._task_logic_layer.discount_spec(),
        # The observation spec corresponds to the one exposed by the adapter.
        observation=self._observation_space_adapter.observation_spec(),
    )

    self._zero_reward, self._zero_discount = tree.map_structure(
        _read_only_zeros_like_spec,
        (self._timestep_spec.reward, self._timestep_spec.discount),
    )

  def close(self) -> None:
    """Frees any resources used by the environment."""
    if self._environment_closer is not None:
      self._environment_closer.close()

  def default_reset_options(self) -> gdmr_env.ResetOptions:
    return self._environment_reset.default_reset_configuration()

  def reset_with_options(
      self,
      *,
      options: gdmr_env.ResetOptions,
  ) -> dm_env.TimeStep:
    """Starts a new sequence and returns the first `TimeStep`."""
    if self._should_finalize_episode:
      self._finalize_episode()
    self._environment_reset.do_reset(options)
    self._task_logic_layer.perform_reset()
    measurements = self._data_acquisition_and_control_layer.begin_stepping()
    features = self._task_logic_layer.compute_all_features(measurements)
    observations = self._compute_observations_from_features(features)

    self._last_timestep = self._restart(observation=observations)
    # Make sure any early reset after this one triggers `_finalize_episode`.
    self._should_finalize_episode = True
    return self._last_timestep

  def action_spec(self) -> gdmr_types.ActionSpec:
    """Defines the actions that should be provided to `step`."""
    # The action spec corresponds to the one exposed by the adapter.
    return self._action_space_adapter.action_spec()

  def timestep_spec(self) -> gdmr_types.TimeStepSpec:
    """Returns the spec associated to the returned TimeStep."""
    return self._timestep_spec

  def step(self, action: gdmr_types.ActionType) -> dm_env.TimeStep:
    """Updates the environment according to action and returns a `TimeStep`."""

    action = self._enforce_action_spec(action)
    if self._last_timestep is None or self._last_timestep.last():
      return self.reset()

    # Process the action to obtain a command.
    commands = self._compute_commands_from_agent_action(action)
    commands = self._task_logic_layer.compute_final_commands(commands)
    measurements = self._data_acquisition_and_control_layer.step(commands)

    # Compute all the features.
    features = self._task_logic_layer.compute_all_features(measurements)

    # Compute the elements of the timestep.
    reward = self._task_logic_layer.compute_reward(features)
    termination_state = self._task_logic_layer.check_for_termination(features)
    discount = self._task_logic_layer.compute_discount(
        features, termination_state
    )

    observations = self._compute_observations_from_features(features)

    if termination_state.is_terminated():
      self._last_timestep = self._termination(
          reward=reward, observation=observations
      )
    elif termination_state.is_truncated():
      self._last_timestep = self._truncation(
          reward=reward, observation=observations, discount=discount
      )
    else:
      self._last_timestep = self._transition(
          reward=reward, observation=observations, discount=discount
      )

    if self._last_timestep.last():
      self._finalize_episode()
    return self._last_timestep

  def _finalize_episode(self) -> None:
    self._data_acquisition_and_control_layer.end_stepping()
    # It's crucial to call `end_stepping` on the dacl before invoking the end
    # of episode handler. This ensures no further `set_command` or
    # `get_measurements` calls are made. In contrast, the end of episode
    # handler might interact with devices, requiring them to be informed
    # beforehand.
    self._end_of_episode_handler.on_end_of_episode_stepping(self._last_timestep)
    self._should_finalize_episode = False

  @property
  def data_acquisition_and_control_layer(
      self,
  ) -> reaf_dacl.DataAcquisitionAndControlLayer:
    return self._data_acquisition_and_control_layer

  @property
  def task_logic_layer(self) -> reaf_tll.TaskLogicLayer:
    return self._task_logic_layer

  @property
  def environment_reset(self) -> EnvironmentReset:
    return self._environment_reset

  @environment_reset.setter
  def environment_reset(self, environment_reset: EnvironmentReset) -> None:
    self._environment_reset = environment_reset

  def add_logger(self, logger: reaf_logger.Logger) -> None:
    self._task_logic_layer.add_logger(logger)

  def remove_logger(self, logger: reaf_logger.Logger) -> None:
    self._task_logic_layer.remove_logger(logger)

  def _validate_dacl_and_ttl_specs(self) -> None:
    """Validates the specs on the task logic layer."""
    # Validate the spec on the task logic layer.
    self._task_logic_layer.validate_spec(
        dacl_commands_spec=(
            self._data_acquisition_and_control_layer.commands_spec()
        ),
        dacl_measurements_spec=(
            self._data_acquisition_and_control_layer.measurements_spec()
        ),
    )

  def _validate_adapters_specs(self) -> None:
    # Collect the full commands and features spec and validate them against
    # the adapters.
    commands_spec = set(
        self._task_logic_layer.commands_spec(
            self._data_acquisition_and_control_layer.commands_spec()
        ).keys()
    )
    features_spec = set(
        self._task_logic_layer.features_spec(
            self._data_acquisition_and_control_layer.measurements_spec()
        )
    )

    # Check the action space adapter.
    adapter_keys = self._action_space_adapter.task_commands_keys()

    if adapter_keys != commands_spec:
      raise ValueError(
          "Mismatch between commands exposed by the action space adapter:"
          f" {adapter_keys} and commands spec expected by the task layer:"
          f" {commands_spec}."
      )

    # Check the observation spec adapter.
    adapter_keys = self._observation_space_adapter.task_features_keys()
    if not adapter_keys.issubset(features_spec):
      raise ValueError(
          "Failed to validate observation space adapter specs. Missing keys:"
          f" {adapter_keys - features_spec}"
      )

  def _compute_observations_from_features(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> tree.Structure[gdmr_types.ArrayType]:
    return self._observation_space_adapter.observations_from_features(features)

  def _compute_commands_from_agent_action(
      self, agent_action: gdmr_types.ActionType
  ) -> Mapping[str, gdmr_types.ArrayType]:
    return self._action_space_adapter.commands_from_environment_action(
        agent_action
    )

  def _restart(
      self,
      observation: tree.Structure[gdmr_types.ArrayType],
  ) -> dm_env.TimeStep:
    """Returns a `TimeStep` with `step_type` set to `StepType.FIRST`."""
    return dm_env.TimeStep(
        step_type=np.asarray(dm_env.StepType.FIRST, dtype=np.uint8),
        observation=observation,
        reward=self._zero_reward,
        discount=self._zero_discount,
    )

  def _transition(
      self,
      reward: tree.Structure[gdmr_types.ArrayType],
      observation: tree.Structure[gdmr_types.ArrayType],
      discount: tree.Structure[gdmr_types.ArrayType],
  ) -> dm_env.TimeStep:
    """Returns a `TimeStep` with `step_type` set to `StepType.MID`."""
    return dm_env.TimeStep(
        step_type=np.asarray(dm_env.StepType.MID, dtype=np.uint8),
        observation=observation,
        reward=reward,
        discount=discount,
    )

  def _termination(
      self,
      reward: tree.Structure[gdmr_types.ArrayType],
      observation: tree.Structure[gdmr_types.ArrayType],
  ) -> dm_env.TimeStep:
    """Returns a `TimeStep` with `step_type` set to `StepType.LAST`."""
    return dm_env.TimeStep(
        step_type=np.asarray(dm_env.StepType.LAST, dtype=np.uint8),
        observation=observation,
        reward=reward,
        discount=self._zero_discount,
    )

  def _truncation(
      self,
      reward: tree.Structure[gdmr_types.ArrayType],
      observation: tree.Structure[gdmr_types.ArrayType],
      discount: tree.Structure[gdmr_types.ArrayType],
  ) -> dm_env.TimeStep:
    """Returns a `TimeStep` with `step_type` set to `StepType.LAST`."""
    return dm_env.TimeStep(
        step_type=np.asarray(dm_env.StepType.LAST, dtype=np.uint8),
        observation=observation,
        reward=reward,
        discount=discount,
    )

  def _enforce_action_spec(
      self, action: gdmr_types.ActionType
  ) -> gdmr_types.ActionType:
    """Enforces the action spec."""
    match self._action_spec_enforcement_option:
      case ActionSpecEnforcementOption.IGNORE:
        pass
      case ActionSpecEnforcementOption.CLIP_TO_SPEC:
        try:

          def clip_to_spec(a, s):
            if isinstance(s, specs.BoundedArray):
              return np.clip(a, s.minimum, s.maximum)
            return a

          action = tree.map_structure(
              clip_to_spec,
              action,
              self._action_space_adapter.action_spec(),
          )
        except ValueError as e:
          raise ValueError(
              "Failed to clip action to spec. Action:"
              f" {action} and spec: {self._action_space_adapter.action_spec()}"
          ) from e
      case ActionSpecEnforcementOption.WARNING:

        def _validate_without_raising(a, s):
          dtype_ok = s.dtype == a.dtype
          shape_ok = s.shape == a.shape
          minimum_ok = True
          maximum_ok = True
          if isinstance(s, specs.BoundedArray):
            minimum_ok = (s.minimum <= a).all()
            maximum_ok = (a <= s.maximum).all()
          return dtype_ok and shape_ok and minimum_ok and maximum_ok

        if not all(
            tree.flatten(
                tree.map_structure(
                    _validate_without_raising,
                    action,
                    self._action_space_adapter.action_spec(),
                )
            )
        ):
          logging.warning(
              "Failed to validate action against spec. Action: %r and spec: %r",
              action,
              self._action_space_adapter.action_spec(),
          )
      case ActionSpecEnforcementOption.RAISE_ERROR:
        action = tree.map_structure(
            lambda a, spec: spec.validate(a), action, self.action_spec()
        )
      case _:
        raise ValueError(
            "Unknown action spec enforcement option:"
            f" {self._action_spec_enforcement_option}"
        )
    return action


def _read_only_zeros_like_spec(spec: specs.Array) -> np.ndarray:
  """Returns a zero array matching the specified spec."""
  arr = np.zeros(shape=spec.shape, dtype=spec.dtype)
  arr.flags.writeable = False
  return arr

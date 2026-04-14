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
"""Task logic layer for the Robotics Environment Authoring Framework."""

from collections.abc import Mapping, Sequence
import itertools
from typing import Protocol

from absl import logging
from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
from reaf.core import commands_processor as reaf_commands_processor
from reaf.core import default_discount_provider
from reaf.core import discount_provider as reaf_discount_provider
from reaf.core import features_observer as reaf_features_observers
from reaf.core import features_producer as reaf_features_producer
from reaf.core import logger as reaf_logger
from reaf.core import reward_provider as reaf_reward_provider
from reaf.core import termination_checker as reaf_termination_checker
from reaf.core import zero_reward_provider
import tree


class _ResettableObject(Protocol):
  """Protocol for an object that can be reset."""

  def reset(self) -> None:
    ...


class TaskLogicLayer:
  """Task logic layer for the Robotics Environment Authoring Framework."""

  def __init__(
      self,
      *,
      commands_processors: Sequence[reaf_commands_processor.CommandsProcessor],
      features_producers: Sequence[reaf_features_producer.FeaturesProducer],
      termination_checkers: Sequence[
          reaf_termination_checker.TerminationChecker
      ],
      reward_provider: reaf_reward_provider.RewardProvider | None = None,
      discount_provider: reaf_discount_provider.DiscountProvider | None = None,
      features_observers: Sequence[
          reaf_features_observers.FeaturesObserver
      ] = (),
      loggers: Sequence[reaf_logger.Logger] = (),
  ):
    """Initializes the task logic layer.

    Args:
      commands_processors: `CommandsProcessor`s that modify the commands before
        being sent down to the DACL. They are called sequentially, starting from
        the commands supplied by the policy and ending with the commands that
        will be sent to the DACL.
      features_producers: `FeaturesProducer`s that generate new features.
        Measurements collected by the DACL and features produced by these
        `FeaturesProducer`s are then merged into the final feature set that is
        provided to the `reward_provider`, `termination_checkers`,
        `discount_provider`, `features_observers`, and `loggers`.
      termination_checkers: `TerminationChecker`s that check the episode
        termination based on the final feature set.
      reward_provider: `RewardProvider` that computes a reward based on the
        final feature set. If None, the ZeroRewardProvider is used and the
        reward is set to 0.
      discount_provider: `DiscountProvider` that compute a discount based on the
        final feature set and final termination state. If None, the
        DefaultDiscountProvider is used returning 0 for termination and 1 for
        truncation and non-termination.
      features_observers: `FeaturesObserver`s that get a view over the final
        feature set.
      loggers: `Logger`s for logging measurements, features, and commands in the
        task layer.
    """
    self._commands_processors = commands_processors
    self._features_producers = features_producers
    self._reward_provider = (
        reward_provider
        if reward_provider
        else zero_reward_provider.ZeroRewardProvider()
    )
    self._termination_checkers = termination_checkers
    self._discount_provider = (
        discount_provider
        if discount_provider
        else default_discount_provider.DefaultDiscountProvider()
    )
    self._features_observers = features_observers
    self._loggers = list(loggers)

    # We make a set of all resettable objects so that these objects only get
    # their resets called once. This is important for e.g. when having a single
    # object that derives from two interfaces.
    self._resettable_objects: list[_ResettableObject] = []
    unique_ids = set()
    for resettable_object in itertools.chain(
        self._commands_processors,
        self._features_producers,
        self._termination_checkers,
        [self._reward_provider],
        [self._discount_provider],
    ):
      resettable_object_id = id(resettable_object)
      if resettable_object_id not in unique_ids:
        unique_ids.add(resettable_object_id)
        self._resettable_objects.append(resettable_object)

  def validate_spec(
      self,
      *,
      dacl_commands_spec: Mapping[str, gdmr_types.AnyArraySpec],
      dacl_measurements_spec: Mapping[str, specs.Array],
  ) -> None:
    """Checks that the specs have consistent keys."""
    logging.vlog(3, "Validate features processing")
    self._validate_features_spec(dacl_measurements_spec)
    self._validate_commands_spec(dacl_commands_spec)

  def features_spec(
      self,
      dacl_measurements_spec: Mapping[str, specs.Array],
  ) -> Mapping[str, specs.Array]:
    """Returns the features spec as exposed by the task layer."""
    spec = dict(dacl_measurements_spec)
    for features_producer in self._features_producers:
      spec.update(features_producer.produced_features_spec())

    return spec

  def commands_spec(
      self, dacl_commands_spec: Mapping[str, gdmr_types.AnyArraySpec]
  ) -> Mapping[str, gdmr_types.AnyArraySpec]:
    """Returns the commands spec exposed by the task layer."""
    # Each processor consumes commands (as described by its
    # `consumed_commands_spec`) and outputs a potentially different set of
    # commands (as described by its `produced_commands_keys`).
    # Starting with the DACL command spec, we iterate in reverse order (i.e. in
    # the direction DACL -> Policy) through every processor to remove the
    # `produced_commands_keys` from the spec, and add their
    # `consumed_commands_spec` to the spec.
    spec: Mapping[str, gdmr_types.AnyArraySpec] = dict(dacl_commands_spec)
    for processor in reversed(self._commands_processors):
      processor_produced_keys = processor.produced_commands_keys()
      spec = {
          key: value
          for key, value in spec.items()
          if key not in processor_produced_keys
      }
      spec.update(processor.consumed_commands_spec())
    return spec

  def reward_spec(self) -> tree.Structure[specs.Array]:
    return self._reward_provider.reward_spec()

  def discount_spec(self) -> tree.Structure[specs.Array]:
    return self._discount_provider.discount_spec()

  def perform_reset(self) -> None:
    """Reset the internal state of the task logic layer."""
    for resettable_object in self._resettable_objects:
      resettable_object.reset()

  def compute_all_features(
      self, measurements: Mapping[str, gdmr_types.ArrayType]
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Computes all the task logic features given the current measurements."""
    for logger in self._loggers:
      logger.record_measurements(measurements)

    # Produce all the features.
    current_features = dict(measurements)
    for feature_producer in self._features_producers:
      required_features = {
          key: current_features[key]
          for key in feature_producer.required_features_keys()
      }
      current_features.update(
          feature_producer.produce_features(required_features)
      )

    # Observe the features.
    for feature_observer in self._features_observers:
      feature_observer.observe_features(current_features)

    # Log the resulting features.
    for logger in self._loggers:
      logger.record_features(current_features)
    return current_features

  def compute_final_commands(
      self,
      policy_commands: Mapping[str, gdmr_types.ArrayType],
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Processes the policy commands and returns the final processed commands."""
    current_commands = dict(policy_commands)
    for processor in self._commands_processors:
      # Get commands to be consumed by the processor and remove the commands
      # from the current_commands.. They correspond to the
      # `consumed_command_spec`.
      consumed_commands = {
          key: current_commands.pop(key)
          for key in processor.consumed_commands_spec().keys()
      }
      produced_commands = processor.process_commands(consumed_commands)
      current_commands.update(produced_commands)

      # Log the modification.
      for logger in self._loggers:
        logger.record_commands_processing(
            processor.name, consumed_commands, produced_commands
        )

    for logger in self._loggers:
      logger.record_final_commands(current_commands)
    return current_commands

  def compute_reward(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Computes the reward given the features."""
    return self._reward_provider.compute_reward({
        key: features[key]
        for key in self._reward_provider.required_features_keys()
    })

  def check_for_termination(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> reaf_termination_checker.TerminationResult:
    """Checks for termination."""
    current_state = reaf_termination_checker.TerminationResult.DO_NOT_TERMINATE
    for termination_checker in self._termination_checkers:
      current_state = reaf_termination_checker.TerminationResult.combine(
          current_state,
          termination_checker.check_termination({
              key: features[key]
              for key in termination_checker.required_features_keys()
          }),
      )
    return current_state

  def compute_discount(
      self,
      features: Mapping[str, gdmr_types.ArrayType],
      termination_state: reaf_termination_checker.TerminationResult,
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Computes the discount given the features and termination state."""
    return self._discount_provider.compute_discount(
        {
            key: features[key]
            for key in self._discount_provider.required_features_keys()
        },
        termination_state,
    )

  def add_logger(self, logger: reaf_logger.Logger) -> None:
    self._loggers.append(logger)

  def remove_logger(self, logger: reaf_logger.Logger) -> None:
    self._loggers.remove(logger)

  def _validate_features_spec(
      self, dacl_measurements_spec: Mapping[str, specs.Array]
  ) -> None:
    """Validates the features spec."""
    # Check measurements/features path.
    current_key_set = set(dacl_measurements_spec.keys())
    logging.vlog(4, "DACL measurements keys: %s", current_key_set)

    for producer in self._features_producers:
      logging.vlog(
          4,
          "Producer %s requires %s.",
          producer.name,
          producer.required_features_keys(),
      )
      # Check required features are available.
      if not producer.required_features_keys().issubset(current_key_set):
        raise ValueError(
            "Failed to validate feature specs for feature producer"
            f" {producer.name}. Missing keys:"
            f" {producer.required_features_keys() - current_key_set}"
        )
      # Check that there are not duplicates in the output.
      if not current_key_set.isdisjoint(
          producer.produced_features_spec().keys()
      ):
        raise ValueError(
            "Failed to validate feature specs for feature producer"
            f" {producer.name}. Duplicate keys:"
            f" {current_key_set & producer.produced_features_spec().keys()}"
        )
      # Now extend the spec.
      logging.vlog(
          4,
          "Update available keys (from producer %s) with %s.",
          producer.name,
          producer.produced_features_spec().keys(),
      )
      current_key_set.update(producer.produced_features_spec().keys())
      logging.vlog(4, "Available features keys %s.", current_key_set)

  def _validate_commands_spec(
      self, dacl_commands_spec: Mapping[str, gdmr_types.AnyArraySpec]
  ) -> None:
    """Validates the commands spec."""
    # Check commands. Starting from the DACL command specs we propagate up in
    # the chain.
    logging.vlog(3, "Validate commands processing from DACL to Policy.")
    current_key_set = set(dacl_commands_spec.keys())
    logging.vlog(4, "DACL commands keys: %s", current_key_set)

    for processor in reversed(self._commands_processors):
      produced_command_keys = processor.produced_commands_keys()

      logging.vlog(
          4,
          "Processor %s: specs (accepted keys) %s. Exposes %s.",
          processor.name,
          processor.consumed_commands_spec().keys(),
          produced_command_keys,
      )
      if not produced_command_keys.issubset(current_key_set):
        raise ValueError(
            "Failed to validate commands specs for commands processor"
            f" {processor.name}. Missing (consumable) keys:"
            f" {produced_command_keys - current_key_set}"
        )
      # Remove the produced keys and add the consumed commands specs (as the
      # processor is mutable).
      current_key_set = current_key_set - produced_command_keys
      current_key_set.update(processor.consumed_commands_spec().keys())

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
"""Testing functions for asserting on Mock objects with numpy structures."""

from collections.abc import Sequence
from unittest import mock
import numpy as np


def assert_called_once_with(mock_obj: mock.Mock, *args, **kwargs) -> None:
  if mock_obj.call_count != 1:
    raise AssertionError(
        f"Expected exactly one call to {mock_obj}, got {mock_obj.call_count}"
    )

  assert_called_with(mock_obj, *args, **kwargs)


def assert_called_with(mock_obj: mock.Mock, *args, **kwargs) -> None:
  """Asserts that the last call to mock_obj had the specified arguments."""
  if mock_obj.call_args is None:
    raise AssertionError(
        f"Mock object {mock_obj} not called. Expected one call."
    )
  call_args, call_kwargs = mock_obj.call_args
  np.testing.assert_equal(call_args, args)
  np.testing.assert_equal(call_kwargs, kwargs)


def assert_has_calls(
    mock_obj: mock.Mock, calls: Sequence[mock._Call], any_order: bool = False
) -> None:
  """Asserts that mock_obj has been called with the specified calls."""
  mock_calls = mock_obj.mock_calls

  # Check that there are at least enough calls.
  if mock_obj.call_count < len(calls):
    raise AssertionError(
        f"Expected at least {len(calls)} calls to {mock_obj}, got"
        f" {mock_obj.call_count}"
    )

  def _calls_are_equal(actual: mock._Call, expected: mock._Call) -> bool:
    _, actual_args, actual_kwargs = actual
    _, expected_args, expected_kwargs = expected
    # Quickest way to transform the assertion into a comparator.
    try:
      np.testing.assert_equal(actual_args, expected_args)
      np.testing.assert_equal(actual_kwargs, expected_kwargs)
      return True
    except AssertionError:
      return False

  if any_order:
    # We just check for the calls to be contained.
    for expected_call in calls:
      for actual_call in mock_calls:
        if _calls_are_equal(actual_call, expected_call):
          break
      raise AssertionError(
          f"Expected call {expected_call} not found in mock calls {mock_calls}."
      )
    return

  # We need to check in order, but first find the first call.
  starting_index = -1
  first_expected_call = calls[0]
  for index, actual_call in enumerate(mock_calls):
    if _calls_are_equal(actual_call, first_expected_call):
      starting_index = index
      break
  if starting_index == -1:
    raise AssertionError(f"Calls {calls} not found in mock calls {mock_calls}.")

  non_matching_calls = []

  # We have the first element. Now we need to compare element wise.
  for index, expected_call in enumerate(calls):
    actual_call = mock_calls[starting_index + index]
    if not _calls_are_equal(actual_call, expected_call):
      non_matching_calls.append((index, expected_call, actual_call))

  if non_matching_calls:
    raise AssertionError(
        f"Calls {calls} do not match mock calls {mock_calls}. Mismatch (index,"
        f" expected, actual): {non_matching_calls}"
    )

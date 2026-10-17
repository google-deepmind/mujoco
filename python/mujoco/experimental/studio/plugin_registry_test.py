# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests for Studio plugin discovery, dispatch, and failure diagnostics."""

import logging
import traceback
from unittest import mock

from absl.testing import absltest
from absl.testing import parameterized

from mujoco.experimental.studio import messages
from mujoco.experimental.studio import plugin_registry


class _Plugin:

  def __init__(self, callback):
    self.callback = callback

  @messages.handler
  def on_event(self, event: messages.Event):
    return self.callback(event)


class _CriticalPlugin(_Plugin):

  @messages.handler(priority=messages.Priority.CRITICAL)
  def on_event(self, event: messages.Event):
    return self.callback(event)


class PluginRegistryTest(parameterized.TestCase):

  def test_empty_registry(self):
    with self.assertNoLogs():
      plugin_registry.PluginRegistry().dispatch(messages.BuildGuiEvent())

  @parameterized.parameters(None, False)
  def test_unconsumed_event_reaches_next_handler(self, result):
    first = mock.Mock(return_value=result)
    second = mock.Mock(return_value=None)
    registry = plugin_registry.PluginRegistry([_Plugin(first), _Plugin(second)])
    event = messages.BuildGuiEvent()

    with self.assertNoLogs():
      registry.dispatch(event)

    first.assert_called_once_with(event)
    second.assert_called_once_with(event)

  def test_consumed_event_stops_dispatch(self):
    first = mock.Mock(return_value=True)
    second = mock.Mock()
    registry = plugin_registry.PluginRegistry([_Plugin(first), _Plugin(second)])

    registry.dispatch(messages.BuildGuiEvent())

    first.assert_called_once()
    second.assert_not_called()

  def test_priorities_and_stable_registration_order(self):
    calls = []
    first = _Plugin(lambda event: calls.append('first'))
    second = _Plugin(lambda event: calls.append('second'))
    critical = _CriticalPlugin(lambda event: calls.append('critical'))
    registry = plugin_registry.PluginRegistry([first, critical, second])

    registry.dispatch(messages.BuildGuiEvent())

    self.assertEqual(calls, ['critical', 'first', 'second'])

  def test_inherited_handler_and_message(self):
    class InheritedPlugin(_Plugin):
      pass

    callback = mock.Mock(return_value=None)
    registry = plugin_registry.PluginRegistry([InheritedPlugin(callback)])
    event = messages.BuildGuiEvent()

    registry.dispatch(event)
    registry.dispatch(messages.Snapshot())

    callback.assert_called_once_with(event)

  @parameterized.parameters(ValueError, TypeError)
  def test_exception_logs_handler_event_and_traceback_then_reraises(
      self, error_type
  ):
    error = error_type('deliberate plugin failure')

    def fail(event):
      raise error

    second = mock.Mock()
    registry = plugin_registry.PluginRegistry([_Plugin(fail), _Plugin(second)])

    with self.assertLogs(level='ERROR') as logs:
      with self.assertRaises(error_type) as raised:
        registry.dispatch(messages.BuildGuiEvent())

    self.assertIs(raised.exception, error)
    second.assert_not_called()
    self.assertLen(logs.records, 1)
    record = logs.records[0]
    self.assertEqual(record.levelno, logging.ERROR)
    self.assertIn('_Plugin.on_event', record.getMessage())
    self.assertIn('BuildGuiEvent', record.getMessage())
    self.assertIs(record.exc_info[1], error)
    self.assertIn(
        'fail', [f.name for f in traceback.extract_tb(record.exc_info[2])]
    )
    self.assertIn('deliberate plugin failure', logs.output[0])

  def test_failure_does_not_disable_handler(self):
    callback = mock.Mock(side_effect=[ValueError('first call'), None])
    registry = plugin_registry.PluginRegistry([_Plugin(callback)])
    event = messages.BuildGuiEvent()

    with self.assertLogs(level='ERROR'):
      with self.assertRaises(ValueError):
        registry.dispatch(event)
    with self.assertNoLogs():
      registry.dispatch(event)

    self.assertEqual(
        callback.call_args_list, [mock.call(event), mock.call(event)]
    )

  @parameterized.parameters(KeyboardInterrupt, SystemExit)
  def test_process_control_exceptions_propagate_without_error_log(
      self, error_type
  ):
    error = error_type()
    callback = mock.Mock(side_effect=error)
    registry = plugin_registry.PluginRegistry([_Plugin(callback)])

    with self.assertNoLogs():
      with self.assertRaises(error_type) as raised:
        registry.dispatch(messages.BuildGuiEvent())

    self.assertIs(raised.exception, error)


if __name__ == '__main__':
  absltest.main()

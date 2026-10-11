# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Thread-safety tests for MuJoCo Python bindings.

Validates that the free-threading mutex guards in the bindings work correctly:
- Concurrent access to lazy-initialized indexers and struct lists.
- Concurrent callback setter/getter operations.
- Concurrent MjModel/MjData creation and destruction.
- Re-entrancy safety (no deadlock when __del__ re-enters callback setters).
"""

import threading

from absl.testing import absltest
import mujoco


# A model with contacts so data.contact is non-empty after stepping.
_CONTACT_XML = r"""
<mujoco>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.05">
      <freejoint/>
      <geom type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
"""

_NUM_THREADS = 8
_ITERS_PER_THREAD = 50


class ConcurrentStressTest(absltest.TestCase):
  """Tests concurrent access to bindings under multiple threads."""

  def test_concurrent_indexer_access(self):
    """Concurrent first-access of lazy indexers must not corrupt state."""
    model = mujoco.MjModel.from_xml_string(_CONTACT_XML)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    errors = []

    def access_indexers():
      try:
        for _ in range(_ITERS_PER_THREAD):
          # Each of these triggers lazy init on first access.
          _ = model.geom(0)
          _ = data.qpos
          _ = data.xpos
      except Exception as e:  # pylint: disable=broad-except
        errors.append(e)

    threads = [threading.Thread(target=access_indexers)
               for _ in range(_NUM_THREADS)]
    for t in threads:
      t.start()
    for t in threads:
      t.join()
    self.assertEmpty(errors, f"Errors in concurrent indexer access: {errors}")

  def test_concurrent_struct_list_access(self):
    """Concurrent access to contact struct lists must not crash."""
    model = mujoco.MjModel.from_xml_string(_CONTACT_XML)
    data = mujoco.MjData(model)
    # Step to generate contacts.
    mujoco.mj_step(model, data)
    errors = []

    def access_contacts():
      try:
        for _ in range(_ITERS_PER_THREAD):
          ncon = data.ncon
          if ncon > 0:
            contacts = data.contact[:ncon]
            _ = len(contacts)
      except Exception as e:  # pylint: disable=broad-except
        errors.append(e)

    threads = [threading.Thread(target=access_contacts)
               for _ in range(_NUM_THREADS)]
    for t in threads:
      t.start()
    for t in threads:
      t.join()
    self.assertEmpty(
        errors, f"Errors in concurrent struct list access: {errors}")

  def test_concurrent_callback_set_get(self):
    """Concurrent callback setter/getter calls must not crash or deadlock."""
    errors = []

    def toggle_callback():
      try:
        for i in range(_ITERS_PER_THREAD):
          if i % 2 == 0:
            mujoco.set_mjcb_passive(lambda m, d: None)
          else:
            mujoco.set_mjcb_passive(None)
          _ = mujoco.get_mjcb_passive()
      except Exception as e:  # pylint: disable=broad-except
        errors.append(e)

    threads = [threading.Thread(target=toggle_callback)
               for _ in range(_NUM_THREADS)]
    for t in threads:
      t.start()
    for t in threads:
      t.join()
    # Clean up.
    mujoco.set_mjcb_passive(None)
    self.assertEmpty(
        errors, f"Errors in concurrent callback set/get: {errors}")

  def test_concurrent_model_data_lifecycle(self):
    """Concurrent MjModel/MjData creation and destruction must not crash."""
    errors = []

    def create_destroy():
      try:
        for _ in range(_ITERS_PER_THREAD):
          m = mujoco.MjModel.from_xml_string(_CONTACT_XML)
          d = mujoco.MjData(m)
          mujoco.mj_step(m, d)
          del d
          del m
      except Exception as e:  # pylint: disable=broad-except
        errors.append(e)

    threads = [threading.Thread(target=create_destroy)
               for _ in range(_NUM_THREADS)]
    for t in threads:
      t.start()
    for t in threads:
      t.join()
    self.assertEmpty(
        errors, f"Errors in concurrent model/data lifecycle: {errors}")


class ReentrancyTest(absltest.TestCase):
  """Tests that mutexes don't deadlock when __del__ re-enters setters."""

  def test_callback_setter_from_del(self):
    """Setting a callback whose __del__ re-enters the setter must not hang."""

    class ReentrantCallback:
      """A callable whose destructor re-enters the callback setter."""

      def __call__(self, m, d):
        pass

      def __del__(self):
        # When this object is destroyed by set_mjcb_passive(None) or by
        # being replaced, __del__ will try to set the callback again.
        # This must not deadlock.
        try:
          mujoco.set_mjcb_passive(None)
        except Exception:  # pylint: disable=broad-except
          pass  # Swallow — we just want to verify no deadlock.

    mujoco.set_mjcb_passive(ReentrantCallback())
    # This replaces the callback → drops last ref → triggers __del__ →
    # re-enters set_mjcb_passive. Must complete without hanging.
    mujoco.set_mjcb_passive(None)

    # If we get here, no deadlock occurred.
    self.assertIsNone(mujoco.get_mjcb_passive())

  def test_callback_replacement_from_del(self):
    """Replacing a callback whose __del__ sets a new callback must not hang."""

    class ChainedCallback:
      """A callable whose destructor sets a different callback."""

      def __call__(self, m, d):
        pass

      def __del__(self):
        try:
          mujoco.set_mjcb_passive(lambda m, d: None)
        except Exception:  # pylint: disable=broad-except
          pass

    mujoco.set_mjcb_passive(ChainedCallback())
    # Replace with a plain lambda — old ChainedCallback.__del__ fires.
    mujoco.set_mjcb_passive(lambda m, d: None)

    # Clean up.
    cb = mujoco.get_mjcb_passive()
    self.assertIsNotNone(cb)
    mujoco.set_mjcb_passive(None)


if __name__ == "__main__":
  absltest.main()

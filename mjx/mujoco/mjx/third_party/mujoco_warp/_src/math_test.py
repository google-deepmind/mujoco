# Copyright 2025 The Newton Developers
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
# ==============================================================================

import warp as wp
from absl.testing import absltest

from mujoco.mjx.third_party.mujoco_warp._src.math import closest_segment_to_segment_points
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_tri_index
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_trid_index


class ClosestSegmentSegmentPointsTest(absltest.TestCase):
  """Tests for closest segment-to-segment points."""

  def test_closest_segments_points(self):
    """Test closest points between two segments."""
    a0 = wp.vec3([0.73432405, 0.12372768, 0.20272314])
    a1 = wp.vec3([1.10600128, 0.88555209, 0.65209485])
    b0 = wp.vec3([0.85599262, 0.61736299, 0.9843583])
    b1 = wp.vec3([1.84270939, 0.92891793, 1.36343326])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [1.09063, 0.85404, 0.63351], 5)
    self.assertSequenceAlmostEqual(best_b, [0.99596, 0.66156, 1.03813], 5)

  def test_intersecting_segments(self):
    """Tests segments that intersect."""
    a0, a1 = wp.vec3([0.0, 0.0, -1.0]), wp.vec3([0.0, 0.0, 1.0])
    b0, b1 = wp.vec3([-1.0, 0.0, 0.0]), wp.vec3([1.0, 0.0, 0.0])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [0.0, 0.0, 0.0], 5)
    self.assertSequenceAlmostEqual(best_b, [0.0, 0.0, 0.0], 5)

  def test_intersecting_lines(self):
    """Tests that intersecting lines get clipped."""
    a0, a1 = wp.vec3([0.2, 0.2, 0.0]), wp.vec3([1.0, 1.0, 0.0])
    b0, b1 = wp.vec3([0.2, 0.4, 0.0]), wp.vec3([1.0, 2.0, 0.0])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [0.3, 0.3, 0.0], 2)
    self.assertSequenceAlmostEqual(best_b, [0.2, 0.4, 0.0], 2)

  def test_parallel_segments(self):
    """Tests that parallel segments have closest points at the midpoint."""
    a0, a1 = wp.vec3([0.0, 0.0, -1.0]), wp.vec3([0.0, 0.0, 1.0])
    b0, b1 = wp.vec3([1.0, 0.0, -1.0]), wp.vec3([1.0, 0.0, 1.0])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [0.0, 0.0, 0.0], 5)
    self.assertSequenceAlmostEqual(best_b, [1.0, 0.0, 0.0], 5)

  def test_parallel_offset_segments(self):
    """Tests that offset parallel segments are close at segment endpoints."""
    a0, a1 = wp.vec3([0.0, 0.0, -1.0]), wp.vec3([0.0, 0.0, 1.0])
    b0, b1 = wp.vec3([1.0, 0.0, 1.0]), wp.vec3([1.0, 0.0, 3.0])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [0.0, 0.0, 1.0], 5)
    self.assertSequenceAlmostEqual(best_b, [1.0, 0.0, 1.0], 5)

  def test_zero_length_segments(self):
    """Test that zero length segments don't return NaNs."""
    a0, a1 = wp.vec3([0.0, 0.0, -1.0]), wp.vec3([0.0, 0.0, -1.0])
    b0, b1 = wp.vec3([1.0, 0.0, 0.1]), wp.vec3([1.0, 0.0, 0.1])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [0.0, 0.0, -1.0], 5)
    self.assertSequenceAlmostEqual(best_b, [1.0, 0.0, 0.1], 5)

  def test_overlapping_segments(self):
    """Tests that perfectly overlapping segments intersect at the midpoints."""
    a0, a1 = wp.vec3([0.0, 0.0, -1.0]), wp.vec3([0.0, 0.0, 1.0])
    b0, b1 = wp.vec3([0.0, 0.0, -1.0]), wp.vec3([0.0, 0.0, 1.0])

    best_a, best_b = closest_segment_to_segment_points(a0, a1, b0, b1)
    self.assertSequenceAlmostEqual(best_a, [0.0, 0.0, 0.0], 5)
    self.assertSequenceAlmostEqual(best_b, [0.0, 0.0, 0.0], 5)

  def test_upper_tri_index2(self):
    """Tests upper_tri_index with size 2"""
    arr = []
    for i in range(2):
      for j in range(i + 1, 2):
        arr.append(upper_tri_index(2, i, j))
    self.assertEqual(arr, list(range(0, 1)))

  def test_upper_tri_index10(self):
    """Tests upper_tri_index with size 10"""
    arr = []
    for i in range(10):
      for j in range(i + 1, 10):
        arr.append(upper_tri_index(10, i, j))
    self.assertEqual(arr, list(range(0, 45)))

  def test_upper_trid_index1(self):
    """Tests upper_trid_index with size 1"""
    arr = []
    for i in range(1):
      for j in range(i, 1):
        arr.append(upper_trid_index(1, i, j))
    self.assertEqual(arr, list(range(0, 1)))

  def test_upper_trid_index10(self):
    """Tests upper_trid_index with size 10"""
    arr = []
    for i in range(10):
      for j in range(i, 10):
        arr.append(upper_trid_index(10, i, j))
    self.assertEqual(arr, list(range(0, 55)))

  def test_upper_trid_index10(self):
    """Tests upper_trid_index works with symmetric matrix"""
    self.assertEqual(upper_trid_index(10, 1, 5), upper_trid_index(10, 5, 1))


if __name__ == "__main__":
  wp.init()
  absltest.main()

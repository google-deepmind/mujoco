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
# ==============================================================================

from absl.testing import absltest
from mujoco import _render_filament as mjrf
from mujoco.rendering.filament import renderer
import numpy as np


class FakeDLPackTensor:
  """Mock object implementing the DLPack protocol without Buffer."""

  def __init__(self, array: np.ndarray, is_cuda: bool = False):
    self._array = array
    self._is_cuda = is_cuda

  def __dlpack__(self, *, stream=None, **kwargs):
    del stream
    if self._is_cuda:
      raise BufferError("Unsupported device in DLTensor.")
    return self._array.__dlpack__(**kwargs)

  def __dlpack_device__(self) -> tuple[int, int]:
    return (2, 0) if self._is_cuda else self._array.__dlpack_device__()


class RendererTest(absltest.TestCase):

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    cls.ctx = mjrf.Context(mjrf.ContextConfig())

  def setUp(self):
    super().setUp()
    self.renderer = renderer.Renderer(self.ctx)

  def test_target_with_numpy_buffer(self):
    buf = np.zeros((128, 256, 3), dtype=np.uint8)
    target = self.renderer.target("test", buf)
    self.assertIsNotNone(target)

  def test_target_with_dlpack_tensor(self):
    tensor = FakeDLPackTensor(np.zeros((128, 256, 3), dtype=np.uint8))
    target = self.renderer.target("test", tensor)
    self.assertIsNotNone(target)

  def test_target_single_channel_2d_and_3d_shapes(self):
    buf_2d = np.zeros((128, 256), dtype=np.float32)
    target1 = self.renderer.target(
        "depth_2d",
        buf_2d,
        pixel_format=mjrf.PixelFormat.PIXEL_FORMAT_DEPTH32F,
    )
    self.assertIsNotNone(target1)

    buf_3d = np.zeros((128, 256, 1), dtype=np.float32)
    target2 = self.renderer.target(
        "depth_3d",
        buf_3d,
        pixel_format=mjrf.PixelFormat.PIXEL_FORMAT_DEPTH32F,
    )
    self.assertIsNotNone(target2)

  def test_target_update_and_format_mismatch_raises(self):
    buf1 = np.zeros((128, 128, 3), dtype=np.uint8)
    target1 = self.renderer.target("test", buf1)
    buf2 = np.zeros((256, 256, 3), dtype=np.uint8)
    target2 = self.renderer.target("test", buf2)
    self.assertIs(target1, target2)

    buf_rgba = np.zeros((256, 256, 4), dtype=np.uint8)
    with self.assertRaisesRegex(
        ValueError, "already exists with a different pixel format"
    ):
      self.renderer.target(
          "test",
          buf_rgba,
          pixel_format=mjrf.PixelFormat.PIXEL_FORMAT_RGBA8,
      )

  def test_target_with_gpu_dlpack_tensor_raises(self):
    tensor = FakeDLPackTensor(
        np.zeros((128, 128, 3), dtype=np.uint8), is_cuda=True
    )
    with self.assertRaisesRegex(BufferError, "Unsupported device in DLTensor"):
      self.renderer.target("test", tensor)

  def test_target_with_invalid_shape_raises(self):
    # Wrong number of channels (4 instead of 3 for RGB8)
    wrong_channels = np.zeros((128, 256, 4), dtype=np.uint8)
    with self.assertRaisesRegex(
        ValueError, r"Buffer shape \(128, 256, 4\) does not match expected"
    ):
      self.renderer.target("test", wrong_channels)

    # 2D buffer for 3-channel RGB8 format
    buf_2d = np.zeros((128, 256), dtype=np.uint8)
    with self.assertRaisesRegex(
        ValueError, r"Buffer shape \(128, 256\) does not match expected"
    ):
      self.renderer.target("test", buf_2d)

    # 1D buffer
    flat = bytearray(256 * 128 * 3)
    with self.assertRaisesRegex(
        ValueError, "Buffer shape .* does not match expected shape"
    ):
      self.renderer.target("test", flat)

    # Zero dimension
    zero_dim = np.zeros((0, 128, 3), dtype=np.uint8)
    with self.assertRaisesRegex(
        ValueError, "Buffer dimensions must be positive"
    ):
      self.renderer.target("test", zero_dim)

  def test_target_with_mismatched_dtype_raises(self):
    wrong_dtype = np.zeros((128, 128, 3), dtype=np.float32)
    with self.assertRaisesRegex(
        ValueError, "Buffer dtype float32 does not match expected dtype uint8"
    ):
      self.renderer.target("test", wrong_dtype)

  def test_target_with_readonly_buffer_raises(self):
    readonly_buf = np.zeros((128, 128, 3), dtype=np.uint8)
    readonly_buf.flags.writeable = False
    with self.assertRaisesRegex(ValueError, "read-only"):
      self.renderer.target("test", readonly_buf)

    valid_buf = np.zeros((128, 128, 3), dtype=np.uint8)
    target = self.renderer.target("test", valid_buf)
    self.assertIsNotNone(target)

  def test_target_with_non_contiguous_buffer_raises(self):
    non_contig = np.zeros((128, 256, 3), dtype=np.uint8)[:, ::2, :]
    with self.assertRaisesRegex(ValueError, "must be C-contiguous"):
      self.renderer.target("test", non_contig)


if __name__ == "__main__":
  absltest.main()

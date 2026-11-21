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
"""Tests for the Studio web server."""

import asyncio
import gc
import socket
import tempfile
from unittest import mock

from absl.testing import absltest
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed
from websockets.frames import CloseCode

from mujoco.studio.web import web_server


class WebServerTest(absltest.TestCase):

  async def _open_unresponsive_connection(self, port):
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    # Complete the opening handshake but do not answer the close frame.
    writer.write(
        b"GET /state?sid=reconnect HTTP/1.1\r\n"
        b"Host: localhost\r\n"
        b"Upgrade: websocket\r\n"
        b"Connection: Upgrade\r\n"
        b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        b"Sec-WebSocket-Version: 13\r\n\r\n"
    )
    await writer.drain()
    response = await asyncio.wait_for(
        reader.readuntil(b"\r\n\r\n"), timeout=3.0
    )
    self.assertStartsWith(response, b"HTTP/1.1 101 ")
    return reader, writer

  def _run_with_server(self, test):
    with mock.patch.object(
        asyncio.BaseEventLoop, "call_exception_handler"
    ) as loop_error:
      with tempfile.TemporaryDirectory() as static_dir:
        with socket.create_server(("127.0.0.1", 0)) as sock:
          server = web_server.WebServer(sock, static_files_dir=static_dir)
          port = sock.getsockname()[1]
          server.start()
          thread = server._thread
          try:
            asyncio.run(test(server, port))
          finally:
            server.stop()
          self.assertFalse(thread.is_alive())
      gc.collect()  # Surface unhandled stream-task exceptions before checking.
      loop_error.assert_not_called()

  def test_reconnect_closes_previous_state_connection(self):
    async def test(server, port):
      url = f"ws://127.0.0.1:{port}/state?sid=reconnect"
      async with connect(url) as previous:
        self.assertIn("viewers=1;", await previous.recv())
        async with connect(url) as current:
          self.assertIn("viewers=1;", await current.recv())
          await asyncio.wait_for(previous.wait_closed(), timeout=3.0)
          with self.assertRaises(ConnectionClosed) as closed:
            await previous.recv()
          self.assertEqual(closed.exception.rcvd.code, CloseCode.GOING_AWAY)
          server.update_state(b"latest state")
          self.assertEqual(
              await asyncio.wait_for(current.recv(), timeout=3.0),
              b"latest state",
          )
          self.assertEqual(server.viewer_count, 1)

    self._run_with_server(test)

  def test_reconnect_closes_unresponsive_previous_connection(self):
    async def test(server, port):
      reader, writer = await self._open_unresponsive_connection(port)
      try:
        async with connect(
            f"ws://127.0.0.1:{port}/state?sid=reconnect"
        ) as current:
          self.assertIn(
              "viewers=1;",
              await asyncio.wait_for(current.recv(), timeout=3.0),
          )
          # read() returns only once the server closes the stale TCP socket.
          await asyncio.wait_for(reader.read(), timeout=3.0)
          server.update_state(b"latest state")
          self.assertEqual(
              await asyncio.wait_for(current.recv(), timeout=3.0),
              b"latest state",
          )
          self.assertEqual(server.viewer_count, 1)
      finally:
        writer.close()
        await writer.wait_closed()

    with mock.patch.object(web_server, "_RECONNECT_CLOSE_TIMEOUT_SEC", 0.05):
      self._run_with_server(test)

  def test_repeated_reconnect_streams_while_previous_connection_closes(self):
    async def test(server, port):
      _, writer = await self._open_unresponsive_connection(port)
      try:
        url = f"ws://127.0.0.1:{port}/state?sid=reconnect"
        async with connect(url) as intermediate:
          self.assertIn(
              "viewers=1;",
              await asyncio.wait_for(intermediate.recv(), timeout=3.0),
          )
          async with connect(url) as current:
            self.assertIn("viewers=1;", await current.recv())
            await asyncio.wait_for(intermediate.wait_closed(), timeout=3.0)
            server.update_state(b"latest state")
            self.assertEqual(
                await asyncio.wait_for(current.recv(), timeout=3.0),
                b"latest state",
            )
            self.assertEqual(server.viewer_count, 1)
      finally:
        writer.close()
        await writer.wait_closed()

    # Keep the first close handshake pending throughout the two reconnects.
    with mock.patch.object(web_server, "_RECONNECT_CLOSE_TIMEOUT_SEC", 60.0):
      self._run_with_server(test)


if __name__ == "__main__":
  absltest.main()

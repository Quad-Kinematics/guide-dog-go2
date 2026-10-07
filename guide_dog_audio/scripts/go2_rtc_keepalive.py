"""Holds a WebRTC session to the Go2 open so its speaker plays whole clips.

The Go2 audio hub stops playback after ~0.3 s unless a WebRTC client is
connected (tested 2026-10-05: the same 10 s clip stopped at 0.25 s without a
session and played in full with one). tts_node plays over DDS, so this process
only keeps a session up; it sends nothing over it.

Needs Python >= 3.10 with go2_webrtc_driver (go2-webrtc-connect). Start it with
the go2_rtc_keepalive wrapper, which picks that interpreter.

The Go2 accepts one WebRTC client at a time: while this runs the Unitree app
cannot connect, and if the app takes the session this reconnects after
RETRY_SEC.

While the session is open it refreshes STATUS_FILE every CHECK_SEC, and
deletes it when the session drops. tts_node waits for a fresh file before
playing, so a clip is not cut off while the session is still connecting.
"""

import asyncio
import contextlib
import logging
import os
import pathlib

from go2_webrtc_driver.constants import WebRTCConnectionMethod
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection

GO2_IP = os.environ.get('GO2_IP', '192.168.123.161')
CHECK_SEC = 2.0
RETRY_SEC = 5.0
# Must match tts_node's rtc_status_file parameter
STATUS_FILE = os.environ.get(
    'GO2_RTC_STATUS_FILE', os.path.join(os.path.expanduser('~'), '.ros', 'go2_rtc_keepalive.up'))

logging.getLogger().setLevel(logging.CRITICAL)  # the driver logs every state change


def log(text):
    print(f'[go2_rtc_keepalive] {text}', flush=True)


def is_up(conn):
    # isConnected is only cleared on "closed", not on "failed"/"disconnected"
    return conn.pc is not None and conn.pc.connectionState in ('connecting', 'connected')


def mark_up():
    os.makedirs(os.path.dirname(STATUS_FILE), exist_ok=True)
    pathlib.Path(STATUS_FILE).touch()  # tts_node reads only the modification time


def mark_down():
    with contextlib.suppress(FileNotFoundError):
        os.remove(STATUS_FILE)


async def main():
    while True:
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=GO2_IP)
        try:
            await conn.connect()
            log(f'WebRTC session to {GO2_IP} open')
            while is_up(conn):
                mark_up()
                await asyncio.sleep(CHECK_SEC)
            log('WebRTC session lost; reconnecting')
        except Exception as exc:  # the driver raises plain Exceptions on connect errors
            log(f'connect to {GO2_IP} failed: {exc}')
        finally:
            mark_down()
            with contextlib.suppress(Exception):
                await conn.disconnect()
        await asyncio.sleep(RETRY_SEC)


if __name__ == '__main__':
    try:
        with contextlib.suppress(KeyboardInterrupt):
            asyncio.run(main())
    finally:
        mark_down()

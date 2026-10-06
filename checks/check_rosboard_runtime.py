#!/usr/bin/env python3
"""Check the observer's HTTP endpoint and live browser-compatible joint feedback."""

import argparse
import asyncio
import json
from urllib.request import urlopen

from tornado.websocket import websocket_connect


async def check(port):
    with urlopen(f"http://127.0.0.1:{port}/", timeout=10) as response:
        assert response.status == 200
    ws = await websocket_connect(f"ws://127.0.0.1:{port}/rosboard/v1", connect_timeout=10)
    try:
        ws.write_message(json.dumps(["s", {"topicName": "/joint_states", "maxUpdateRate": 5}]))
        async with asyncio.timeout(20):
            while True:
                raw = await ws.read_message()
                assert raw is not None, "Rosboard closed the connection"
                message = json.loads(
                    raw,
                    parse_constant=lambda value: (_ for _ in ()).throw(
                        ValueError(f"Invalid JSON constant: {value}")
                    ),
                )
                if message[0] == "p":
                    ws.write_message(json.dumps(["q", message[1]]))
                if message[0] == "m" and message[1].get("_topic_name") == "/joint_states":
                    sample = message[1]
                    assert len(sample["name"]) == 22, sample["name"]
                    assert len(sample["position"]) == 22
                    assert all(isinstance(value, (int, float)) for value in sample["position"])
                    print("PASS: rosboard HTTP/WebSocket streams all 22 joints as valid JSON.")
                    return
    finally:
        ws.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8888)
    asyncio.run(check(parser.parse_args().port))

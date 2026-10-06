#!/usr/bin/env python3
"""Missing effort and non-finite sensor values must remain browser-readable JSON."""

import json
from rosboard.handlers import json_safe

sample = [
    "m",
    {
        "_topic_name": "/joint_states",
        "position": [0.1, 0.2],
        "effort": [float("nan"), float("inf")],
        "nested": {"value": float("-inf")},
    },
]
encoded = json.dumps(json_safe(sample), allow_nan=False)
decoded = json.loads(encoded)
assert decoded[1]["position"] == [0.1, 0.2]
assert decoded[1]["effort"] == [None, None]
assert decoded[1]["nested"]["value"] is None
assert sample[1]["effort"][1] == float("inf")
print("PASS: unknown effort/non-finite values serialize as null without changing valid feedback.")

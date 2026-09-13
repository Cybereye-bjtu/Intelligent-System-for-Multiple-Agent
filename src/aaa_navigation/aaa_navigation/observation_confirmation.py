"""Independent multi-frame confirmation histories for detection query streams."""

from __future__ import annotations

import math


class ConfirmationHistory:
    """Keep bounded position histories isolated by stream and prompt."""

    def __init__(self):
        self._values = {}

    def clear(self, stream=None):
        if stream is None:
            self._values.clear()
            return
        for key in [key for key in self._values if key[0] == stream]:
            del self._values[key]

    def add(self, stream, prompt, observation, *, maximum_count, distance_limit):
        key = (str(stream), str(prompt))
        values = self._values.setdefault(key, [])
        if values:
            previous = values[-1]["position"]
            current = observation["position"]
            distance = math.sqrt(
                sum((current[index] - previous[index]) ** 2 for index in range(3))
            )
            if distance > float(distance_limit):
                values.clear()
        values.append(observation)
        del values[:-max(1, int(maximum_count))]
        return values

    def count(self, stream, prompt):
        return len(self._values.get((str(stream), str(prompt)), ()))

    def keys(self):
        return self._values.keys()

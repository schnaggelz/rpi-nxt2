#!/usr/bin/env python
#
# Utility code
#

class FpsCalculator:
    def __init__(self, alpha=0.1):
        self.t = time.time()
        self.alpha = alpha
        self.sfps = None

    def __call__(self):
        t = time.time()
        d = t - self.t
        self.t = t
        fps = 1.0 / d
        if self.sfps is None:
            self.sfps = fps
        else:
            self.sfps = fps * self.alpha + self.sfps * (1.0 - self.alpha)
        return self.sfps

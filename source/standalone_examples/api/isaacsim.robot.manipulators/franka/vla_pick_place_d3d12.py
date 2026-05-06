# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# D3D12 wrapper for the observer-only VLA calibration experiment.

from __future__ import annotations

import os
import runpy

os.environ["ISAACSIM_PYTHON_EXPERIENCE"] = os.path.join(
    os.environ["EXP_PATH"],
    "isaacsim.exp.base.python.d3d12.kit",
)

if __name__ == "__main__":
    runpy.run_path(
        os.path.join(
            os.path.dirname(__file__),
            "vla_pick_place.py",
        ),
        run_name="__main__",
    )

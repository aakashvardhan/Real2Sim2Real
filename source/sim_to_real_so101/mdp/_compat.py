# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Normalizes Isaac Lab asset/sensor `.data.*` outputs to torch.Tensor.

Isaac Lab 2.x returns torch.Tensor from `.data.*` properties; Isaac Lab 3.0
returns warp.array instead. This keeps the mdp package working unmodified
on either version.
"""
import torch


def as_torch(x):
    if isinstance(x, torch.Tensor):
        return x
    try:
        import warp as wp

        if isinstance(x, wp.array):
            return wp.to_torch(x)
    except ImportError:
        pass
    return x

# Copyright 2026 The MuJoCo Metal contributors
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

"""Machine-readable pinned-version support inventory; import is host-only."""

from dataclasses import dataclass
from enum import Enum

import mujoco

TARGET_MUJOCO_VERSION = "3.10.0"
SOURCE_TREE_VERSION = "3.14.1 (not a target)"


class Stage(str, Enum):
    MODEL = "model"
    KINEMATICS = "kinematics"
    DYNAMICS = "dynamics"
    COLLISION = "collision"
    CONSTRAINTS = "constraints"
    INTEGRATION = "integration"
    SENSOR = "sensor"
    RENDERING = "rendering"
    API = "api"


class Implementation(str, Enum):
    LOWERED = "lowered"
    CPU_REFERENCE = "cpu_reference"
    NATIVE_GPU = "native_gpu"
    UPSTREAM_CPU_ONLY = "upstream_cpu_only"
    NOT_IMPLEMENTED = "not_implemented"


class Qualification(str, Enum):
    UNQUALIFIED = "unqualified"
    CPU_ORACLE = "cpu_oracle"
    GPU_QUALIFIED = "gpu_qualified"


class Execution(str, Enum):
    HOST = "host"
    DEVICE = "device"
    UPSTREAM_HOST = "upstream_host"
    NONE = "none"


@dataclass(frozen=True)
class Feature:
    name: str
    stage: Stage
    implementation: Implementation
    qualification: Qualification
    execution: Execution
    limitation: str


# Every member is an explicit inventory row, but remains unsupported until the
# corresponding stage gains an implementation and qualification record.
_ENUMS = (
    "mjtJoint",
    "mjtGeom",
    "mjtIntegrator",
    "mjtCone",
    "mjtJacobian",
    "mjtSolver",
    "mjtEq",
    "mjtTrn",
    "mjtDyn",
    "mjtGain",
    "mjtBias",
    "mjtSensor",
    "mjtState",
    "mjtDisableBit",
    "mjtEnableBit",
)


def _enum_inventory():
    rows = []
    for enum_name in _ENUMS:
        enum_type = getattr(mujoco, enum_name, None)
        if enum_type is None:
            rows.append(
                Feature(
                    f"enum:{enum_name}",
                    Stage.MODEL,
                    Implementation.NOT_IMPLEMENTED,
                    Qualification.UNQUALIFIED,
                    Execution.NONE,
                    "not exposed by pinned Python bindings",
                )
            )
            continue
        for member_name in enum_type.__members__:
            rows.append(
                Feature(
                    f"{enum_name}.{member_name}",
                    Stage.MODEL,
                    Implementation.NOT_IMPLEMENTED,
                    Qualification.UNQUALIFIED,
                    Execution.NONE,
                    "inventory only; not implemented by this package",
                )
            )
    return rows


_API_FAMILIES = (
    "mj_forward/mj_step/mj_step1/mj_step2",
    "mj_inverse/mj_compareFwdInv",
    "mj_resetData/mj_resetDataKeyframe/mj_copyData",
    "mj_getState/mj_setState",
    "mj_fullM/mj_mulM/mj_solveM/mj_factorM",
    "mj_jac/mj_jacBody/mj_jacSite/mj_jacGeom",
    "mj_ray/mj_ray flex and geom query APIs",
    "mj_energyPos/mj_energyVel",
    "mj_sensorAcc/mj_objectVelocity/mj_objectAcceleration",
    "mj_addPlugin/mj_plugin APIs",
    "mjv_* visualization and renderer APIs",
    "mj_saveModel/mj_loadModel/mj_printModel",
)


def _inventory():
    result = [
        Feature(
            "compiled model lowering",
            Stage.MODEL,
            Implementation.LOWERED,
            Qualification.CPU_ORACLE,
            Execution.HOST,
            "requires MuJoCo 3.10.0",
        )
    ]
    result.append(
        Feature(
            "generic CPU joint FK",
            Stage.KINEMATICS,
            Implementation.CPU_REFERENCE,
            Qualification.CPU_ORACLE,
            Execution.HOST,
            "kinematics only; no physics stepping",
        )
    )
    result.append(
        Feature(
            "generic Metal joint FK",
            Stage.KINEMATICS,
            Implementation.NATIVE_GPU,
            Qualification.UNQUALIFIED,
            Execution.DEVICE,
            "not GPU-qualified; no physics stepping",
        )
    )
    result.extend(_enum_inventory())
    result.extend(
        Feature(
            f"api:{name}",
            Stage.API,
            Implementation.UPSTREAM_CPU_ONLY,
            Qualification.UNQUALIFIED,
            Execution.UPSTREAM_HOST,
            "not exposed as a Metal API",
        )
        for name in _API_FAMILIES
    )
    result.extend(
        Feature(
            f"python-api:{name}",
            Stage.API,
            Implementation.UPSTREAM_CPU_ONLY,
            Qualification.UNQUALIFIED,
            Execution.UPSTREAM_HOST,
            "upstream Python binding only; not exposed as a Metal API",
        )
        for name in sorted(dir(mujoco))
        if name.startswith(("mj_", "mju_")) and callable(getattr(mujoco, name))
    )
    result.extend(
        (
            Feature(
                "collision pipeline",
                Stage.COLLISION,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device collisions",
            ),
            Feature(
                "constraint assembly and solvers",
                Stage.CONSTRAINTS,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device constraints",
            ),
            Feature(
                "actuator evaluation",
                Stage.DYNAMICS,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device actuation",
            ),
            Feature(
                "forward/inverse dynamics",
                Stage.DYNAMICS,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device dynamics",
            ),
            Feature(
                "integrators/stepping",
                Stage.INTEGRATION,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device stepping",
            ),
            Feature(
                "built-in sensors",
                Stage.SENSOR,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device sensors",
            ),
            Feature(
                "deformables and plugins",
                Stage.DYNAMICS,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no device deformables or plugins",
            ),
            Feature(
                "rendering",
                Stage.RENDERING,
                Implementation.NOT_IMPLEMENTED,
                Qualification.UNQUALIFIED,
                Execution.NONE,
                "no Metal renderer",
            ),
        )
    )
    return tuple(result)


FEATURES = _inventory()
INVENTORY_COMPLETE = False


def feature_status():
    """Return the immutable pinned-version inventory without loading Torch."""
    return FEATURES

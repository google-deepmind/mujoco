"""Utility helpers for working with MuJoCo's ``mj_geomDistance``.

MuJoCo returns a *signed* surface–to–surface distance where *positive* means the
shapes are separated, *negative* means the *effective* contact surfaces are
already interpenetrating.  The effective surfaces are the real geometry
surfaces **expanded by their individual contact margins** (``geom_margin``).

If you want the *raw* geometric gap – i.e. the distance between the actual
surfaces with **no margins** – you must add the margins back yourself, or set
those margins to zero before the call.  These helper functions make that easy
and add a tiny batch wrapper for better performance when you need many
distances in one simulation step.
"""
from __future__ import annotations

from typing import Iterable, Sequence, Tuple, Union
import numpy as np
import mujoco

# -----------------------------------------------------------------------------
# Internal helpers
# -----------------------------------------------------------------------------

def _ensure_fromto(fromto: Union[np.ndarray, None]) -> np.ndarray:
    """Return an appropriately shaped *fromto* array.

    The caller can supply *None* to avoid allocations; this function will create
    a new ``np.zeros(6)`` array that is compatible with ``mj_geomDistance``.
    """
    if fromto is None:
        return np.zeros(6, dtype=np.float64)
    return fromto

# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

def sdf(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gid1: int,
    gid2: int,
    maxdist: float = 20.0,
    fromto: Union[np.ndarray, None] = None,
) -> float:
    """Return MuJoCo's *signed* distance (includes margins).

    This is a very thin convenience wrapper around ``mj_geomDistance`` that just
    hides the *fromto* pre-allocation boilerplate.
    """
    fromto_arr = _ensure_fromto(fromto)
    return mujoco.mj_geomDistance(model, data, int(gid1), int(gid2), maxdist, fromto_arr)


def raw_gap(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gid1: int,
    gid2: int,
    maxdist: float = 20.0,
    fromto: Union[np.ndarray, None] = None,
) -> float:
    """Surface-to-surface distance *without* the geom margins.

    This is often what researchers expect when comparing signed distances from
    other geometry libraries.  Internally MuJoCo returns ``sdf - m1 - m2`` so
    we simply add the two margins back.
    """
    fromto_arr = _ensure_fromto(fromto)
    sdf_dist = mujoco.mj_geomDistance(model, data, int(gid1), int(gid2), maxdist, fromto_arr)
    return sdf_dist + model.geom_margin[int(gid1)] + model.geom_margin[int(gid2)]


def batch_raw_gap(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gid_pairs: Sequence[Tuple[int, int]],
    maxdist: float = 20.0,
) -> np.ndarray:
    """Vectorised **raw** gaps for many geom pairs.

    Parameters
    ----------
    model, data
        Standard MuJoCo structures.
    gid_pairs
        Iterable of ``(gid1, gid2)`` integer pairs.
    maxdist
        Max distance fed to MuJoCo (see ``mj_geomDistance`` docs).

    Returns
    -------
    np.ndarray[float]
        Array of raw gaps of shape ``(len(gid_pairs),)``.
    """
    n = len(gid_pairs)
    gaps = np.empty(n, dtype=np.float64)

    if n == 0:
        return gaps

    fromto = np.zeros((n, 6), dtype=np.float64)

    # We keep the loop in Python but avoid repeated allocations inside the loop.
    # MuJoCo's collision routine is C-level, so the Python overhead here is
    # usually negligible compared with the kernel inside mj_geomDistance.
    for i, (g1, g2) in enumerate(gid_pairs):
        gaps[i] = mujoco.mj_geomDistance(
            model, data, int(g1), int(g2), maxdist, fromto[i]
        ) + model.geom_margin[int(g1)] + model.geom_margin[int(g2)]

    return gaps

# -----------------------------------------------------------------------------
# Example usage (run as a script)
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    import pathlib, sys

    if len(sys.argv) != 2:
        print("Usage: python geom_distance_utils.py <model.xml>")
        sys.exit(1)

    xml_path = pathlib.Path(sys.argv[1])
    if not xml_path.exists():
        raise FileNotFoundError(xml_path)

    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)

    # Just pick two geoms so the example runs without knowing the model.
    gid1, gid2 = 0, 1

    # Forward pass to position geoms.
    mujoco.mj_forward(m, d)

    print("Signed distance (includes margins):", sdf(m, d, gid1, gid2))
    print("Raw gap (no margins):           ", raw_gap(m, d, gid1, gid2))

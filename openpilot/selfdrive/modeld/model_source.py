"""Where the big model and its precompiled artifacts are served from.

Kept in its own file so switching mirrors does not require touching big_model.py:
only this one small module changes.

precompiled.json and firmware/amdgpu are resolved relative to the model URL (see
precompiled_model.ensure_precompiled / firmware.ensure_firmware), so keep the
model file inside <base>/models/.
"""

MODEL_BASE = "http://op.gitop.vip:82/egpu"

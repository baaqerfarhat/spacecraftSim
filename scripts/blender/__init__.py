#!/usr/bin/env -S blender --python-exit-code 1 --python
"""
Script for setting up Blender preferences.

- Use Cycles with CUDA GPU
- Enable Node Wrangler addon
"""

import bpy

# Use Cycles with GPU
bpy.context.preferences.addons["cycles"].preferences.compute_device_type = "CUDA"
bpy.context.preferences.addons["cycles"].preferences.get_devices()
for device in bpy.context.preferences.addons["cycles"].preferences.devices:
    device.use = device.type == "CUDA"
bpy.context.scene.render.engine = "CYCLES"
bpy.context.scene.cycles.device = "GPU"

# Enable Node Wrangler
bpy.ops.preferences.addon_enable(module="node_wrangler")

# Save preferences
bpy.ops.wm.save_userpref()

import omni.usd

# Get stage
stage = omni.usd.get_context().get_stage()

# Replace with your true Venus Express prim path
camera_prim = stage.GetPrimAtPath("/World/envs/env_0/venus_express/base/camera_onboard")
print("Camera prim:", camera_prim)

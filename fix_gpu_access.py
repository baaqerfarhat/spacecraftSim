#!/usr/bin/env python3

"""
FIX GPU ACCESS IN CONTAINER
===========================

Diagnose and fix GPU access issues in Docker container.
The RTX 8000 hardware exists but container can't access it.
"""

import subprocess
import os
import sys

def check_gpu_hardware():
    """Check if GPU hardware is available on host"""
    print("🖥️  CHECKING GPU HARDWARE...")
    print("="*40)
    
    try:
        # Try nvidia-smi first
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("✅ nvidia-smi working!")
            print("📊 GPU info:")
            lines = result.stdout.split('\n')[:15]  # First 15 lines
            for line in lines:
                if line.strip():
                    print(f"   {line}")
            return True
        else:
            print("❌ nvidia-smi failed")
            print(f"Error: {result.stderr}")
            return False
            
    except FileNotFoundError:
        print("❌ nvidia-smi not found in container")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def check_cuda_libraries():
    """Check if CUDA libraries are available"""
    print("\n🔧 CHECKING CUDA LIBRARIES...")
    print("="*40)
    
    cuda_paths = [
        '/usr/local/cuda/lib64/libcuda.so',
        '/usr/lib/x86_64-linux-gnu/libcuda.so.1',
        '/usr/local/cuda/lib64/libcuda.so.1',
        '/lib/x86_64-linux-gnu/libcuda.so.1'
    ]
    
    found_cuda = False
    for path in cuda_paths:
        if os.path.exists(path):
            print(f"✅ Found CUDA library: {path}")
            found_cuda = True
        else:
            print(f"❌ Missing: {path}")
    
    return found_cuda

def check_container_gpu_setup():
    """Check container GPU environment"""
    print("\n🐳 CHECKING CONTAINER GPU SETUP...")
    print("="*40)
    
    # Check environment variables
    gpu_envs = [
        'NVIDIA_VISIBLE_DEVICES',
        'CUDA_VISIBLE_DEVICES', 
        'NVIDIA_DRIVER_CAPABILITIES',
        'NVIDIA_REQUIRE_CUDA'
    ]
    
    print("🔍 GPU Environment variables:")
    for env in gpu_envs:
        value = os.environ.get(env, 'Not set')
        status = "✅" if value != 'Not set' else "❌"
        print(f"   {status} {env}: {value}")
    
    # Check if running in container
    in_container = os.path.exists('/.dockerenv')
    print(f"\n🐳 Running in container: {in_container}")
    
    if in_container:
        print("📋 Container detected - this explains the GPU access issues!")
    
    return in_container

def check_nvidia_runtime():
    """Check if NVIDIA container runtime is available"""
    print("\n🔧 CHECKING NVIDIA CONTAINER RUNTIME...")
    print("="*40)
    
    try:
        # Check for nvidia-container-runtime
        result = subprocess.run(['which', 'nvidia-container-runtime'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"✅ nvidia-container-runtime found: {result.stdout.strip()}")
            return True
        else:
            print("❌ nvidia-container-runtime not found")
            return False
            
    except Exception as e:
        print(f"❌ Error checking runtime: {e}")
        return False

def provide_gpu_fixes():
    """Provide step-by-step GPU access fixes"""
    print("\n🔧 GPU ACCESS FIXES")
    print("="*50)
    
    print("Your RTX 8000 hardware exists but container can't access it!")
    print()
    print("📋 SOLUTION 1: Restart container with GPU access")
    print("   Stop current container and restart with:")
    print("   docker run --gpus all --rm -it \\")
    print("     -v /tmp/.X11-unix:/tmp/.X11-unix \\")
    print("     -e DISPLAY=$DISPLAY \\")
    print("     <your_isaac_sim_image>")
    print()
    print("📋 SOLUTION 2: If using Docker Compose")
    print("   Add to your docker-compose.yml:")
    print("   services:")
    print("     isaac-sim:")
    print("       runtime: nvidia")
    print("       environment:")
    print("         - NVIDIA_VISIBLE_DEVICES=all")
    print("         - NVIDIA_DRIVER_CAPABILITIES=all")
    print()
    print("📋 SOLUTION 3: Use headless mode (no GPU rendering)")
    print("   srb agent ros -e orbital_evasion env.robot=lab_sc --headless")
    print("   (This works without GPU for physics simulation)")
    print()
    print("📋 SOLUTION 4: Install NVIDIA container toolkit")
    print("   On host machine:")
    print("   curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | sudo apt-key add -")
    print("   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)")
    print("   curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list")
    print("   sudo apt-get update && sudo apt-get install -y nvidia-container-runtime")

def quick_test_headless():
    """Test if headless mode would work"""
    print("\n🧪 TESTING HEADLESS OPTION...")
    print("="*40)
    
    print("💡 Headless mode bypasses GPU rendering issues")
    print("📊 It can still run physics simulation and ROS interface")
    print("🎯 Perfect for IMU data and spacecraft control!")
    print()
    print("🚀 Try this command:")
    print("   srb agent ros -e orbital_evasion env.robot=lab_sc --headless")
    print()
    print("✅ Benefits of headless mode:")
    print("   - No GPU rendering (avoids current errors)")
    print("   - Physics simulation still works") 
    print("   - ROS topics still publish")
    print("   - IMU data can be generated")
    print("   - Perfect for automated control testing")

def main():
    print("🛰️  GPU ACCESS DIAGNOSTICS")
    print("="*50)
    print("Diagnosing why RTX 8000 GPU is not accessible...")
    print("="*50)
    
    # Run all diagnostics
    gpu_hw = check_gpu_hardware()
    cuda_libs = check_cuda_libraries()
    container = check_container_gpu_setup()
    nvidia_runtime = check_nvidia_runtime()
    
    print(f"\n📊 DIAGNOSTIC SUMMARY:")
    print(f"GPU Hardware (nvidia-smi): {'✅' if gpu_hw else '❌'}")
    print(f"CUDA Libraries: {'✅' if cuda_libs else '❌'}")
    print(f"Container Environment: {'✅ Detected' if container else '❌'}")
    print(f"NVIDIA Runtime: {'✅' if nvidia_runtime else '❌'}")
    
    # Provide diagnosis
    if not gpu_hw and container:
        print(f"\n🎯 DIAGNOSIS: CONTAINER GPU ACCESS ISSUE")
        print(f"✅ You're in a Docker container")
        print(f"❌ Container doesn't have GPU access") 
        print(f"💡 RTX 8000 hardware exists but container can't see it")
        provide_gpu_fixes()
    elif gpu_hw:
        print(f"\n🎉 GPU ACCESS WORKING!")
        print(f"Something else is causing Isaac Sim issues")
    else:
        print(f"\n❌ GPU NOT ACCESSIBLE")
        print(f"Need to fix container GPU setup")
        provide_gpu_fixes()
    
    # Always suggest headless as immediate solution
    quick_test_headless()

if __name__ == '__main__':
    main()

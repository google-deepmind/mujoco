import os
import ctypes

def test_mujoco_library():
    """
    Validates whether the MuJoCo shared library exists and can be loaded.
    Update the 'lib_path' according to your operating system.
    """
    # 👉 Change this path based on your OS
    lib_path = "dist/bin/libmujoco.so"       # For Linux
    # lib_path = "dist/bin/mujoco.dll"       # For Windows
    # lib_path = "dist/bin/libmujoco.dylib"  # For macOS

    if not os.path.exists(lib_path):
        raise FileNotFoundError(f"Shared library not found at: {lib_path}")

    try:
        ctypes.CDLL(lib_path)
        print(f"✅ MuJoCo library loaded successfully from: {lib_path}")
    except OSError as e:
        print("❌ Failed to load MuJoCo library")
        raise e

if __name__ == "__main__":
    test_mujoco_library()

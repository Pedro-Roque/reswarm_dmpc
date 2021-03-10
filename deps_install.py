import subprocess
import sys

def install(package):
    print(subprocess.check_call([sys.executable, "-m", "pip", "install", package]))

if __name__ == "__main__":
    install("matplotlib")
    install("casadi")
    install("numpy")
    install("control")
    install("scipy")
    install("unittest-parallel")

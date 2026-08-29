import time
import gc
import psutil
import os
import atsonic_ungc

def print_mem(label):
    process = psutil.Process(os.getpid())
    mb = process.memory_info().rss / (1024 * 1024)
    print(f"[{label}] Memory Usage: {mb:.2f} MB")

print("=== ATSONIC UNGC STRESS TEST (10,000,000 STEPS) ===")
print_mem("Kabla ya Kuanza")

# Pre-allocate engine capacity
engine = atsonic_ungc.AtsonicEngine(capacity=10_000_000)
gc.disable()

print_mem("Baada ya Engine Pre-allocation")

start = time.time()
for i in range(10_000_000):
    # Simulation workload
    _ = [x * 2 for x in range(20)]
    
    # Check memory kila baada ya steps 2M
    if (i + 1) % 2_000_000 == 0:
        print_mem(f"Step {i+1:,}")

end = time.time()
print(f"\nUtekelezaji Umekamilika kwa Sec {end - start:.2f}")
print_mem("Baada ya Mwisho wa Mjaribio")

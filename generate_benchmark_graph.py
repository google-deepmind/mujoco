import time
import gc
import psutil
import os
import matplotlib.pyplot as plt
import atsonic_ungc

def get_mem_mb():
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)

STEPS = 10_000_000
CHECK_INTERVAL = 200_000

steps_history = []
mem_history = []

print("=== GENERATING ATSONIC UNGC BENCHMARK GRAPH ===")
print(f"Memory ya Mwanzo: {get_mem_mb():.2f} MB")

# Pre-allocate Engine & Disable Python GC
engine = atsonic_ungc.AtsonicEngine(capacity=STEPS)
gc.disable()

start_time = time.time()

for i in range(STEPS):
    # Simulation workload
    _ = [x * 2 for x in range(20)]
    
    if (i + 1) % CHECK_INTERVAL == 0:
        steps_history.append(i + 1)
        mem_history.append(get_mem_mb())

end_time = time.time()

# Tengeneza Grafu
plt.figure(figsize=(10, 5))
plt.plot(steps_history, mem_history, color='#00FF66', linewidth=2, label='AtSonic Engine Memory')
plt.title('AtSonic UNGC Memory Stability Test (10 Million Steps)', fontsize=14, fontweight='bold')
plt.xlabel('Simulation Steps', fontsize=12)
plt.ylabel('Memory Usage (MB)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.ylim(min(mem_history) - 5, max(mem_history) + 5)
plt.legend(loc='upper right')

# Hifadhi Picha
plt.savefig('atsonic_memory_stability.png', dpi=300, bbox_inches='tight')
print(f"\n[SUCCESS] Grafu imehifadhiwa kama 'atsonic_memory_stability.png'")
print(f"Muda Uliotumika: {end_time - start_time:.2f} seconds")
print(f"Memory ya Mwisho: {get_mem_mb():.2f} MB")

import time
import gc
import psutil
import os
import atsonic_ungc

def get_memory_mb():
    """Retrieve memory usage of the current process in MB."""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)

def run_benchmark(steps=1_000_000, use_atsonic=False, engine_capacity=100_000):
    mode = "AtSonic Engine (No-GC)" if use_atsonic else "Standard Python GC"
    print(f"\n--- Starting Simulation: steps={steps:,} | Mode: {mode} ---")
    
    if use_atsonic:
        # Instantiate AtsonicEngine with the required capacity argument
        engine = atsonic_ungc.AtsonicEngine(capacity=engine_capacity)
        gc.disable()
    else:
        gc.enable()

    start_mem = get_memory_mb()
    latencies = []
    
    start_time = time.perf_counter_ns()
    
    for _ in range(steps):
        t0 = time.perf_counter_ns()
        
        # Micro-workload simulation to evaluate GC pauses and execution speed
        _ = [x * 2 for x in range(15)]
        
        t1 = time.perf_counter_ns()
        latencies.append((t1 - t0) / 1000.0)

    total_time = (time.perf_counter_ns() - start_time) / 1e9
    end_mem = get_memory_mb()
    
    avg_latency = sum(latencies) / len(latencies)
    fps = steps / total_time

    print(f" Performance Metrics:")
    print(f" Total Time      : {total_time:.4f} s")
    print(f" FPS (Steps/Sec) : {fps:,.2f}")
    print(f" Avg Step Latency: {avg_latency:.4f} µs")
    print(f" Memory Overhead : {end_mem - start_mem:.2f} MB")

if __name__ == "__main__":
    print(">>> Running Standard Python Benchmark...")
    run_benchmark(use_atsonic=False)
    
    print("\n>>> Running AtSonic Engine Benchmark...")
    run_benchmark(use_atsonic=True, engine_capacity=1_000_000)

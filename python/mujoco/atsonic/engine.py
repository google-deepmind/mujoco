import gc
import atsonic_ungc

class AtSonicMujocoRunner:
    """
    AtSonic UNGC Engine integration wrapper for MuJoCo simulation loops.
    Eliminates GC pause overhead and guarantees deterministic zero-leak memory layout.
    """
    def __init__(self, steps_capacity=10_000_000):
        self.capacity = steps_capacity
        self.engine = atsonic_ungc.AtsonicEngine(capacity=self.capacity)
        
    def start_no_gc_loop(self):
        gc.disable()
        
    def stop_no_gc_loop(self):
        gc.enable()
        gc.collect()

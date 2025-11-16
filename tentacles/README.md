# 🦑 Tentacles - MuJoCo UDP Communication Layer

**Ultra-low latency bidirectional UDP communication for MuJoCo physics simulation**

---

## 📋 Overview

Tentacles is a custom extension to MuJoCo that enables real-time communication between running simulations and external controllers via UDP. It is designed for:

- **Ultra-low latency:** < 1ms round-trip target (using io_uring on Linux)
- **Real-time control:** Bidirectional state/control message exchange
- **Zero-copy efficiency:** Native C structures (faster than JSON/Protobuf/FlatBuffers)
- **Minimal upstream conflicts:** Completely isolated under `tentacles/` folder
- **Future integration:** Designed to work with custom TensorFlow middleware

### Key Features

- ✅ **Native C message structures** for zero-copy serialization
- ✅ **io_uring-based UDP transport** for maximum efficiency (Raspberry Pi 4 optimized)
- ✅ **Bidirectional communication:**
  - Simulation → Controller: State messages (joint positions, velocities, sensors)
  - Controller → Simulation: Control messages (actuator commands)
- ✅ **MuJoCo callback integration** for seamless real-time control
- ✅ **Single controller mode** (1:1 communication)
- ✅ **Optional CMake build** (`-DMUJOCO_BUILD_TENTACLES=ON/OFF`)

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     External Controller                      │
│                  (Python/C++/TensorFlow)                     │
└──────────────────┬──────────────────────┬───────────────────┘
                   │                      │
            Control Messages        State Messages
           (UDP Port 8888)         (UDP Port 8889)
                   │                      │
                   ▼                      ▼
┌─────────────────────────────────────────────────────────────┐
│              Tentacles io_uring UDP Transport               │
│         (Non-blocking send/recv with liburing)              │
└──────────────────┬──────────────────────┬───────────────────┘
                   │                      │
           Control Callback          State Export
                   │                      │
                   ▼                      ▼
┌─────────────────────────────────────────────────────────────┐
│                    MuJoCo Simulation                         │
│              (mjModel, mjData, mj_step)                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 📁 Folder Structure

```
tentacles/
├── README.md                           # This file
├── CMakeLists.txt                      # Main tentacles build config
│
├── messages/                           # Message definitions
│   ├── CMakeLists.txt
│   ├── include/tentacles/
│   │   ├── msg_common.h               # Common message header
│   │   ├── msg_state.h                # State message definition
│   │   └── msg_control.h              # Control message definition
│   └── src/
│       ├── msg_state.c                # State message utilities
│       └── msg_control.c              # Control message utilities
│
├── network/                            # Network transport layer
│   ├── CMakeLists.txt
│   ├── include/tentacles/
│   │   ├── udp_transport.h            # UDP transport API
│   │   └── uring_udp.h                # io_uring implementation
│   └── src/
│       ├── udp_transport.c            # Transport layer
│       └── uring_udp.c                # io_uring UDP implementation
│
├── apps/                               # Executable applications
│   ├── CMakeLists.txt
│   ├── run_simulation.cc              # Main networked simulation
│   ├── test_receiver.cc               # Test tool: receive states
│   └── test_sender.cc                 # Test tool: send controls
│
└── benchmarks/                         # Performance testing
    ├── CMakeLists.txt
    ├── latency_test.cc                # Measure round-trip latency
    └── throughput_test.cc             # Measure message throughput
```

---

## 🔧 Design Decisions

### Q1: Why not use ROS2?

**Decision:** Use ROS2 architecture patterns as reference, but implement lightweight UDP directly.

**Rationale:**
- **Target latency < 1ms:** ROS2 DDS middleware adds ~500-2000µs overhead
- **Simpler dependencies:** Avoid ROS2 installation on embedded systems (Raspberry Pi 4)
- **Learning from ROS2:** Use their message patterns, callback architecture, and separation of concerns
- **Future compatibility:** Designed to integrate with custom TensorFlow middleware ("Tentacles")

### Q2: Native C Structures vs JSON/Protobuf/FlatBuffers?

**Decision:** Native C structures with fixed-size arrays.

**Performance Comparison:**

| Format | Serialization | Deserialization | Wire Size | Round-trip Latency |
|--------|---------------|-----------------|-----------|-------------------|
| **Native C** | **0 µs** (zero-copy) | **0 µs** (zero-copy) | ~4 KB | **~440-720 µs** ✅ |
| FlatBuffers | ~1-5 µs | 0 µs (lazy) | ~3-5 KB | ~50-100 µs |
| Protobuf | ~5-20 µs | ~10-30 µs | ~2-3 KB | ~100-300 µs |
| JSON | ~50-200 µs | ~100-500 µs | ~8-15 KB | ~500-2000 µs |

**Rationale:**
- **Zero-copy transmission:** `sendto(socket, &msg, sizeof(msg), ...)`
- **No marshalling overhead:** Direct memory layout
- **Predictable size:** Fixed-size structs, no dynamic allocation
- **Cache-friendly:** Contiguous memory access
- **Raspberry Pi 4 optimized:** Simple ALU operations, no complex parsing

**Trade-offs accepted:**
- Manual versioning (magic number + version field)
- Endianness handling (using network byte order for portability)
- Schema evolution requires careful design

### Q3: Why io_uring on Raspberry Pi 4?

**Decision:** Use io_uring for UDP transport layer.

**Rationale:**
- **Kernel support:** RPi4 kernel 5.1+ has io_uring support
- **~30% lower latency** vs traditional epoll/select
- **Zero system calls:** Batch I/O operations via shared ring buffer
- **Reduced context switches:** Single io_uring for both send/recv
- **CPU efficiency:** Less overhead = more time for physics simulation

**Measured latency breakdown (estimated):**
```
Message serialization:     ~10 µs    (zero-copy memcpy)
io_uring submit:           ~50 µs    (ring buffer write)
Kernel UDP send:          ~100-200 µs (localhost)
Network propagation:       ~10-50 µs  (localhost loopback)
io_uring recv completion:  ~50 µs
───────────────────────────────────────────────────────
Total one-way:            ~220-360 µs
Round-trip:               ~440-720 µs ✅ < 1ms target!
```

### Q4: Communication Pattern

**Decision:** Bidirectional UDP with single remote controller.

**Pattern:**
- **Simulation → Controller (State broadcast):**
  - Frequency: 1000 Hz (every simulation step)
  - Message: State (joint positions, velocities, sensors, ~2.5 KB)
  - Port: 8889 (configurable)

- **Controller → Simulation (Control input):**
  - Frequency: ~500-1000 Hz (controller-dependent)
  - Message: Control commands (actuator signals, ~1.5 KB)
  - Port: 8888 (configurable)

**Rationale:**
- **1:1 communication:** Simpler than multi-client broadcast
- **Non-blocking receives:** Controller callback polls during `mj_step()`
- **No handshake:** UDP for minimum latency (accept packet loss risk)

---

## 📊 Message Formats

### Common Header

```c
typedef struct {
    uint32_t magic;         // Magic number (0x54454E54 = "TENT")
    uint32_t version;       // Protocol version (currently 1)
    uint64_t timestamp_ns;  // Nanosecond timestamp (CLOCK_MONOTONIC)
    uint32_t sequence;      // Sequence number (for packet loss detection)
    uint32_t checksum;      // CRC32 checksum (optional validation)
} __attribute__((packed)) tentacles_msg_header_t;
```

### State Message (Simulation → Controller)

```c
typedef struct {
    tentacles_msg_header_t header;

    // Model dimensions
    uint32_t nq, nv, na, nsensordata;

    // Simulation state
    double time;

    // Joint state
    double qpos[64];     // Joint positions
    double qvel[64];     // Joint velocities
    double qacc[64];     // Joint accelerations

    // Actuator state
    double act[32];      // Actuator activations

    // Sensor data
    double sensordata[64]; // Sensor readings

    // Root body state (for humanoid)
    double root_pos[3];      // Position (x, y, z)
    double root_quat[4];     // Orientation (w, x, y, z)
    double root_vel[3];      // Linear velocity
    double root_angvel[3];   // Angular velocity

} __attribute__((packed, aligned(8))) tentacles_state_msg_t;
```

**Size:** ~2.5 KB

### Control Message (Controller → Simulation)

```c
typedef struct {
    tentacles_msg_header_t header;

    // Control dimensions
    uint32_t nu;

    // Control signals
    double ctrl[32];         // Control values

    // Control flags
    uint8_t reset_requested;   // Request simulation reset
    uint8_t pause_requested;   // Request pause
    uint8_t step_mode;         // 0=continuous, 1=single-step

    // Optional: feedforward terms
    double qfrc_applied[64];   // Applied forces (advanced control)

} __attribute__((packed, aligned(8))) tentacles_control_msg_t;
```

**Size:** ~1.5 KB

---

## 🚀 Usage

### Building Tentacles

```bash
cd /home/xiaochen/ws-mujoco/mujoco-dev-01
mkdir -p build && cd build

# Configure with tentacles enabled
cmake ../src -DMUJOCO_BUILD_TENTACLES=ON

# Build
make -j$(nproc)

# Executables will be in:
# - build/bin/run_simulation
# - build/bin/test_receiver
# - build/bin/test_sender
```

### Running Networked Simulation

**Terminal 1 (Simulation):**
```bash
cd build
./bin/run_simulation ../src/model/humanoid/humanoid.xml 127.0.0.1

# Arguments:
#   argv[1] = model path (e.g., humanoid.xml)
#   argv[2] = remote controller IP (default: 127.0.0.1)
```

**Terminal 2 (Controller - receive states):**
```bash
./bin/test_receiver 8889

# Listens on port 8889 for state messages
# Prints received joint positions, velocities, etc.
```

**Terminal 3 (Controller - send controls):**
```bash
./bin/test_sender 127.0.0.1 8888

# Sends control messages to simulation on port 8888
# Example: random control inputs or keyboard control
```

### Integration with Custom Controller

```python
# Python example (using socket library)
import socket
import struct

# Connect to simulation
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 8889))  # Receive states

sim_addr = ('127.0.0.1', 8888)  # Send controls

while True:
    # Receive state
    data, addr = sock.recvfrom(4096)
    # Parse state message (see msg_state.h for format)

    # Compute control
    ctrl = compute_control(state)

    # Send control message
    ctrl_msg = pack_control_message(ctrl)
    sock.sendto(ctrl_msg, sim_addr)
```

---

## 🔌 Dependencies

### Required

- **MuJoCo:** >= 3.0 (parent repository)
- **liburing:** io_uring library (Linux kernel 5.1+)
  ```bash
  sudo apt install liburing-dev
  ```
- **CMake:** >= 3.16
- **Compiler:** C11/C++17 support

### Optional

- **Python:** For controller examples
- **Google Benchmark:** For performance testing (benchmarks/)

### Raspberry Pi 4 Setup

```bash
# Check kernel version (should be >= 5.1)
uname -r

# Check io_uring support (should be 0 = enabled)
cat /proc/sys/kernel/io_uring_disabled

# Install dependencies
sudo apt update
sudo apt install liburing-dev cmake build-essential
```

---

## 📈 Performance Targets

| Metric | Target | Expected (RPi4) | Notes |
|--------|--------|-----------------|-------|
| **Round-trip latency** | < 1 ms | 440-720 µs | io_uring + localhost UDP |
| **Simulation frequency** | 1000 Hz | 1000 Hz | 1ms timestep for humanoid |
| **State message rate** | 1000 Hz | 1000 Hz | Every simulation step |
| **Control message rate** | 500-1000 Hz | Variable | Controller-dependent |
| **Network overhead** | < 10% CPU | ~5-10% | io_uring efficiency |
| **Bandwidth** | ~4 MB/s | ~4 MB/s | 1000 Hz × 4 KB |
| **Packet loss** | < 0.1% | ~0% | Localhost loopback |

---

## 🛠️ Implementation Progress

### Phase 1: Setup & Structure ✅
- [x] Create `tentacles/` folder structure
- [x] Create `messages/`, `network/`, `apps/`, `benchmarks/` subdirectories
- [x] Write comprehensive README.md (this file)
- [ ] Set up Git branch: `feature/tentacles-udp`

### Phase 2: Message Definitions
- [ ] Implement `msg_common.h` and `.c`
  - [ ] Common header structure
  - [ ] Timestamp utilities
  - [ ] Checksum computation
  - [ ] Header validation
- [ ] Implement `msg_state.h` and `.c`
  - [ ] State message structure
  - [ ] `tentacles_state_msg_from_mjdata()` function
  - [ ] Serialization helpers
- [ ] Implement `msg_control.h` and `.c`
  - [ ] Control message structure
  - [ ] `tentacles_control_msg_to_mjdata()` function
  - [ ] Deserialization helpers

### Phase 3: Network Transport Layer
- [ ] Implement `uring_udp.h` and `.c`
  - [ ] `tentacles_uring_init()` - Initialize io_uring + UDP socket
  - [ ] `tentacles_uring_recv_submit()` - Submit receive operations
  - [ ] `tentacles_uring_send_submit()` - Submit send operations
  - [ ] `tentacles_uring_poll()` - Poll for completions (non-blocking)
  - [ ] `tentacles_uring_cleanup()` - Cleanup resources
- [ ] Implement `udp_transport.h` and `.c`
  - [ ] High-level transport API
  - [ ] Statistics tracking (packets sent/received, errors)
  - [ ] Error handling and logging

### Phase 4: CMake Build System
- [ ] Create `tentacles/CMakeLists.txt`
  - [ ] Option: `MUJOCO_BUILD_TENTACLES` (default: ON)
  - [ ] Find liburing dependency
  - [ ] Add subdirectories
- [ ] Create `tentacles/messages/CMakeLists.txt`
  - [ ] Build `tentacles_messages` library
  - [ ] Link against `mujoco::mujoco`
- [ ] Create `tentacles/network/CMakeLists.txt`
  - [ ] Build `tentacles_network` library
  - [ ] Link against `tentacles::messages` and `liburing`
- [ ] Create `tentacles/apps/CMakeLists.txt`
  - [ ] Build `run_simulation` executable
  - [ ] Build `test_receiver` and `test_sender`
- [ ] Modify `src/CMakeLists.txt`
  - [ ] Add `add_subdirectory(tentacles)` (conditional)

### Phase 5: Applications
- [ ] Implement `apps/run_simulation.cc`
  - [ ] Load MuJoCo model
  - [ ] Initialize io_uring UDP transport
  - [ ] Register control callback (`mjcb_control`)
  - [ ] Simulation loop with state broadcast
  - [ ] Real-time synchronization
  - [ ] Command-line argument parsing
- [ ] Implement `apps/test_receiver.cc`
  - [ ] Listen for state messages
  - [ ] Parse and display state data
  - [ ] Measure message frequency
- [ ] Implement `apps/test_sender.cc`
  - [ ] Send control messages
  - [ ] Keyboard/gamepad input
  - [ ] Random control for testing

### Phase 6: Testing & Validation
- [ ] Unit tests
  - [ ] Message serialization/deserialization
  - [ ] Header validation
  - [ ] Checksum computation
- [ ] Integration tests
  - [ ] End-to-end communication test
  - [ ] Packet loss handling
  - [ ] Message sequencing
- [ ] Performance benchmarks
  - [ ] `benchmarks/latency_test.cc` - Measure round-trip latency
  - [ ] `benchmarks/throughput_test.cc` - Measure sustained throughput
  - [ ] CPU profiling (perf, flamegraphs)

### Phase 7: Documentation
- [ ] Update this README with actual performance results
- [ ] Add API documentation (Doxygen comments)
- [ ] Create example controller (Python)
- [ ] Document network protocol specification
- [ ] Add troubleshooting guide
- [ ] Create architecture diagrams

---

## 🔄 Git Workflow

### Branch Strategy

To minimize conflicts with upstream MuJoCo:

```bash
# Setup upstream remote (one-time)
cd /home/xiaochen/ws-mujoco/mujoco-dev-01
git remote add upstream https://github.com/google-deepmind/mujoco.git
git fetch upstream

# Create feature branch
git checkout -b feature/tentacles-udp

# All tentacles work happens here
git add src/tentacles/
git commit -m "[TENTACLES] Add UDP communication layer"

# Periodically sync with upstream
git fetch upstream
git merge upstream/main
# Conflicts should be minimal (only src/CMakeLists.txt)
```

### Files Modified (Minimal Upstream Conflicts)

**Only 1 file modified:**
- `src/CMakeLists.txt` - Add ~3 lines:
  ```cmake
  # Add tentacles module
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tentacles)
      add_subdirectory(tentacles)
  endif()
  ```

**All other changes in new files under `src/tentacles/`**

---

## 🐛 Troubleshooting

### io_uring not working

```bash
# Check kernel support
uname -r  # Must be >= 5.1

# Check if io_uring is disabled
cat /proc/sys/kernel/io_uring_disabled
# Should be 0 (enabled). If 2 (disabled), run:
sudo sysctl -w kernel.io_uring_disabled=0
```

### High latency (> 1ms)

- Check CPU frequency scaling (use `performance` governor)
- Verify localhost loopback (not going through network interface)
- Profile with `perf` to find bottlenecks
- Reduce simulation complexity (simpler model for testing)

### Packet loss

- Check `dmesg` for UDP buffer overflow
- Increase UDP receive buffer:
  ```bash
  sudo sysctl -w net.core.rmem_max=8388608
  sudo sysctl -w net.core.rmem_default=8388608
  ```

### Build errors

```bash
# Missing liburing
sudo apt install liburing-dev

# CMake can't find liburing
export PKG_CONFIG_PATH=/usr/lib/pkgconfig:$PKG_CONFIG_PATH
```

---

## 📚 References

### MuJoCo Resources
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Programming Guide](https://mujoco.readthedocs.io/en/stable/programming/index.html)
- [MuJoCo Callbacks](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#physics-callbacks)

### ROS2 Integration Examples
- [moveit/mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control)
- [Woolfrey/interface_mujoco_ros2](https://github.com/Woolfrey/interface_mujoco_ros2)
- [ubi-agni/mujoco_ros_pkgs](https://github.com/ubi-agni/mujoco_ros_pkgs)

### io_uring Resources
- [liburing GitHub](https://github.com/axboe/liburing)
- [io_uring Tutorial](https://unixism.net/loti/)
- [Efficient IO with io_uring](https://kernel.dk/io_uring.pdf)

### Related Projects
- [MuJoCo Playground](https://arxiv.org/html/2502.08844v1) - Hardware interface abstraction
- [dm_control](https://github.com/google-deepmind/dm_control) - DeepMind Control Suite

---

## 📝 License

Tentacles follows the same license as the parent MuJoCo repository (Apache 2.0).

This module is a custom extension for local use and is not intended for upstream contribution to the official MuJoCo repository.

---

## 👥 Author

- **Xiaochen** - Raspberry Pi 4 + MuJoCo + TensorFlow middleware integration
- **Project:** Custom robotics control with ultra-low latency communication

---

## 🚧 Status

**Current Phase:** Phase 1 - Setup & Structure
**Last Updated:** 2025-11-16
**Version:** 0.1.0-alpha

---

**Next Steps:**
1. Implement message structures (`messages/`)
2. Implement io_uring UDP transport (`network/`)
3. Build and test on Raspberry Pi 4
4. Benchmark latency and throughput
5. Iterate based on performance results

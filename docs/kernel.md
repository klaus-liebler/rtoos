# Kernel

The kernel is header-only: `rtos::Kernel<Port, Config>` ([include/rtos/rtos.hpp](../include/rtos/rtos.hpp)). The CPU is a template parameter that satisfies the `rtos::CpuPort` concept, so the kernel compiles unchanged for the host (with `MockPort`) and for the Cortex-M33 (`Cm33Port`).

## Configuration

```cpp
struct MyConfig : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 8;        // application threads (idle is extra)
    static constexpr std::size_t priority_levels = 32;   // 0 = most urgent, last level = idle
    static constexpr std::size_t notify_slots = 4;       // notification slots per thread
    static constexpr std::size_t timer_wheel_slots = 32; // power of two
    static constexpr std::size_t idle_stack_bytes = 512;
    static constexpr rtos::Tick initial_tick = 0;        // optional: start near the 32-bit wrap in tests
};
```

The `KernelConfig` concept rejects invalid values at compile time ([config.hpp](../include/rtos/config.hpp)).

## Scheduling

- **Preemptive, fixed priorities**, with an O(1) ready queue: one 32-bit bitmap, one CLZ.
- **Round robin** between threads of equal priority.
- **Preemption threshold:** a thread can run with a threshold above its priority. Threads that share data get the same threshold (a ceiling). They never preempt each other and need no mutex, while more urgent threads still preempt them.
- **Run statistics** for task monitors: every thread counts how often it was switched in (`run_count`) and, with a port that has `cycle_count()` (Cortex-M33: DWT_CYCCNT, after `CycleCounter::enable()`), the CPU cycles it ran (`run_cycles`).
- **Timeouts** are kept in a timer wheel. `Tick` is 32 bits (1 ms); `Kernel::now64()` extends it to 64 bits for clocks that must never wrap (OPC UA).

## Services and execution contexts

| Service | Thread | kernel-aware ISR (priority ≥ 5) | zero-latency ISR (priority < 5) |
|---|---|---|---|
| `Kernel::notify` / `notify_from_isr` | ✓ | ✓ | ✓ (lock-free) |
| `Kernel::resume` / `resume_from_isr` | ✓ | ✓ | – |
| `Kernel::defer` / `defer_from_isr` | ✓ | ✓ | ✓ (lock-free) |
| `Semaphore::take`, `give` / `give_from_isr` | ✓ | give | – |
| `MessageQueue::send`, `receive` / `try_send_from_isr`, `try_receive_from_isr` | ✓ | try | – |
| `ObjectPool::make(timeout)` | ✓ (blocking) | – | – |
| `ObjectPool::try_make`, `try_allocate` | ✓ | ✓ | ✓ (lock-free) |
| `ObjectPool::destroy` / `destroy_from_isr` | ✓ | ✓ (direct hand-over) | ✓ (lock-free, wake-up deferred) |
| `BlockPool` | ✓ | ✓ | ✓ (lock-free) |
| `SpscRing` | one producer, one consumer, any context | | |

- **Contracts check the context:** every `*_from_isr` service checks that it runs in an ISR; the normal services check the opposite. Services that take the kernel lock also check `Port::may_lock()` (the ISR priority is not above the kernel mask).
- **No "higher priority task woken" flag:** every service requests the context switch (PendSV) itself when the choice of thread can change.
- **Waiting order:** blocked threads are served first come, first served. A released unit (semaphore count, pool block, message) goes straight to the first waiter.
- **Two phases:** blocking calls also exist as `*_begin` plus `Kernel::wait_end()`. On the hardware the context switch happens between the two; host tests call both phases explicitly.
- **`Kernel::Lock`:** a short critical section (BASEPRI on the Cortex-M33). The Modbus `RegisterBank` uses it to share registers between application threads and the network thread.

## Rules

- **Interrupt priorities:** ISRs with a priority `< RTOS_CM33_KERNEL_PRIORITY` (default 5) are never masked by the kernel. They may only use the lock-free services in the table above.
- **Stacks:** static, 8-byte aligned. The port programs PSPLIM for every thread, so an overflow faults on the hardware and in QEMU. Renode does not enforce PSPLIM: size stacks from the frames of the deepest path, and keep large buffers of single-threaded code static.
- **FPU:** lazy stacking. Only threads that use the FPU pay for the extra context.
- **Contracts:** the host uses `enforce` (report through the default handler). The target uses `quick_enforce` (`std::terminate` → `abort()` → breakpoint).
- **Workaround:** in `noexcept` functions, preconditions are written as `contract_assert` at the start of the body instead of `pre()`, because of a GCC 16 bug (see [CLAUDE.md](../CLAUDE.md)).

## CPU port (Cortex-M33)

[port/cortex_m33](../port/cortex_m33) implements `CpuPort`: PendSV/SVC context switches, the BASEPRI lock, PSPLIM, lazy FPU, SysTick and the DWT cycle counter (`CycleCounter`). `RTOS_CM33_BIND_KERNEL(kernel)` connects the exception handlers to the application's kernel object.

## Tests

- Host: [kernel_test.cpp](../tests/kernel_test.cpp), [sync_test.cpp](../tests/sync_test.cpp), [block_pool_test.cpp](../tests/block_pool_test.cpp) on the mock port. `KernelWrapTest` runs the kernel tests with `initial_tick` just below the 32-bit wrap.
- Target: [apps/selftest](../apps/selftest/main.cpp) in QEMU, Renode and on the board ([examples](examples.md#fcu_selftest-the-kernel-on-the-real-mechanisms)).

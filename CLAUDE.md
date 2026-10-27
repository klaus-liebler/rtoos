# Project rules – rtoos

## No legacy baggage (binding)
- **Exactly one compiler: GCC 16 or newer**, for the host (x86_64-w64-mingw32) and the target (arm-none-eabi). CMake aborts on any other compiler.
- **Exactly one language standard: C++26** (`-std=c++26 -fcontracts`).
- **No compat layers.** No fallbacks or shims for older standards, older compiler versions, or other compilers (MSVC, Clang, GCC < 16). No `#if __cpp_…`/`__has_include` branches that simulate a missing feature, no polyfills, no compiler-specific CMake branches (`if(MSVC)` etc.).
- Use standard features directly: `pre()`/`post()`/`contract_assert` (C++26 contracts), `std::inplace_vector`, Concepts, `std::span`, and so on.
- If a feature is missing or broken, update the compiler. Never re-implement the feature in the project.

## Workarounds for compiler bugs
A workaround is allowed only when it
1. is as narrow as possible (preferably a single build flag, no code change),
2. is commented where it applies, with the symptom and the removal condition,
3. is listed here.

| Workaround | Location | Symptom | Remove when |
|---|---|---|---|
| In `noexcept` functions, preconditions are written as `contract_assert(...)` as the first statements of the body instead of `pre(...)` | all headers (comment in `CMakeLists.txt`) | GCC 16.2: `noexcept` + `pre()`/`post()` + `-fno-exceptions` ICEs or **silently miscompiles** as soon as anything with a destructor is involved (RAII locals, by-value parameters: a moved-in `PacketRef` arrived empty in the callee). The flag workarounds don't help: `-fcontracts-definition-check=off -fcontracts-client-check=all` breaks by-value parameters, and `-fexceptions` pulls the unwinder and `malloc` into the image | This compiles with `-std=c++26 -fcontracts -fno-exceptions -Os` and returns 0: `struct R{int*p; R(int*q):p(q){} R(R&&o)noexcept:p(o.p){o.p=nullptr;} ~R(){}}; [[gnu::noinline]] bool f(R r) noexcept pre(r.p){return r.p;} int main(){int x; return f(R(&x))?0:1;}` — then convert back to `pre()` |

## Toolchain
- GCC 16 (x64 and arm-none-eabi): `C:\opt\msys64\ucrt64\bin`
- QEMU with the `fcu` machine: `C:\opt\qemu-stm32h5` (built by `emulation/qemu/build-qemu.sh`)
- Renode: `C:\opt\renode`
- Build only through `CMakePresets.json` (`cmake --preset host|arm|board`, `cmake --build --preset ...`, `ctest --preset host|arm`); the presets put the toolchain on the PATH (`cmake --build build/host` fails silently: missing GCC DLLs).
- `arm` = firmware for the emulators (emulation network settings, emulator tests), `board` = firmware for the real FCU. Emulation-only values belong in the `arm` preset's cache variables (e.g. `RTOS_NET_*`), never as constants in firmware code.
- Version control: git, branch `main`. Commit finished, tested steps; end messages with the Co-Authored-By line.
- Text files use LF (`.gitattributes`); keep the user's `task.md` as is.

## Emulation (QEMU machine `fcu`, Renode `emulation/renode/fcu.repl`)
- Every change to kernel, port, chip, or board must pass `ctest --preset arm` (self-test in both emulators, network test `net_renode` in Renode over TAP) in addition to `ctest --preset host`. `net_renode` reports *skipped* without the TAP adapter (`emulation/renode/setup-tap.ps1`); a skip doesn't count as a pass for network changes.
- Test certificates (`certs/`, from `tools/make_certs.py`) are for development only and never go into version control.
- The server certificate is signed by the project CA `rtoosCA` (`RTOS_CA_CERT`/`RTOS_CA_KEY` in CMakePresets.json, in the user's OneDrive certificates folder; profile in `tools/make_ca.py`: ECDSA P-256, RFC 5280 strict, name-constrained to `.local` and private IPv4 nets). The CA key is only read by openssl in place: never copy or print it. The whole chain is ECDSA P-256: `NO_RSA` everywhere, no strictness exceptions in tests.
- Hardware facts (addresses, bits, reset values, IRQ numbers) come from the STM32H563 SVD / RM0481, never from memory. Chip code, Renode platform, and QEMU model must agree.
- An emulator gap is fixed in the emulator model where possible. If it can't be fixed there, the self-test reports the feature as `UNSUPPORTED` and the runner allows exactly that feature for that emulator (`--allow-unsupported`). Never weaken a test.
- Known gaps: Renode does not enforce PSPLIM.
- Timing: both emulators run 250 MIPS (CPI 1) at the firmware's 250 MHz SYSCLK. Renode uses `PerformanceInMips`; QEMU uses `-icount shift=2,sleep=off` (set by `emulation/run.py`; without it Windows host timers make SysTick lose periods). `DWT_CYCCNT` values from emulators are instruction counts, not hardware cycles; never quote them as hardware performance.
- QEMU model changes: edit `emulation/qemu/{hw,include}` and the build patch `patches/0001-*.patch`, then rebuild (`build-qemu.sh`, or incrementally in `build/qemu/src/build` with `make && make install`).

## Network architecture
- Everything network runs in one thread: `NetRuntime` (stack poll, services, flush). New servers are services added to the runtime; TCP servers implement a `TcpSession` for `TcpServer` instead of their own connection handling.
- Several interfaces: drivers satisfy `NetDriver` and are `attach`ed to the stack; services take an `InterfaceId` where it matters (DHCP, sending broadcasts) and react to `NetEventHandler` events instead of polling.

## Embedded rules
- No heap at run time, no exceptions, no RTTI on the target. Contract semantic on the target: `quick_enforce`.
- Hardware access only through ports that satisfy a concept (`rtos::CpuPort`); the kernel stays testable on the host.

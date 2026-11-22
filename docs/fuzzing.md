# Sanitizers and fuzzing (Linux)

MinGW has neither AddressSanitizer nor libFuzzer, so the Windows host tests ([testing.md](testing.md))
cannot see a read past the end of a buffer. A second host build on Linux closes that gap. It uses
the project's one compiler, GCC 16, and runs in a container:

| Preset | Compiler | What it does |
|---|---|---|
| `linux-asan` | `g++` | all host tests under ASan + UBSan (reports, every library of the build: rtoos, wolfSSL, littlefs, FileX); the AFL++ harnesses replay their corpus and the regressions (`fuzz_replay_*`) |
| `linux-afl` | `afl-g++-fast` (AFL++ GCC plugin) | the AFL++ harnesses of [tests/fuzz](../tests/fuzz) with ASan + UBSan, edge coverage, persistent mode |

Both keep their own test CA in the build tree (`RTOS_CERT_DIR`): the rtoosCA is not available in
the container, and they must never re-issue the certificates in `certs/`.

## The container

[tools/fuzz/Dockerfile](../tools/fuzz/Dockerfile): `gcc:16.2.0` (Debian, GCC with plugin headers),
CMake, Ninja, and AFL++ built from a pinned tag against GCC 16's plugin API.

```
docker build -t rtoos-fuzz tools/fuzz
docker run --rm -v ${PWD}:/src -w /src rtoos-fuzz sh -c "cmake --preset linux-asan && cmake --build --preset linux-asan -j && ctest --preset linux-asan -j 28"
```

(run in `rtoos/`; PowerShell: `-v "${PWD}:/src"`).

## Harnesses

Each harness ([tests/fuzz](../tests/fuzz)) defines `fuzz_one(input)` and includes
[fuzz_main.hpp](../tests/fuzz/fuzz_main.hpp): with `afl-g++-fast` that is AFL++'s persistent loop,
with `g++` a replay of files and directories. The input format is described at the top of each.

| Harness | Target | Input |
|---|---|---|
| `http` | `HttpRequestParser` | segment size + TCP byte stream (pipelined requests) |
| `websocket` | `WebSocketDecoder` (RFC 6455) | segment size + frames |
| `dns` | DNS names: compression, labels (resolver, mDNS) | offset + message |
| `dhcp` | DHCP options (client and server) | options |
| `modbus` | PDU server; RTU framing with silences | mode + PDU or RTU stream |
| `opcua` | `opcua::Connection`: Hello, SecureChannel (None, Basic256Sha256), Session, all services, subscriptions | the client's byte stream, cut into UA TCP messages as the server does |
| `ethercat` | `ethercat::Slave` over the simulated ESC: AL state machine, SII EEPROM, mailbox, CoE SDO + SDO information, PDO assignment, process data, watchdog | a hostile master's operations (any SyncManager configuration) |
| `plc` | `plc::Vm::load` (the verifier) and the interpreter, debugger included | a program image (length and CRC corrected by the harness) |

Seed corpus: [tests/fuzz/corpus](../tests/fuzz/corpus).
[make_corpus.py](../tests/fuzz/make_corpus.py) writes all of it except `opcua` and `plc`:
- `plc`: the compiler's test images ([tests/plc_corpus](../tests/plc_corpus)).
- `opcua`: the client streams of [opcua_test.cpp](../tests/opcua_test.cpp), written with
  `RTOS_FUZZ_CORPUS=<dir>` and reduced with `afl-cmin` (1564 streams, 51 kept).

## A campaign

```
docker run --rm -v ${PWD}:/src -w /src rtoos-fuzz tools/fuzz/run.sh <seconds> [jobs per harness] [harness...]
```

[run.sh](../tools/fuzz/run.sh) builds `linux-afl` and fuzzes the harnesses in parallel (one main
instance and `jobs`-1 secondaries each). A second call resumes the campaign. Findings are in
`build/linux-afl/findings/<harness>/*/crashes` and `*/hangs`. To reproduce a finding, run it under
the replay build (`build/linux-asan/tests/fuzz/fuzz_<harness> <file>`). Once it is fixed, the
input goes to `tests/fuzz/regressions/<harness>/`, where `fuzz_replay_<harness>` keeps it.

## Findings

| Where | What | Fix |
|---|---|---|
| `WebSocketDecoder` | A fragmented message, then a continuation frame with a 64-bit length near 2^64: `message_size_ + length_` wrapped to a small value, passed the size check, and the decoder wrote the peer's bytes past its message buffer - from any client that reaches port 443, before anything else (AFL++, 11 crashes, one cause) | `length_ > MaxMessage - message_size_`; a 64-bit length with the top bit set is a protocol error (RFC 6455 5.2); `WebSocket.ContinuationLengthCannotWrapTheSizeCheck`, `regressions/websocket` |
| wolfSSL 5.9.4 `RsaPad_OAEP`, `RsaUnPad_OAEP` | With `WOLFSSL_SMALL_STACK` + `WOLFSSL_NO_MALLOC` (this project's settings) the temporaries are stack arrays, but they are freed under the weaker `SMALL_STACK` condition: `wolfSSL_Free` of a stack address. In the static pool that counts a free and, on a size match, links the stack address into the free list. It ran for every secured OPC UA OpenSecureChannel and could be triggered by any client. | patched copy of `rsa.c` ([cmake/wolfssl.cmake](../cmake/wolfssl.cmake)); `OpcUaTest.OaepFreesOnlyWhatItAllocated` checks the pool's alloc/free counts (also on MinGW and the target) |
| wolfSSL 5.9.4 `RsaPrivateDecryptEx` | `WOLFSSL_NO_MALLOC`: decrypts in place and moves the message to the front with `memcpy`, an overlapping copy | `XMEMMOVE` in the same patched `rsa.c`; ASan in every OPC UA test with a secured channel |
| `plc::Vm::load` | A task's instruction budget - the watchdog of a runaway program - came from the image unchecked: images with budgets up to 2^32 held the VM for minutes (AFL++ hangs) | `Vm(data, contexts, max_budget)`, default 2^24; `PlcVm.BudgetAboveTheVmsLimitIsRefused` |
| `opcua::ByteWriter::WriteRaw` | `memcpy(dst, nullptr, 0)` for an empty String/ByteString (undefined; GCC may drop later null checks) | no copy for 0 bytes |
| tests: `net_fixture.hpp`, `https_fairness_test.cpp`, `opcua_service_harness.hpp` | the same `memcpy`; `read()` returned strings pointing into a freed response | fixed in the tests |
| wolfSSL `sp_c32.c`, `aes.c` (64-bit `GMULT`) | signed overflow, misaligned 64-bit load - only in the C paths of a 64-bit host; the firmware links `sp_cortexm.c` and the Thumb-2 `GCM_gmult_len` (map file) | UBSan's two checks off for wolfSSL in the Linux presets ([cmake/wolfssl.cmake](../cmake/wolfssl.cmake)) |

### Open

- **HTTPS slots** (the FCU's black-box test `tools/pentest-renode.py`, "slowloris"): the FCU has two HTTPS
  sessions. Two TCP connections that send nothing hold both for the handshake timeout (10 s) and are not
  evicted (only an idle keep-alive connection is); a client that reconnects every 10 s keeps the web UI
  away. Without authentication two open WebSockets hold them for good. Modbus and OPC UA are not affected.

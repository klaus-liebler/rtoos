# Design decisions

Decisions in rtoos are made from measurements. This page records the ones that are not obvious from the code.

## Zero-copy TLS record layer: measured and rejected

For a while rtoos had its own TLS 1.3 record layer. After the handshake it took over the keys from wolfSSL, encrypted directly into the packet buffers and decrypted in the receive buffer. To pay off, it had to save more than the copies cost. [apps/bench](../apps/bench/main.cpp) (`fcu_bench`) measures exactly this: it produces the same ChaCha20-Poly1305 record once without and once with the copies of the wolfSSL path. The crypto is identical in both cases; wolfSSL's own record handling is not included.

```
python emulation/run.py qemu build/arm/apps/fcu_bench.elf --timeout 30
```

Results in the emulators (instructions, not real clock cycles; on the board the same firmware reports real cycles):

| Record | zero copy | with copies | share of the copies |
|---|---|---|---|
| send 39 B | 4 392 | 4 535 | 3.1 % |
| send 512 B | 15 945 | 16 618 | 4.0 % |
| send 1438 B (from flash) | 36 699 | 38 400 | 4.4 % |
| receive 39 B | 4 401 | 4 594 | 4.2 % |
| receive 512 B | 16 002 | 16 959 | 5.6 % |
| receive 1438 B | 36 735 | 38 748 | 5.1 % |

Reasons for the decision:

- **Small gain:** the copies cost 3–6 % of the record CPU time. wolfSSL reports the same order of magnitude for its own zero-copy send path: +1.0 % with ChaCha20, +2.9 % with AES-GCM ([PR 10899](https://github.com/wolfSSL/wolfssl/pull/10899)). That path exists only for TLS 1.2 and is off with `WOLFSSL_NO_TLS12`.
- **Worse for large downloads:** one record per segment cost more fixed per-record overhead (about 4 500 instructions) than the saved copies.
- **Throughput:** at 100 Mbit/s the wire is the bottleneck, not the CPU.
- **Security and maintenance:** an own record layer (nonces, sequence numbers, KeyUpdate, tags) is about 700 lines of security-critical code to maintain alone. wolfSSL is maintained by others and patched when vulnerabilities appear.
- **In the whole system:** a WebSocket echo costs 37 700 instead of 34 000 instructions with wolfSSL records (151 instead of 136 µs at 250 MHz).

## wolfSSL pool sized from measurements

wolfSSL with `WOLFSSL_STATIC_MEMORY` serves allocations from fixed-size buckets. A guessed layout either wastes RAM or fails under load with `MEMORY_E`. wolfSSL tracks no high-water marks itself, so [tls_memory.hpp](../include/rtos/net/tls_memory.hpp) counts them through the pool's debug callback (`WOLFSSL_STATIC_MEMORY_DEBUG_CALLBACK`): per bucket size the number of blocks, the peak in use and the largest request served.

Method:

1. Load the pool with many small buckets in every size, so that requests do not spill into larger buckets and hide the real demand.
2. Run the Renode network test at full load: 6 HTTPS sessions (5 downloads and a WebSocket), concurrent handshakes, OPC UA RSA opens.
3. Choose bucket sizes that fit the largest requests, and counts of the peak plus about 50 %; the largest buckets for two concurrent handshakes (`max_handshakes`) plus one.

Result for `fcu_net`: a peak of about 28 KB. The pool went from 160 KB for TLS plus 48 KB for OPC UA to one shared 72 KB pool. The network test checks under full load that no allocation fails, and the firmware prints the marks every 10 s (`tls: pool peak …`). The host test uses its own sizes, because the x64 structures are larger.

## One network thread

All network services, including TLS, OPC UA and USB, run in one thread. Locks, per-connection stacks and priority inversion between servers disappear. The price is that every service must be short in the foreground. Expensive work is split into bounded background steps, and fairness is tested on the wire ([networking](networking.md#fairness-in-one-thread)). The measured worst case is a single RSA-2048 private-key operation in an OpenSecureChannel step.

## ARP: learn only from replies

The stack deviates from RFC 826 on purpose: it creates or updates entries only from replies to its own requests ([security](security.md#arp-hardening)).

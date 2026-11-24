# Security

## Certificates

A device needs:

| File | Use | Key |
|---|---|---|
| `server.der`, `server.key.der` | HTTPS server certificate, SAN = DNS name and IP of the device | ECDSA P-256 |
| `opcua.der`, `opcua.key.der` | OPC UA application certificate, SAN = ApplicationUri, DNS name, IP | RSA-2048 (Basic256Sha256 requires RSA) |
| `ca.der` | the CA that OPC UA client certificates must chain to | |

The firmware embeds them with `#embed` from a directory given by `--embed-dir`. RSA exists only for OPC UA: wolfSSL is built with RSA, but TLS never offers it.

### Project CA

[tools/make_ca.py](../tools/make_ca.py) creates a CA once:

```
python tools/make_ca.py C:\opt\msys64\ucrt64\bin\openssl.exe <directory> --name rtoosCA
```

Its profile:

- **Key:** ECDSA P-256 with SHA-256, the curve that the device and TLS use anyway, so the TLS chain needs no RSA. The OPC UA RSA-2048 certificate is signed by the same CA.
- **Extensions per RFC 5280:** `basicConstraints` critical with `CA:TRUE, pathlen:0`, `keyUsage` critical (keyCertSign, cRLSign), and a Subject Key Identifier. Strict verifiers such as OpenSSL `-x509_strict` and Python 3.13+ accept it.
- **Name constraints (critical):** the CA signs only names under `.local` and private IPv4 networks (10/8, 172.16/12, 192.168/16). Even with the key, nobody can forge a certificate for a public name.

The CA key never leaves its location: openssl reads it in place, and only `ca.crt`/`ca.der` are copied into the certificate directory.

### Device certificates

Products usually issue device certificates with their own build or provisioning tool and pass the directory to the firmware with `--embed-dir`. rtoos needs nothing else.

The rtoos default is [tools/make_certs.py](../tools/make_certs.py), which the examples use through the CMake function `rtos_certificates()`:

```cmake
rtos_certificates(my_certs DIR ${CMAKE_SOURCE_DIR}/certs
                  HOST my-device.local IP 192.168.1.50 OPCUA_URI urn:my-company:my-device)
target_link_libraries(my_firmware PRIVATE my_certs)   # adds --embed-dir
```

At every configure step it checks the certificates strictly against the CA (`-x509_strict`, host name, IP, ApplicationUri, at least 30 days left) and issues them again only when needed: 397 days validity. The CA comes from `RTOS_CA_CERT` and `RTOS_CA_KEY` (CMake cache or environment). Without them, a local test CA with the same profile is created in the certificate directory. It also issues `opcua_client.*` for the tests.

Certificate directories contain unencrypted development keys: keep them out of version control.

### Trusting the CA in a browser

```
certutil -user -addstore Root certs\ca.crt
certutil -user -delstore Root "rtoos FCU development CA"     # remove again
```

curl on Windows needs `--ssl-no-revoke` (Schannel), because the CA publishes no revocation list.

## TLS

- TLS 1.3 only (wolfSSL), X25519 and P-256 key exchange, ECDSA P-256 server certificate, ChaCha20-Poly1305 and AES-128-GCM (the two suites every browser offers; AES-256-GCM-SHA384 would only add SHA-384).
- Handshakes and records are done entirely by wolfSSL, including KeyUpdate and close_notify.
- The static pool and the handshake limit keep a flood of connections from exhausting memory; idle handshakes are aborted after 10 s.
- A handshake older than 2 s (`HttpsServerConfig::handshake_evict_ms`) gives way to a new client - its session when all are taken, its handshake turn when another handshake waits: connections that never finish (slowloris) cannot lock browsers out.
- WebSocket upgrades from foreign origins are rejected (403).

## Authentication (web, later OPC UA and SSH)

The building blocks live in `include/rtos/auth/` and know nothing of a protocol; the application (the FCU firmware: `src/auth.cpp`) decides the policy.

| Part | File | What it does |
|---|---|---|
| PBKDF2-HMAC-SHA256 | [pbkdf2.hpp](../include/rtos/auth/pbkdf2.hpp) | RFC 8018, one 32-byte block, **in steps** (`step(n)`): a check costs 100-300 ms by design and must not run as one call in the network thread. Inner/outer HMAC midstates are hashed once, so an iteration is two SHA-256 compressions (wolfSSL's HMAC would need four). Same output as Python's `hashlib.pbkdf2_hmac` and .NET's `Rfc2898DeriveBytes.Pbkdf2`. |
| Accounts | [credentials.hpp](../include/rtos/auth/credentials.hpp) | `UserRecord` (name, role, flags, salt, iterations, hash - never a password), `PasswordCheck`/`PasswordSetter` (stepped), password rules, `CredentialStore`: A/B in two flash blocks with sequence number and CRC written last (a power loss keeps the old or the new table) and a provisioning epoch. |
| Sessions | [session.hpp](../include/rtos/auth/session.hpp) | 256-bit tokens from the RNG (base64url, 43 characters), constant-time lookup, inactivity and absolute timeout, least-recently-used eviction, `SessionHandle {slot, generation}` for long-lived bindings (a WebSocket) that cannot inherit a reused slot. A login always issues a new token (session fixation). |
| Throttle | [throttle.hpp](../include/rtos/auth/throttle.hpp) | Per address: free failures, then exponential delay, then lockout; global limit against many addresses, which spares the trusted local interface. Never blocks: a refused attempt is answered at once (429), and no PBKDF2 runs for it. |
| Roles, presence | [access.hpp](../include/rtos/auth/access.hpp) | Observer < Operator < Programmer < Admin; a `Permission` is the least role plus "needs physical presence"; `decide()` also refuses everything but the password change to a login with a one-time password. `PresenceButton`: a press opens a window (one press, one critical action). |

`HttpsServer` ([https_server.hpp](../include/rtos/net/https_server.hpp)) routes requests under an API prefix to an `ApiHandler`: GET and POST with a Content-Length body (the parser refuses `Transfer-Encoding` and duplicate or non-numeric lengths), cross-origin POSTs are refused before the handler (login CSRF), an answer may set a cookie and is never cached (`Cache-Control: no-store`). A handler returns `Pending` for expensive work, which then runs as background steps (service.hpp fairness: one step per pass). `authorize_upgrade` decides the WebSocket upgrade after the Origin check (401). TCP connections know the interface their SYN arrived on (`arrival_interface()`), so "came in over USB" is not judged by addresses.

Login cost (`apps/authbench`, PBKDF2 with the target's SHA-256): **7000 instructions per iteration** in QEMU and Renode (250 MIPS, CPI 1: 28 µs per iteration as an emulator estimate, not a hardware measurement). The FCU uses 6000 iterations: about 42 M instructions, about 170 ms at 250 MIPS. On the hardware, flash wait states make it slower (to be measured with `fcu_authbench` on the board). The firmware runs 32 iterations per background step (about 0.9 ms).

Tests: `tests/auth_test.cpp` (RFC 7914/Python reference values, steps = one call, sessions with timeouts, wrap-around and fixation, throttle, roles and presence, credential store with power loss at every flash operation), `Http.*Body*` in `http_ws_test.cpp`, `HttpsApi.*` in `https_fairness_test.cpp` (background steps, Set-Cookie, CSRF, 405, upgrade consent after the Origin check). End to end and as an attacker: the FCU's `tools/test-auth-renode.py` and the login group of `tools/pentest-renode.py`.

## OPC UA trust

- Basic256Sha256 Sign or SignAndEncrypt with a client certificate that chains to the trusted CA (name constraints: DNS names under `.local`) and is valid at the current SNTP time: full access.
- SecurityPolicy None: read only.
- Self-signed or foreign client certificates and tampered signatures are rejected; nonces must not be reused.

## ARP hardening

The stack learns a MAC address only from the reply to its own request ([stack.hpp](../include/rtos/net/stack.hpp)). Unsolicited replies, gratuitous ARP and the sender of a request neither create nor change an entry. Under RFC 826 any station in the LAN could otherwise redirect the traffic to the gateway to itself (ARP spoofing). Attempts to change a known entry are counted in `StackStats::arp_ignored`. If a device really changes its MAC, the new one is used once the entry expires (5 min). `Cfg::arp_trust_unsolicited = true` restores the RFC 826 behaviour.

Tests: `ArpSpoofTest` in [net_test.cpp](../tests/net_test.cpp); all five fail with the RFC 826 behaviour.

The real protection still comes from the network and the protocols: Dynamic ARP Inspection on the switch, TLS, and OPC UA with signing and encryption.

## Robustness

Every parser that sees network or USB input is fuzzed (see [Testing](testing.md#fuzzing)): Ethernet/IP/TCP/UDP receive path with all services, HTTP, WebSocket, DNS names, DHCP options, Modbus PDU/RTU/TCP, OPC UA messages, USB SETUP packets and NTBs. On Linux the host tests also run under AddressSanitizer, and AFL++ fuzzes the parsers, the OPC UA server (secured channels included), the EtherCAT slave against a hostile master and the PLC image verifier ([fuzzing.md](fuzzing.md), with the findings so far).

A PLC image cannot switch off the runaway watchdog: its tasks' instruction budgets must stay within the VM's `max_budget` (default 2^24), or `load()` refuses the image.

EtherCAT has no authentication by design (ETG): whoever is the master on the segment may write every output and configuration object through CoE and the process data, exactly as Modbus TCP and the web UI may on the IP side. The slave's own memory stays safe against any SyncManager configuration: the mailbox and process data buffers have the sizes of the application's layout, not the master's.

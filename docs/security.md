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

- TLS 1.3 only (wolfSSL), X25519 and P-256 key exchange, ECDSA P-256 server certificate, ChaCha20-Poly1305 and AES-GCM.
- Handshakes and records are done entirely by wolfSSL, including KeyUpdate and close_notify.
- The static pool and the handshake limit keep a flood of connections from exhausting memory; idle handshakes are aborted after 10 s.
- WebSocket upgrades from foreign origins are rejected (403).

## OPC UA trust

- Basic256Sha256 Sign or SignAndEncrypt with a client certificate that chains to the trusted CA (name constraints: DNS names under `.local`) and is valid at the current SNTP time: full access.
- SecurityPolicy None: read only.
- Self-signed or foreign client certificates and tampered signatures are rejected; nonces must not be reused.

## ARP hardening

The stack learns a MAC address only from the reply to its own request ([stack.hpp](../include/rtos/net/stack.hpp)). Unsolicited replies, gratuitous ARP and the sender of a request neither create nor change an entry. Under RFC 826 any station in the LAN could otherwise redirect the traffic to the gateway to itself (ARP spoofing). Attempts to change a known entry are counted in `StackStats::arp_ignored`. If a device really changes its MAC, the new one is used once the entry expires (5 min). `Cfg::arp_trust_unsolicited = true` restores the RFC 826 behaviour.

Tests: `ArpSpoofTest` in [net_test.cpp](../tests/net_test.cpp); all five fail with the RFC 826 behaviour.

The real protection still comes from the network and the protocols: Dynamic ARP Inspection on the switch, TLS, and OPC UA with signing and encryption.

## Robustness

Every parser that sees network or USB input is fuzzed (see [Testing](testing.md#fuzzing)): Ethernet/IP/TCP/UDP receive path with all services, HTTP, WebSocket, DNS names, DHCP options, Modbus PDU/RTU/TCP, OPC UA messages, USB SETUP packets and NTBs.

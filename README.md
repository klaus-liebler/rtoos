# rtoos

Header-only C++26-RTOS mit IP-Stack, TLS 1.3, HTTPS und WebSocket für STM32H563 (Cortex-M33). Hardwarezugriffe laufen über das Concept `rtos::CpuPort`. Derselbe Code läuft deshalb auf dem Host (mit Mock), auf der Hardware und in den Emulatoren. Einziger unterstützter Compiler ist GCC 16 (siehe [CLAUDE.md](CLAUDE.md)).

```
include/rtos/                 portabler Kernel, keine Register
include/rtos/sync/            Semaphore, MessageQueue, ObjectPool, BlockPool
include/rtos/net/             Zero-Copy-IP-Stack (ARP/IPv4/ICMP/UDP/TCP), DHCP, DNS, mDNS, SNTP,
                              TLS (wolfSSL), HTTP, WebSocket, HTTPS-Server
src/net/                      wolfSSL-Anbindung (I/O, statischer Speicher, Entropie)
port/cortex_m33/              CPU-Port: PendSV/SVC, BASEPRI-Lock, PSPLIM, Lazy-FPU
port/stm32h563/               Chip: Vektortabelle (aus SVD generiert), Startup, Linkerskript,
                              RCC, GPIO, USART, ICACHE/DCACHE, MPU, RNG, Ethernet (EQoS)
boards/fcu/                   Board: Factory Control Unit (Takt, Konsole, LED, RMII/LAN8742A)
apps/demo/                    Demo-Threads
apps/selftest/                On-Target-Selbsttest des Schedulers
apps/net/                     HTTPS-Server mit Web-App und WebSocket (fcu_net)
apps/bench/                   Benchmark: Kosten der Kopien im TLS-Record-Pfad (fcu_bench)
web/index.html                Web-App (wird brotli-komprimiert per #embed eingebunden)
emulation/renode/             Renode-Plattform STM32H563 + Board FCU
emulation/qemu/               QEMU-Maschine "fcu" (Quellen, Patch, Build-Skript)
emulation/run.py              Runner für beide Emulatoren
emulation/net_test.py         Netzwerk-Ende-zu-Ende-Test in Renode (TAP)
tests/                        GoogleTest + MockPort (Host)
tools/gen_vectors.py          Vektortabellen-Generator (SVD → C++)
tools/make_ca.py              Projekt-CA erzeugen (ECDSA P-256, Name Constraints)
tools/make_certs.py           Server-Zertifikat, signiert von RTOS_CA_CERT/KEY oder einer lokalen Test-CA (→ certs/, nicht versioniert)
```

## Build und Tests

```
cmake --preset host && cmake --build --preset host && ctest --preset host
cmake --preset arm  && cmake --build --preset arm  && ctest --preset arm
```

`ctest --preset arm` lässt den Selbsttest in QEMU und in Renode laufen, außerdem den Netzwerktest `net_renode`. Der Netzwerktest wird übersprungen, wenn der TAP-Adapter fehlt.

Wichtig: immer `cmake --build --preset …` verwenden. Nur das Preset legt die GCC-DLLs in den PATH. `cmake --build build/host` scheitert ohne Fehlermeldung.

## Factory Control Unit

| | |
|---|---|
| MCU | STM32H563VIT6, ohne HSE-Quarz: HSI 64 MHz → PLL1 → SYSCLK 250 MHz (VOS0, 5 Waitstates) |
| Konsole | USART3, PD8 (TX) / PD9 (RX), 115200 8N1 → STDC14 (ST-LINK VCP) |
| LED_INFO | PH1, active low |

## Emulation

Beide Emulatoren bilden die FCU nach: Cortex-M33 mit FPU/MPU und DWT-Zykluszähler, Flash, SRAM, RCC, PWR (Spannungsskalierung), FLASH_ACR, GPIO und USART1–3. Die Konsole ist USART3. Cache-Register lesen 0 („nicht busy“), sodass die echten Treiber unverändert laufen.

Beide rechnen mit 250 MIPS, also einem Befehl pro Takt bei 250 MHz: Renode über `PerformanceInMips`, QEMU über `-icount shift=2,sleep=off`. SysTick und `DWT_CYCCNT` passen dadurch zusammen, die Werte sind aber nicht taktgenau. Sie zählen Befehle, nicht echte Takte mit Flash-Waitstates. Ohne `-icount` verliert QEMU unter Windows SysTick-Perioden, weil die Host-Uhr zu grob ist.

**Renode** (`C:\opt\renode`):

```
python emulation/run.py renode build/arm/apps/fcu_selftest.elf --allow-unsupported "stack limit"
python emulation/run.py renode build/arm/apps/fcu_demo.elf --interactive
```

Mit GUI: `renode` starten, dann `$bin=@build/arm/apps/fcu_demo.elf; include @emulation/renode/fcu.resc; showAnalyzer usart3; start`.

**QEMU** (eigener Build mit Maschine `fcu`, `C:\opt\qemu-stm32h5`):

```
python emulation/run.py qemu build/arm/apps/fcu_selftest.elf
C:\opt\qemu-stm32h5\qemu-system-arm.exe -M fcu -icount shift=2,sleep=off -nographic -kernel build\arm\apps\fcu_demo.elf -trace "led_*"
```

Neu bauen, z. B. für eine neue QEMU-Version:

```
set MSYSTEM=UCRT64& C:\opt\msys64\usr\bin\bash -l emulation/qemu/build-qemu.sh
```

**Bekannte Emulator-Grenzen:** Renode erzwingt das Stack-Limit (PSPLIM) nicht. Der Selbsttest meldet das als `UNSUPPORTED` statt als Fehler. Auf der Hardware muss der Test bestehen.

## Netzwerk, HTTPS und WebSocket (`fcu_net`)

Die Firmware stellt eine Web-App aus dem Flash bereit: `web/index.html` mit inline CSS, SVG und JS, brotli-komprimiert. Die App baut eine WebSocket-Verbindung auf und tauscht binäre Nachrichten von höchstens 512 Byte aus.

| Port | Dienst |
|---|---|
| 68/UDP | DHCP-Client ([dhcp.hpp](include/rtos/net/dhcp.hpp)): Adresse, Router, DNS- und NTP-Server; Erneuerung bei T1/T2. Ohne DHCP-Server nach 10 s statisch 192.168.77.2/24 |
| zufällig/UDP | DNS-Resolver ([dns_resolver.hpp](include/rtos/net/dns_resolver.hpp)): A-Records mit CNAME, Cache mit TTL, zufällige ID und Quellport pro Anfrage (RFC 5452) |
| 443/TCP | HTTPS (TLS 1.3, X25519/P-256, ChaCha20-Poly1305/AES-GCM): `/` (Web-App, `br`), `/blob` (100 KB Durchsatztest), `/ws` (WebSocket) |
| 5353/UDP | mDNS `fcu.local` (neu angekündigt, sobald eine Adresse da ist) |
| SNTP-Client | Server aus DHCP-Option 42, sonst per DNS `pool.ntp.org`; Port 12300 (im Test vom Host bereitgestellt, auf echten Netzen 123) |

WebSocket-Protokoll (binär, erstes Byte = Typ):

| Typ | Richtung | Inhalt |
|---|---|---|
| `0x01` | Gerät → Browser | Status alle 100 ms (47 Byte, LE): Uptime ms u32, Messwert i16, LED u8, Clients u8, Frames rx/tx u32, Unix-Zeit ms u64, dazu Messwerte: letzter TLS-Handshake in CPU-Takten u32, Verschlüsseln und Entschlüsseln in Takten/Byte ×16 u16, letztes Echo im Gerät in Takten u32, SYSCLK in MHz u16, feste Takte pro Record beim Ver- und Entschlüsseln u32. Die Kosten pro Record sind eine Ausgleichsgerade (Takte = fest + pro Byte × Bytes) über alle Records. Ein einfacher Mittelwert würde mit der Mischung der Record-Größen wandern. |
| `0x10` | Browser → Gerät | LED setzen (1 Byte: 0/1) |
| `0x20` / `0x21` | Browser → Gerät / zurück | Echo, Nutzlast bis 511 Byte |

Eigenschaften:

- **Speicher:** kein Heap. Paketpuffer und Ethernet-Deskriptoren liegen in `.dma_buffer`, einem nicht cachebaren MPU-Bereich. wolfSSL nutzt einen statischen Pool von 160 KB für alle Sessions (`WOLFSSL_STATIC_MEMORY`).
- **TLS:** Handshake und Records macht wolfSSL komplett, inklusive KeyUpdate und close_notify ([tls.hpp](include/rtos/net/tls.hpp)).
  - Ein `write()` ergibt einen Record (höchstens 4 KB). Eine WebSocket-Nachricht, also Header und Nutzlast zusammenhängend, ist immer genau ein Record.
  - Kopien: Beim Senden kopiert wolfSSL den Klartext in seinen Record-Puffer, verschlüsselt dort, und der I/O-Callback kopiert den Ciphertext in TCP-Segmente. Beim Empfangen läuft es umgekehrt.
  - Ein eigener Zero-Copy-Record-Layer wurde gebaut, gemessen und wieder entfernt, siehe unten.
- **Header davor ohne Kopie:** Jeder Sendepuffer kommt aus `Stack::allocate()` mit 64 Byte Platz vor den Nutzdaten. Ethernet, IPv4 und TCP brauchen 14 + 20 + 20 = 54 Byte. Jede Schicht verschiebt nur den Datenanfang nach vorn (`push_front`) und schreibt ihren Header dorthin.
- **Verbindungen:** 8 TCP-Verbindungen, 6 HTTPS-Sessions. Browser halten keep-alive-Verbindungen minutenlang offen und öffnen Reserveverbindungen.
  - Findet eine neue Verbindung keine freie Session, verdrängt sie die am längsten ruhende keep-alive-Session, statt abgewiesen zu werden. WebSockets und laufende Antworten werden nie verdrängt.
  - Ohne vollständigen Request schließt eine Session nach 10 s, ein unvollendeter Handshake bricht nach 10 s ab.
  - Nach close_notify und FIN ist die Session sofort frei; TCP gibt FIN_WAIT_2 nach 15 s auf.
  - Ein neuer WebSocket-Client bekommt den Status sofort, nicht erst im nächsten 100-ms-Takt.
- **Sofort senden:** Der Stack hat keinen Nagle-Algorithmus, jeder Record geht sofort mit PSH raus. WebSocket-Header und Nutzlast liegen immer in einem Record.
- **Krypto:** ChaCha20-Poly1305 wird bevorzugt, der H563 hat keinen AES-Beschleuniger. Für AES-GCM gibt es Tabellen (`GCM_TABLE_4BIT`). Thumb-2-Assembler wird für SP-ECC, ChaCha20, Poly1305 und SHA-2 genutzt.
- **Messwerte:** Die Firmware misst mit `DWT_CYCCNT` und zeigt die Werte in der Web-App und auf der Konsole. Die Zahlen aus den Emulatoren sind Befehlszahlen, nicht echte Takte.
- **Größe:** Flash ≈ 285 KB.

### Einmalig: TAP-Netz einrichten

Als Administrator ausführen:

```
powershell -ExecutionPolicy Bypass -File emulation\renode\setup-tap.ps1
```

Das Skript installiert den TAP-Treiber (tap-windows6) und legt den Adapter `rtoos-tap` mit 192.168.77.1/24 an. Außerdem erzeugt es eine Firewall-Regel, die nur für diesen Adapter gilt. Das Gerät bekommt 192.168.77.2 per DHCP vom Testskript, ohne Skript nach 10 s statisch.

### Zertifikate

Die Projekt-CA erzeugt `tools/make_ca.py`, einmalig:

```
python tools/make_ca.py C:\opt\msys64\ucrt64\bin\openssl.exe <Verzeichnis> --name rtoosCA
```

Ihr Profil:

- **Schlüssel:** ECDSA P-256 mit SHA-256. Das ist dieselbe Kurve, die Gerät und TLS ohnehin nutzen; die ganze Kette kommt ohne RSA aus.
- **Erweiterungen nach RFC 5280:** `basicConstraints` critical mit `CA:TRUE, pathlen:0`, `keyUsage` critical (keyCertSign, cRLSign) und ein Subject Key Identifier. Damit bestehen auch strenge Prüfer wie OpenSSL `-x509_strict` und Python ab 3.13.
- **Name Constraints (critical):** Die CA signiert nur Namen unter `.local` und private IPv4-Netze (10/8, 172.16/12, 192.168/16). Selbst mit dem Schlüssel lässt sich kein Zertifikat für öffentliche Namen fälschen.

`RTOS_CA_CERT` und `RTOS_CA_KEY` in `CMakePresets.json` zeigen auf diese CA. Bei jedem Configure prüft `tools/make_certs.py` das Server-Zertifikat streng gegen die CA und stellt es nur bei Bedarf neu aus: ECDSA P-256, SAN `fcu.local` und 192.168.77.2, 397 Tage gültig. Der CA-Schlüssel wird nur gelesen, nach `certs/` kommt nur `ca.crt`. Ohne die beiden Variablen entsteht eine lokale Test-CA mit demselben Profil in `certs/`.

### Automatischer Test

```
ctest --preset arm -R net_renode -V
```

Der Test prüft der Reihe nach:

- DHCP-Lease (DISCOVER/OFFER/REQUEST/ACK) und ihre Erneuerung bei T1
- DNS-Lookup des SNTP-Servers mit zufälligem Quellport
- Ping, mDNS und SNTP
- TLS-Handshake mit Prüfung von Zertifikat und Hostname gegen `certs/ca.crt`
- Web-App byte-genau, dazu 404 und 406
- `/blob` mit Durchsatz über eine Keep-alive-Verbindung
- WebSocket-Upgrade, Status, 100 Echos (1–511 Byte), LED-Befehl
- Abweisung fremder Origins (403)

### Mit dem Browser testen

1. Die CA muss im Browser bzw. in Windows vertrauenswürdig sein, einmalig für den eigenen Benutzer: `certutil -user -addstore Root certs\ca.crt`. Entfernen geht mit `certutil -user -delstore Root "rtoos FCU development CA"`.
2. Firmware in Renode starten, DHCP-, DNS- und SNTP-Server laufen mit:
   `python emulation/net_test.py --browser build/arm/apps/fcu_net.elf`
3. `https://fcu.local/` öffnen. Die Seite zeigt Laufzeit, SNTP-Uhrzeit, LED-Schalter, den Echo-Test und eine Live-Kurve.

Hinweise:

- **curl unter Windows:** Wegen des Schannel-Backends ist `--ssl-no-revoke` nötig, weil die CA keine Sperrliste veröffentlicht.
- **Strenge Prüfung:** `net_test.py` prüft mit `VERIFY_X509_STRICT` nach RFC 5280.

## Kernel-Dienste und Ausführungskontexte

| Dienst | Thread | Kernel-aware ISR (Prio ≥ 5) | Zero-Latency-ISR (Prio < 5) |
|---|---|---|---|
| `Kernel::notify` / `notify_from_isr` | ✓ | ✓ | ✓ (lock-frei) |
| `Kernel::resume` / `resume_from_isr` | ✓ | ✓ | – |
| `Kernel::defer` / `defer_from_isr` | ✓ | ✓ | ✓ (lock-frei) |
| `Semaphore::take`, `give` / `give_from_isr` | ✓ | give | – |
| `MessageQueue::send`, `receive` / `try_send_from_isr`, `try_receive_from_isr` | ✓ | try | – |
| `ObjectPool::make(timeout)` | ✓ (blockierend) | – | – |
| `ObjectPool::try_make`, `try_allocate` | ✓ | ✓ | ✓ (lock-frei) |
| `ObjectPool::destroy` / `destroy_from_isr` | ✓ | ✓ (direkte Übergabe) | ✓ (lock-frei, Wecken verzögert) |
| `BlockPool` | ✓ | ✓ | ✓ (lock-frei) |

- **Contracts:** Jeder `*_from_isr`-Dienst prüft, dass er im ISR läuft; die normalen Dienste prüfen das Gegenteil. Dienste, die die Kernel-Sperre nehmen, prüfen außerdem `Port::may_lock()` (ISR-Priorität nicht über der Kernel-Maske).
- **Kein „HigherPriorityTaskWoken“:** Jeder Dienst fordert den Kontextwechsel (PendSV) selbst an, wenn sich die Auswahl ändern kann.
- **Wartereihenfolge:** Blockierte Threads werden in Ankunftsreihenfolge (FIFO) bedient. Freigegebene Einheiten (Semaphore, Pool-Block, Nachricht) gehen direkt an den ersten Wartenden.
- **Zwei Phasen:** Blockierende Aufrufe gibt es auch als `*_begin` plus `Kernel::wait_end()`. So lassen sie sich auf dem Host testen.

## Wichtige Regeln

- **Interrupt-Prioritäten:** ISRs mit Priorität `< RTOS_CM33_KERNEL_PRIORITY` (Default 5) werden nie maskiert. Sie dürfen nur die lock-freien Dienste aus der Tabelle oben verwenden.
- **Blockierende Aufrufe:** Sie bestehen aus `*_begin` und `*_end`. Auf Hardware erfolgt der Kontextwechsel dazwischen. Host-Tests rufen beide Phasen explizit auf.
- **Preemption-Threshold:** Threads, die Daten teilen, bekommen den gleichen Threshold (Ceiling). Sie unterbrechen sich dann nie gegenseitig und brauchen keinen Mutex.
- **Contracts:** Der Host nutzt `enforce` (Meldung über den Standard-Handler). Das Target nutzt `quick_enforce` (`std::terminate` → eigenes `abort()` → Breakpoint).

## Zero-Copy im TLS-Record-Layer: gemessen und verworfen (`fcu_bench`)

Zeitweise gab es einen eigenen TLS-1.3-Record-Layer. Er übernahm nach dem Handshake die Schlüssel von wolfSSL, verschlüsselte direkt in die Paketpuffer und entschlüsselte im Empfangspuffer. Damit er sich lohnt, muss er mehr bringen als die Kopien kosten. `apps/bench` misst genau das: Es erzeugt denselben ChaCha20-Poly1305-Record einmal ohne und einmal mit den Kopien des wolfSSL-Pfads. Die Krypto ist in beiden Fällen identisch, wolfSSLs eigene Record-Verwaltung ist nicht enthalten.

```
python emulation/run.py qemu build/arm/apps/fcu_bench.elf --timeout 30
```

Ergebnis in den Emulatoren (Befehle, nicht echte Takte; auf der Platine liefert dieselbe Firmware echte Takte):

| Record | ohne Kopie | mit Kopien | Anteil der Kopien |
|---|---|---|---|
| senden 39 B | 4 392 | 4 535 | 3,1 % |
| senden 512 B | 15 945 | 16 618 | 4,0 % |
| senden 1438 B (aus Flash) | 36 699 | 38 400 | 4,4 % |
| empfangen 39 B | 4 401 | 4 594 | 4,2 % |
| empfangen 512 B | 16 002 | 16 959 | 5,6 % |
| empfangen 1438 B | 36 735 | 38 748 | 5,1 % |

Gründe für die Entscheidung:

- **Kleiner Gewinn:** Die Kopien kosten 3–6 % der Record-CPU. wolfSSL kommt für seinen eigenen Zero-Copy-Sendepfad auf dieselbe Größenordnung: +1,0 % bei ChaCha20, +2,9 % bei AES-GCM ([PR 10899](https://github.com/wolfSSL/wolfssl/pull/10899)). Dieser Pfad gilt nur für TLS 1.2 und ist bei `WOLFSSL_NO_TLS12` aus.
- **Bei großen Downloads sogar schlechter:** Ein Record pro Segment kostete mehr feste Kosten pro Record (rund 4 500 Befehle), als die gesparten Kopien einbrachten.
- **Durchsatz:** Bei 100 Mbit/s ist die Leitung der Engpass, nicht die CPU.
- **Sicherheit und Wartung:** Ein eigener Record-Layer (Nonces, Sequenznummern, KeyUpdate, Tags) wären rund 700 Zeilen sicherheitskritischer Code, die man selbst pflegen müsste. wolfSSL wird von anderen gepflegt und bei Lücken repariert.
- **Im Gesamtsystem:** Ein WebSocket-Echo kostet mit wolfSSL-Records 37 700 statt 34 000 Befehle (151 statt 136 µs bei 250 MHz).

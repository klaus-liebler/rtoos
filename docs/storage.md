# Storage: SD cards, FAT and littlefs in flash

[include/rtos/storage/sd_card.hpp](../include/rtos/storage/sd_card.hpp) implements the SD memory card protocol (SD Physical Layer Simplified Specification) over any host controller that satisfies the `SdHost` concept. [port/stm32h563/.../sdmmc.hpp](../port/stm32h563/include/rtos/port/stm32h5/sdmmc.hpp) is the STM32H5 host controller.

```cpp
using Host = h5::Sdmmc<h5::SdmmcId::Sdmmc1, {.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz}>;
Host host;
rtos::storage::SdCard<Host> card{host};

board::init_sdcard();                         // FCU: SDMMC1 1-bit (D0 PA10, CK PC12, CMD PD2)
if (card.init() == rtos::storage::SdStatus::Ok) {
    card.read(block, buffer);                 // whole 512-byte blocks, word-aligned buffer
    card.write(block, data);                  // returns once the card has programmed the data
}
```

## Protocol

- **Initialisation:** CMD0, CMD8 (version 2 cards echo the check pattern, version 1 cards don't answer), ACMD41 with HCS until the card is ready (up to about 1 s), CMD2, CMD3, CMD9 (the CSD gives the capacity, CSD 1.0 and 2.0), CMD7, CMD16 for byte-addressed SDSC cards. After that the transfer clock is switched on.
- **Addressing:** block numbers for SDHC/SDXC, byte addresses for SDSC. The caller always uses block numbers.
- **Transfers:** CMD17/CMD24 for one block, CMD18/CMD25 plus CMD12 for several. After a write, CMD13 polls until the card is back in the transfer state.
- **Errors:** `SdStatus` (timeout, CRC error, card error, no card, unsupported card, not ready). Partial blocks and ranges beyond the card are rejected before anything is sent.

## STM32H5 host controller

- **Configuration at compile time:** the clock dividers for identification (≤ 400 kHz) and transfer (≤ 25 MHz, default speed) are computed and checked from the kernel clock (PLL1 Q). On the FCU this gives 400 kHz and 25 MHz.
- **Data by IDMA:** the SDMMC's internal DMA moves data directly between the card and the caller's buffer, with no CPU copy and no FIFO handling.
- **Busy:** after R1b responses the driver polls the D0 line (STAR.BUSYD0).
- **Polled:** commands and data wait on the DWT cycle counter. A 512-byte block at 25 MHz on a 1-bit bus takes about 0.2 ms.

## littlefs in the MCU's flash

[include/rtos/storage/littlefs.hpp](../include/rtos/storage/littlefs.hpp) runs [littlefs](https://github.com/littlefs-project/littlefs) (BSD-3; [cmake/littlefs.cmake](../cmake/littlefs.cmake), no heap, no debug output) on any `FlashMemory`. It's a fail-safe file system: a power loss at any point leaves the last committed state. It also does wear levelling. Use it for settings, recipes and missions in the MCU's own flash:

```cpp
namespace flash = rtos::port::stm32h5::flash;
using DataFlash = flash::Region<0x081C'0000, 256 * 1024, board::sysclk_hz>;  // last 32 sectors of bank 2
DataFlash region;
rtos::storage::LittleFs<DataFlash> fs{region};              // link rtos_littlefs
if (fs.mount() != LFS_ERR_OK) { fs.format(); fs.mount(); }
decltype(fs)::File f;                                        // the file's cache lives here
fs.open(f, "mission.bin", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
lfs_file_write(fs.get(), f.get(), data, size);
lfs_file_close(fs.get(), f.get());
```

- **Flash programming** ([flash_program.hpp](../port/stm32h563/include/rtos/port/stm32h5/flash_program.hpp)): `flash::Writer` unlocks the flash for its lifetime. It erases 8 KB sectors and programs quad-words (16 bytes, once per erase), and invalidates the ICACHE afterwards. It refuses to work when the banks are swapped (SWAP_BANK). Erasing or programming the bank the CPU runs from stalls the CPU; a region in bank 2 keeps code in bank 1 running.
- **`flash::Region<Base, Bytes, CpuHz>`:** a data area checked at compile time to be whole sectors, inside one bank and inside the flash. Keep the image out of it with `rtos_add_firmware(... FLASH_DATA_START <Base>)`: the link fails if the code reaches into the area.

## Tests

- **Host** ([sd_card_test.cpp](../tests/sd_card_test.cpp)): the protocol against a simulated card. It covers SDHC, SDSC v2 and v1, a missing card, a card that never gets ready, programming polls after a write, multi-block transfers with CMD12, rejected ranges and CSD capacity.
- **Host** ([filex_media_test.cpp](../tests/filex_media_test.cpp)): FileX on a RAM disk. It covers FAT16 and FAT32 round-trips across a remount, the first partition of an MBR disk (MBR untouched), and unaligned caller buffers (the RAM disk refuses them, as IDMA does).
- **Renode** ([sdcard_test.py](../emulation/sdcard_test.py)): driver, protocol and file system against the SDMMC + card model with real image files ([Testing](testing.md#sd-card-test-renode)).
- **Host** ([littlefs_test.cpp](../tests/littlefs_test.cpp)): littlefs on a simulated NOR flash with the H5's rules (erase to 0xFF, each quad-word programmed once). It checks files and directories across a remount, wear levelling over 400 rewrites in 8 sectors, and a power loss after every single program operation of a rewrite (the file is always entirely old or entirely new).
- **Renode** (`ctest -R flashfs_renode`): [`fcu_flashfs`](../apps/flashfs/main.cpp) formats littlefs in the last 256 KB of bank 2, writes a mission file and 100 counter rewrites (sector erases), remounts, reads back, and finds the superblock in the flash.

## File system: FileX (FAT12/16/32)

[include/rtos/storage/filex_media.hpp](../include/rtos/storage/filex_media.hpp) runs [FileX](https://github.com/eclipse-threadx/filex) (Eclipse ThreadX, MIT; fetched and pinned by [cmake/filex.cmake](../cmake/filex.cmake), configured by [third_party/filex/fx_user.h](../third_party/filex/fx_user.h)) on any `BlockDevice`, such as an `SdCard` or a RAM disk:

```cpp
rtos::storage::FileXMedia<rtos::storage::SdCard<Host>> volume{card};    // link rtos_filex
if (volume.open() != FX_SUCCESS) {        // cards come formatted
    volume.format_fat("FCU");             // FAT12/16/32 by cluster count, no partition table
    volume.open();
}
FX_FILE file;
fx_file_create(volume.media(), const_cast<CHAR*>("log.txt"));
fx_file_open(volume.media(), &file, const_cast<CHAR*>("log.txt"), FX_OPEN_FOR_WRITE);
fx_file_write(&file, data, size);
fx_file_close(&file);
volume.flush();
```

- **Standalone:** no ThreadX, no locks. One thread owns a volume. Dates and times come from the application (`fx_system_date_set`, `fx_system_time_set`), not from a timer.
- **Partitions:** a card with a partition table is opened at its first partition, an unpartitioned card at sector 0. The driver takes the offset from the MBR, not from the boot sector's hidden-sector count. `format_fat` formats the whole device without a partition table.
- **Alignment:** FileX transfers directly from and to the caller's buffers. Buffers that aren't word-aligned go through a bounce sector, because the SDMMC's IDMA needs word alignment.
- **No exFAT:** the open-source FileX has no exFAT. SDXC cards (over 32 GB, exFAT from the factory) must be formatted with `format_fat` (FAT32).

/* SPDX-License-Identifier: MIT */
/* FileX configuration for rtoos (FX_INCLUDE_USER_DEFINE_FILE). */
#ifndef FX_USER_H
#define FX_USER_H

/* No ThreadX: single-threaded, no internal locks - one thread owns each medium. */
#define FX_STANDALONE_ENABLE
/* No periodic timer: dates and times come from the application (fx_system_date_set/time_set). */
#define FX_NO_TIMER

/* FAT12/16/32 only: the open-source FileX (Eclipse ThreadX) has no exFAT. SDXC cards come
   formatted with exFAT and must be reformatted with FAT32 (FileXMedia::format_fat). */

#define FX_MAX_LONG_NAME_LEN 256
#define FX_MAX_LAST_NAME_LEN 256
/* Logical sector cache per medium: FX_MAX_SECTOR_CACHE sectors in the media buffer. */
#define FX_MAX_SECTOR_CACHE 16

#endif /* FX_USER_H */

/* SPDX-License-Identifier: MIT
 *
 * wolfSSL configuration for rtoos: TLS 1.3 server, no heap, no file system.
 * Based on wolfSSL's examples/configs/user_settings_tls13.h and
 * user_settings_stm32.h (wolfSSL 5.9.4).
 *
 *   key exchange   X25519, ECDHE P-256          (browsers offer X25519 first)
 *   certificate    ECDSA P-256 / SHA-256 (TLS); RSA-2048 only for OPC UA
 *   ciphers        TLS_CHACHA20_POLY1305_SHA256 (constant time, fast without AES HW)
 *                  TLS_AES_128_GCM_SHA256 / TLS_AES_256_GCM_SHA384
 *                  (AES T-tables + 4-bit GHASH tables: lookup-table speed)
 *   memory         WOLFSSL_STATIC_MEMORY + WOLFSSL_NO_MALLOC: every allocation comes
 *                  from the bucketed pool handed to wolfSSL_CTX_load_static_memory()
 *   entropy        rtos_tls_seed() (STM32H5 TRNG on the target)
 *   time           none (no certificate validation on the server, no tickets)
 *
 * Target (Cortex-M33): Thumb-2 inline assembly for AES, SHA-256/512, ChaCha20,
 * Poly1305, X25519 and SP P-256 (constant time, ECC_TIMING_RESISTANT).
 */
#ifndef RTOS_WOLFSSL_USER_SETTINGS_H
#define RTOS_WOLFSSL_USER_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- platform -------------------------------------------------------------------- */
#define SINGLE_THREADED
#define NO_FILESYSTEM
#define NO_WRITEV
#define WOLFSSL_USER_IO
#define WOLFSSL_NO_SOCK
#define WOLFSSL_IGNORE_FILE_WARN
#define NO_MAIN_DRIVER
#define BENCH_EMBEDDED
#define WOLFSSL_SMALL_STACK          /* large temporaries come from the static pool */
#define NO_ASN_TIME                  /* the server validates no certificates */
#define USER_TICKS                   /* LowResTimer() provided by rtoos */
#define WOLFSSL_GENERAL_ALIGNMENT 4
#define WOLFSSL_NOT_WINDOWS_API      /* host tests on Windows: bare platform, no Win32 API */
#define NO_OLD_SHA_NAMES             /* no Sha256/SHA256_DIGEST_SIZE macros: they clash with */
#define NO_OLD_WC_NAMES              /* the names of the OPC UA library (rtos/opcua/) */

/* ---- memory: static pool, no malloc ---------------------------------------------- */
#define WOLFSSL_STATIC_MEMORY
#define WOLFSSL_NO_MALLOC

/* ---- entropy ------------------------------------------------------------------------ */
#define HAVE_HASHDRBG
extern int rtos_tls_seed(unsigned char* output, unsigned int size);
#define CUSTOM_RAND_GENERATE_SEED rtos_tls_seed

/* ---- protocol ----------------------------------------------------------------------- */
#define WOLFSSL_TLS13
#define WOLFSSL_NO_TLS12
#define NO_OLD_TLS
#define HAVE_TLS_EXTENSIONS
#define HAVE_SUPPORTED_CURVES
#define HAVE_HKDF
#define HAVE_SNI
#define HAVE_ENCRYPT_THEN_MAC
#define WOLFSSL_NO_TLS13_EARLY_DATA

/* The host build keeps the client: unit tests run a wolfSSL client against our server. */
#if defined(__arm__)
    #define NO_WOLFSSL_CLIENT
#endif

/* ---- algorithms ----------------------------------------------------------------------- */
#define HAVE_ECC
#define ECC_USER_CURVES              /* P-256 only */
#define ECC_TIMING_RESISTANT
#define TFM_TIMING_RESISTANT
#define HAVE_CURVE25519
#define HAVE_AESGCM
#define GCM_TABLE_4BIT
#define HAVE_CHACHA
#define HAVE_POLY1305
#define HAVE_ONE_TIME_AUTH
#define WOLFSSL_SHA384
#define WOLFSSL_SHA512
#define WOLFSSL_ASN_TEMPLATE
#define WOLFSSL_BASE64_ENCODE        /* WebSocket handshake */
/* RSA-2048 only for OPC UA (SecurityPolicy Basic256Sha256: OAEP-SHA1, PKCS#1 v1.5 SHA-256;
 * the only exception to the ECDSA-only rule, see CLAUDE.md). TLS stays ECDSA P-256: the
 * HTTPS server presents its P-256 certificate and requests no client certificate. */
#define WC_RSA_BLINDING              /* private-key operations resist timing attacks */
#define WC_RSA_PSS                   /* required by wolfSSL's TLS 1.3 code once RSA exists */
#define NO_DH
#define NO_DSA
#define NO_RC4
#define NO_MD4
#define NO_MD5
#define NO_DES3
#define NO_DES3_TLS_SUITES
#define NO_PSK
#define NO_PWDBASED
#define NO_SESSION_CACHE
#define NO_ERROR_STRINGS

/* ---- math ------------------------------------------------------------------------------- */
#define WOLFSSL_SP_MATH
#define WOLFSSL_HAVE_SP_ECC
#define WOLFSSL_HAVE_SP_RSA          /* RSA-2048 (OPC UA) */
#define WOLFSSL_SP_NO_3072
#define WOLFSSL_SP_NO_MALLOC
#define SP_INT_BITS 2048

#if defined(__arm__)
    /* Cortex-M33 (ARMv8-M Mainline, Thumb-2, DSP) */
    #define WOLFSSL_SP_ASM
    #define WOLFSSL_SP_ARM_CORTEX_M_ASM
    #define WOLFSSL_ARMASM
    #define WOLFSSL_ARMASM_INLINE
    #define WOLFSSL_ARMASM_THUMB2
    #define WOLFSSL_ARMASM_NO_HW_CRYPTO
    #define WOLFSSL_ARMASM_NO_NEON
#endif

#ifdef __cplusplus
}
#endif

#endif /* RTOS_WOLFSSL_USER_SETTINGS_H */

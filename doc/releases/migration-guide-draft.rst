:orphan:

..
  See
  https://docs.zephyrproject.org/latest/releases/index.html#migration-guides
  for details of what is supposed to go into this document.

.. _migration_4.6:

Migration guide to Zephyr v4.6.0 (Working Draft)
################################################

This document describes the changes required when migrating your application from Zephyr v4.5.0 to
Zephyr v4.6.0.

Any other changes (not directly related to migrating applications) can be found in
the :ref:`release notes<zephyr_4.6>`.

.. contents::
    :local:
    :depth: 2

Common
******

Build System
************

Kernel
******

Boards
******

Device Drivers and Devicetree
*****************************

.. Only place contents common to all device drivers here. Contents specific to one driver subsystem
   goes into its own subsection, below.

.. Group contents in this section by subsystem, e.g.:
..
.. ADC
.. ===
..
.. ...

.. zephyr-keep-sorted-start re(^\w) ignorecase

.. zephyr-keep-sorted-stop

Bluetooth
*********

Networking
**********

Other subsystems
****************

Modules
*******

HostAP
======

* RSA support is now only enabled by
  :kconfig:option:`CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE`. Without it,
  :kconfig:option:`CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ALT` and
  :kconfig:option:`CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_MBEDTLS_PSA` no longer select the
  ``TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256`` and ``TLS_ECDHE_PSK_WITH_AES_256_CBC_SHA384``
  ciphersuites or the PSA RSA key types. This also drops what those options enabled indirectly:
  the ECDHE-RSA and ECDHE-PSK key exchanges, TLS 1.2, X.509 certificate parsing and the
  RSA PKCS#1 v1.5, RSA-PSS and ECDSA PSA algorithms (ECDSA is still enabled when DPP or
  enterprise support is enabled). Applications that use TLS or X.509 themselves and relied on
  the Wi-Fi supplicant to enable these options must now enable them explicitly, for example
  :kconfig:option:`CONFIG_MBEDTLS_X509_CRT_PARSE_C`,
  :kconfig:option:`CONFIG_MBEDTLS_CIPHERSUITE_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256` and
  :kconfig:option:`CONFIG_PSA_WANT_KEY_TYPE_RSA_PUBLIC_KEY`.

Architectures
*************

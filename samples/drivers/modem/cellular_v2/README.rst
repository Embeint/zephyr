.. zephyr:code-sample:: cellular_v2
   :name: Cellular modem state machine driver
   :relevant-api: cellular_interface modem_cellular_backend

   Control a cellular modem directly, then let its network interface bring it up by itself.

Overview
********

This sample uses the cellular modem driver built on the State Machine Framework,
:kconfig:option:`CONFIG_MODEM_CELLULAR_V2`, in two phases, with the modem fully powered down in
between.

The first phase controls the modem directly through its core device, only using the PPP network
interface once the modem is registered:

#. Powers the modem up by taking a runtime PM reference on it with
   :c:func:`pm_device_runtime_get`.
#. Waits for :c:enumerator:`CELLULAR_EVENT_MODEM_READY`, then prints the modem and SIM identity.
   The LTE RF is still disabled at this point.
#. Selects the APN from the home network in the SIM's IMSI, from a short example table, falling
   back to :kconfig:option:`CONFIG_MODEM_CELLULAR_APN`, and sets it with
   :c:func:`cellular_set_apn`. An empty APN has the network select the default APN of the
   subscription.
#. Requests the LTE RF with :c:func:`modem_cellular_lte_rf_request`, which configures the data
   context with the APN and enables the LTE RF, then waits for the modem to register with the
   network.
#. Brings the PPP network interface up with :c:func:`net_if_up`, which dials the data call on the
   registered modem, resolves :kconfig:option:`CONFIG_SAMPLE_DNS_HOSTNAME` with the DNS servers
   PPP negotiates, then takes the interface down with :c:func:`net_if_down`. The interface holds
   its own LTE RF request, so the modem stays registered while the sample holds its request.
#. Releases the LTE RF with :c:func:`modem_cellular_lte_rf_release` and powers the modem down
   with :c:func:`pm_device_runtime_put`, which disables the LTE RF before the modem is shut down.

The second phase only brings the PPP network interface up with :c:func:`net_if_up`. The driver
then powers the modem up, enables the LTE RF with the APN set in the first phase and dials the
data call by itself. Once PPP has negotiated an IPv4 address, the sample resolves
:kconfig:option:`CONFIG_SAMPLE_DNS_HOSTNAME` again, then keeps the call up for
:kconfig:option:`CONFIG_SAMPLE_CONNECTED_TIME_S` seconds, then takes the interface down with
:c:func:`net_if_down`, which hangs up and powers the modem down again. See
:zephyr:code-sample:`cellular-modem` for a sample that uses the interface for traffic.

Requirements
************

A board with a cellular modem whose vendor driver supports
:kconfig:option:`CONFIG_MODEM_CELLULAR_V2`, referenced by the ``modem`` devicetree alias, and a SIM
card. The sample runs on the :zephyr:board:`nrf93m1dk`.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/modem/cellular_v2
   :board: nrf93m1dk/nrf54l15/cpuapp
   :goals: build flash
   :compact:

Set :kconfig:option:`CONFIG_MODEM_CELLULAR_APN` to the APN of the SIM card's network if it is
not in the sample's table and the network does not provide a default APN.

# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(nau7802.c)
zephyr_library_sources_ifdef(CONFIG_NAU7802 nau7802.c)
zephyr_library_sources_ifdef(CONFIG_NAU7802_TRIGGER nau7802_trigger.c)

if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_ALT_TAB_ENCODER app PRIVATE behaviors/behavior_alt_tab_encoder.c)
endif()
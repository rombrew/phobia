SH_DEF(flash_prog)
SH_DEF(flash_info)
SH_DEF(flash_wipe)
SH_DEF(ap_version)
SH_DEF(ap_gettick)
SH_DEF(ap_dbg_task)
SH_DEF(ap_dbg_heap)
SH_DEF(ap_dbg_hexdump)
SH_DEF(ap_log_flush)
SH_DEF(ap_log_clean)
SH_DEF(ap_reboot)
SH_DEF(ap_bootload)
SH_DEF(pm_probe_impedance)
SH_DEF(pm_probe_saturation)
SH_DEF(pm_probe_spinup)
SH_DEF(pm_probe_detached)
SH_DEF(pm_probe_const_flux_linkage)
SH_DEF(pm_probe_const_inertia)
SH_DEF(pm_probe_threshold_tol)
SH_DEF(pm_adjust_sensor_hall)
SH_DEF(pm_adjust_sensor_eabi)
SH_DEF(pm_adjust_sensor_sincos)
SH_DEF(ld_probe_const_inertia)
SH_DEF(ld_adjust_limit)
SH_DEF(pm_fsm_detached)
SH_DEF(pm_fsm_startup)
SH_DEF(pm_fsm_shutdown)
SH_DEF(pm_default_config)
SH_DEF(pm_default_machine)
SH_DEF(pm_default_scale)
SH_DEF(tlm_default)
SH_DEF(tlm_grab)
SH_DEF(tlm_watch)
SH_DEF(tlm_stop)
SH_DEF(tlm_clean)
SH_DEF(tlm_flush_sync)
SH_DEF(tlm_stream_sync)
#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(tlm_stream_net_async)
#endif /* HW_HAVE_NETWORK_EPCAN */
SH_DEF(help)
#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_survey)
#endif /* HW_HAVE_NETWORK_EPCAN */
#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_assign)
#endif /* HW_HAVE_NETWORK_EPCAN */
#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_revoke)
#endif /* HW_HAVE_NETWORK_EPCAN */
#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_node_remote)
#endif /* HW_HAVE_NETWORK_EPCAN */
#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_node_data)
#endif /* HW_HAVE_NETWORK_EPCAN */
SH_DEF(pm_self_test)
SH_DEF(pm_self_adjust)
SH_DEF(pm_adjust_dcu_voltage)
SH_DEF(pm_scan_impedance)
SH_DEF(hal_ADC_scan)
SH_DEF(hal_PWM_set_DC)
SH_DEF(hal_PWM_set_Z)
#ifdef HW_HAVE_FAN_CONTROL
SH_DEF(hal_FAN_control)
#endif /* HW_HAVE_FAN_CONTROL */
SH_DEF(hal_DBGMCU_mode_stop)
SH_DEF(reg)
SH_DEF(enum_reg)
SH_DEF(config_reg)

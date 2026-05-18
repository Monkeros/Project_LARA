"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 *
 * - Type: Continuous Scanner Driver
 * - Method for continuous measuring mode, where the rotary table
 * - rotates continuously and the scanner captures profiles in real-time.
 *
 * - Description: 
 *   Hardware abstraction layer containing driver for the Micro-Epsilon scanCONTROL laser scanner.
 *   It handles low-level communication via C++ API (ctypes) and serial interface, including 
 *   data extraction, ROI filtering, and hardware error management.
 *
 * - Dependency Notice: 
 *   This software uses the manufacturer-provided Micro-Epsilon
 *   scanCONTROL SDK C++ API (LLT.dll) for scanner communication and control.
 *   All vendor-provided libraries and source files remain the property of
 *   Micro-Epsilon Messtechnik GmbH & Co. KG and are subject to their licensing terms.
 *
"""



import ctypes as ct
import numpy as np
import config
import time
import pyllt as llt
from .llt_datatypes import *



class ScannerDriverContinuous:
    def __init__(self):
        self.hLLT = llt.create_llt_device(TInterfaceType.INTF_TYPE_ETHERNET)

        available_interfaces = (ct.c_uint * 6)()
        llt.get_device_interfaces_fast(self.hLLT, available_interfaces, len(available_interfaces))

        llt.set_device_interface(self.hLLT, available_interfaces[0], 0)

        if llt.connect(self.hLLT) < 1:
            raise ConnectionError("Unable to establish a connection to the sensor via Ethernet.")

        self.scanner_type = ct.c_int(0)
        llt.get_llt_type(self.hLLT, ct.byref(self.scanner_type))

        # Nastavení rozlišení (binning) - stejná logika jako ScannerDriverDiscrete
        available_resolutions = (ct.c_uint * 4)()
        llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))

        binning_index = getattr(config, 'SKENER_BINNING', 0)
        if binning_index < 0 or binning_index >= len(available_resolutions):
            raise ValueError(
                f"SKENER_BINNING index {binning_index} out of range "
                f"(0-{len(available_resolutions) - 1})."
            )

        self.resolution = available_resolutions[binning_index]
        if self.resolution == 0:
            raise ValueError(
                f"Resolution at SKENER_BINNING index {binning_index} is 0 - invalid."
            )

        res_set = llt.set_resolution(self.hLLT, self.resolution)
        if res_set < 1:
            raise ValueError(f"Error setting resolution. Code: {res_set}")

        print(f"[SKENER] Resolution (binning index {binning_index}): {self.resolution} px.")

        llt.set_profile_config(self.hLLT, TProfileConfig.PROFILE)

        self._aplikovat_konfiguraci()

        self.profile_buffer_size = self.resolution * 64
        self.profile_buffer = (ct.c_ubyte * self.profile_buffer_size)()
        self.x_out = (ct.c_double * self.resolution)()
        self.z_out = (ct.c_double * self.resolution)()

        self.null_ushort = ct.POINTER(ct.c_ushort)()
        self.null_uint = ct.POINTER(ct.c_uint)()
        self.last_profile_count = -1

        # instrukce pro LLT.dll k zachytavani kontinualniho UDP streamu do RAM
        llt.set_hold_buffers_for_polling(self.hLLT, 20000)

    def _aplikovat_konfiguraci(self):
        if not hasattr(config, 'SKENER_FREKVENCE') or not hasattr(config, 'SKENER_EXPOZICE'):
            return

        perioda_us = 1000000.0 / config.SKENER_FREKVENCE
        idle_time_us = perioda_us - config.SKENER_EXPOZICE

        if idle_time_us < 0:
            raise ValueError(
                f"Frequency {config.SKENER_FREKVENCE} Hz (period {perioda_us:.0f} us) "
                f"cannot be achieved with exposure {config.SKENER_EXPOZICE} us."
            )

        # převod z us na kroky (1 krok = 10 us, rozsah 1-4095)
        exposure_kroky = int(config.SKENER_EXPOZICE / 10)
        idle_kroky = int(idle_time_us / 10)

        if exposure_kroky < 1 or exposure_kroky > 4095:
            raise ValueError(f"EXPOSURE_TIME out of range: {exposure_kroky} steps (1-4095). "
                            f"Adjust SKENER_EXPOZICE (valid range: 10-40950 us).")
        if idle_kroky < 1 or idle_kroky > 4095:
            raise ValueError(f"IDLE_TIME out of range: {idle_kroky} steps (1-4095). "
                            f"Adjust SKENER_FREKVENCE or SKENER_EXPOZICE.")

        res_exp = llt.set_feature(self.hLLT, FEATURE_FUNCTION_EXPOSURE_TIME, exposure_kroky)
        if res_exp < 1:
            raise ValueError(f"Error writing EXPOSURE_TIME. Code: {res_exp}")
        exp_readback = ct.c_uint(0)
        llt.get_feature(self.hLLT, FEATURE_FUNCTION_EXPOSURE_TIME, ct.byref(exp_readback))
        print(f"[SKENER] EXPOSURE_TIME set: {exposure_kroky} steps ({exposure_kroky * 10} us) | "
            f"readback: {exp_readback.value} steps ({exp_readback.value * 10} us)")

        res_idle = llt.set_feature(self.hLLT, FEATURE_FUNCTION_IDLE_TIME, idle_kroky)
        if res_idle < 1:
            raise ValueError(f"Error writing IDLE_TIME. Code: {res_idle}")
        idle_readback = ct.c_uint(0)
        llt.get_feature(self.hLLT, FEATURE_FUNCTION_IDLE_TIME, ct.byref(idle_readback))
        print(f"[SKENER] IDLE_TIME set: {idle_kroky} steps ({idle_kroky * 10} us) | "
            f"readback: {idle_readback.value} steps ({idle_readback.value * 10} us)")

        if hasattr(config, 'SKENER_LASER_POWER'):
            res_laser = llt.set_feature(self.hLLT, FEATURE_FUNCTION_LASER, int(config.SKENER_LASER_POWER))
            if res_laser < 1:
                raise ValueError(f"Error writing LASER_POWER. Code: {res_laser}")

        skutecna_frekvence = 1000000.0 / ((exp_readback.value + idle_readback.value) * 10) \
            if (exp_readback.value + idle_readback.value) > 0 else 0
        print(f"[SKENER] Target freq: {config.SKENER_FREKVENCE} Hz | "
            f"Effective freq from readback: {skutecna_frekvence:.1f} Hz")

    def start_akvizice(self):
        self.last_profile_count = -1  # reset čítače při každém spuštění
        res = llt.transfer_profiles(self.hLLT, TTransferProfileType.NORMAL_TRANSFER, 1)
        if res < 1:
            raise RuntimeError(f"Failed to start continuous profile transfer. Code: {res}")
        time.sleep(0.5)  # počká než skener začne měřit
        print(f"[SKENER] Transfer started. Code: {res}")

    def ziskat_profil(self):
        lost_profiles = ct.c_int()

        bytes_read = llt.get_actual_profile(
            self.hLLT, self.profile_buffer, self.profile_buffer_size,
            TProfileConfig.PROFILE, ct.byref(lost_profiles)
        )

        if bytes_read <= 0:
            return np.array([], dtype=np.float32), np.array([], dtype=np.float32)

        # OPRAVA: detekce duplikátů přes profile_count odstraněna
        # DLL sama zajišťuje že get_actual_profile vrátí -104 pokud není nový profil

        res = llt.convert_profile_2_values(
            self.hLLT, self.profile_buffer, self.resolution, TProfileConfig.PROFILE,
            TScannerType(self.scanner_type.value), 0, 1,
            self.null_ushort, self.null_ushort, self.null_ushort,
            self.x_out, self.z_out, self.null_uint, self.null_uint
        )

        if res < 1:
            return np.array([], dtype=np.float32), np.array([], dtype=np.float32)

        x_arr = np.array(self.x_out[:self.resolution], dtype=np.float32)
        z_arr = np.array(self.z_out[:self.resolution], dtype=np.float32)

        mask = (
            (z_arr > config.ROI_Z_MIN) & (z_arr < config.ROI_Z_MAX) &
            (x_arr > config.ROI_X_MIN) & (x_arr < config.ROI_X_MAX)
        )

        return x_arr[mask].copy(), z_arr[mask].copy()

    def stop(self):
        llt.transfer_profiles(self.hLLT, TTransferProfileType.NORMAL_TRANSFER, 0)

    def close(self):
        self.stop()
        llt.disconnect(self.hLLT)
        llt.del_device(self.hLLT)
"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 *
 * - Type: Discrete Scanner Driver
 * - Method for discrete measuring mode, where the rotary table
 * - moves in steps and the scanner captures profiles at each position.
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
import time
import config
import pyllt as llt
from .llt_datatypes import *



class ScannerDriverDiscrete:
    def __init__(self):
        """Základní inicializace připojení ke skeneru, nastavení rozlišení a parametrů."""
        self.simulace = getattr(config, 'REZIM_SIMULACE', False)
        if self.simulace:
            print("[SKENER] Simulation mode.")
            return

        print("[SKENER] Initializing system (pyllt)...")

        # alokace instance pro Ethernet
        self.hLLT = llt.create_llt_device(TInterfaceType.INTF_TYPE_ETHERNET)
        if self.hLLT == 0:
            raise Exception("[SKENER_ERROR] Failed to create LLT device instance.")

        # vyhledání a připojení zařízení
        available_interfaces = (ct.c_uint * 6)()
        ret = llt.get_device_interfaces_fast(self.hLLT, available_interfaces, len(available_interfaces))
        if ret < 1:
            raise Exception("[SKENER_ERROR] Scanner not found on the network.")

        llt.set_device_interface(self.hLLT, available_interfaces[0], 0)

        if llt.connect(self.hLLT) < 1:
            raise Exception("[SKENER_ERROR] Cannot connect to scanner.")

        # získání typu skeneru a rozlišení
        self.scanner_type = ct.c_int(0)
        llt.get_llt_type(self.hLLT, ct.byref(self.scanner_type))

        # nastavení profil konfigurace
        llt.set_profile_config(self.hLLT, TProfileConfig.PROFILE)

        # nastavení rozlišení (binning)
        # binning u scanCONTROL neni samostatny feature registr (FEATURE_FUNCTION_BINNING v llt_datatypes neexistuje)
        # rozliseni se nastavuje pres get_resolutions + set_resolution, kde SKENER_BINNING je index do pole dostupnych rozliseni:
        #   index 0 = maximalni rozliseni (2048 px)
        #   index 1 = 1024 px
        #   index 2 = 512 px
        #   index 3 = 256 px
        available_resolutions = (ct.c_uint * 4)()
        llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))

        binning_index = getattr(config, 'SKENER_BINNING', 0)
        if binning_index < 0 or binning_index >= len(available_resolutions):
            raise ValueError(
                f"[SKENER_ERROR] SKENER_BINNING index {binning_index} out of range "
                f"(0-{len(available_resolutions) - 1})."
            )
 
        zvolene_rozliseni = available_resolutions[binning_index]
        if zvolene_rozliseni == 0:
            raise ValueError(
                f"[SKENER_ERROR] Resolution at SKENER_BINNING index {binning_index} "
                f"is 0 - invalid."
            )
 
        res_set = llt.set_resolution(self.hLLT, zvolene_rozliseni)
        if res_set < 1:
            raise ValueError(f"[SKENER_ERROR] Error setting resolution. Code: {res_set}")
 
        self.res_val = zvolene_rozliseni
        print(f"[SKENER] Connected. Resolution (binning index {binning_index}): {self.res_val} px.")

        # aplikace parametrů (Expozice)
        self._konfigurovat_parametry()

        # načtení skutečného rozlišení po aplikaci binningu
        resolution_out = ct.c_uint(0)
        llt.get_resolution(self.hLLT, ct.byref(resolution_out))
        self.res_val = resolution_out.value
 
        print(f"[SKENER] Connected. Resolution after binning: {self.res_val} px.")

        # prealokace paměťových struktur
        self.profile_buffer_size = self.res_val * 64
        self.profile_buffer = (ct.c_ubyte * self.profile_buffer_size)()

        # výstupní pole souřadnice X a Z pro konverzi profilů
        self.x_out = (ct.c_double * self.res_val)()
        self.z_out = (ct.c_double * self.res_val)()

        # tři typy null pointerů dle signatur pyllt
        self.null_ushort = ct.POINTER(ct.c_ushort)()
        self.null_uint   = ct.POINTER(ct.c_uint)()

        # lost_profiles buffer pro get_actual_profile
        self.lost_profiles = ct.c_int(0)

        llt.transfer_profiles(self.hLLT, TTransferProfileType.NORMAL_TRANSFER, 1)

    def _konfigurovat_parametry(self):
        """Nastaví expoziční čas skeneru."""
        if not hasattr(config, 'SKENER_EXPOZICE'):
            return
        
        res = llt.set_feature(self.hLLT, FEATURE_FUNCTION_EXPOSURE_TIME, int(config.SKENER_EXPOZICE))
        if res < 1:
            raise ValueError(f"[SKENER_ERROR] Error writing EXPOSURE_TIME. Code: {res}")

    def ziskat_profil(self):
        """Vyčte aktuální statický profil, provede konverzi a ořízne dle ROI."""
        if self.simulace:
            return self._simulace()

        max_pokusy = 3

        for pokus in range(max_pokusy):
            # vyčtení aktuálního profilu z bufferu skeneru
            bytes_read = llt.get_actual_profile(
                self.hLLT, self.profile_buffer, self.profile_buffer_size,
                TProfileConfig.PROFILE, ct.byref(self.lost_profiles)
            )

            if bytes_read <= 0:
                time.sleep(0.05)
                continue

            # konverze surových bajtů na souřadnice (mm)
            ret_conv = llt.convert_profile_2_values(
                self.hLLT, self.profile_buffer, self.res_val,
                TProfileConfig.PROFILE,
                TScannerType(self.scanner_type.value),
                0, 1,
                self.null_ushort, self.null_ushort, self.null_ushort,
                self.x_out, self.z_out,
                self.null_uint, self.null_uint
            )

            if ret_conv < 1:
                continue

            # převod do NumPy polí
            x_arr = np.array(self.x_out[:self.res_val], dtype=np.float32)
            z_arr = np.array(self.z_out[:self.res_val], dtype=np.float32)

            # ořez přes ROI
            mask = (
                (z_arr > config.ROI_Z_MIN) & (z_arr < config.ROI_Z_MAX) &
                (x_arr > config.ROI_X_MIN) & (x_arr < config.ROI_X_MAX)
            )

            return x_arr[mask].copy(), z_arr[mask].copy()

        return np.array([], dtype=np.float32), np.array([], dtype=np.float32)

    def zmerit_prumer(self, pocet):
        """Změří počet profilů a zprůměruje je."""
        if self.simulace:
            return self._simulace()

        x_ref = None
        z_sum = None
        valid = 0

        for _ in range(pocet):
            x, z = self.ziskat_profil()
            if len(x) > 0:
                if x_ref is None:
                    x_ref = x
                    z_sum = z.astype(float)
                    valid = 1
                elif len(z) == len(z_sum):
                    z_sum += z
                    valid += 1

        if valid > 0:
            return x_ref, (z_sum / valid).astype(np.float32)
        return np.array([], dtype=np.float32), np.array([], dtype=np.float32)

    def close(self):
        """Ukončení připojení a uvolnění portů."""
        if not self.simulace:
            try:
                llt.transfer_profiles(self.hLLT, TTransferProfileType.NORMAL_TRANSFER, 0)
                llt.disconnect(self.hLLT)
                llt.del_device(self.hLLT)
                print("[SKENER] Disconnected successfully.")
            except Exception as e:
                print(f"[SKENER_ERROR] Close failed: {e}")
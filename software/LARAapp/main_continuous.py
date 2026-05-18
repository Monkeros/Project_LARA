"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 * - Description: 
 *   Continuous measurement method for 3D scanning. Implements free-run 
 *   data acquisition with zero-cross synchronization and time-to-angle interpolation.
"""



import time
import math
import sys
import os
import config
from drivers import ScannerDriverContinuous, ArduinoDriver



def ulozit_do_ply(data_buffer, cesta_souboru, t_start, t_end):
    print(f"\n[DATA] Calculate the points and save to: {cesta_souboru}")
    
    validni_data = [z for z in data_buffer if t_start <= z['t'] <= t_end]
    if not validni_data:    
        print("[ERROR] No valid data for saving.")
        return

    doba_otacky = t_end - t_start
    if doba_otacky <= 0:
        print("[ERROR] Invalid rotation time (T_end <= T_start).")
        return
    
    omega = 360.0 / doba_otacky

    pocet_bodu = sum(len(zaznam['x']) for zaznam in validni_data)

    if pocet_bodu == 0:
        print("[ERROR] No valid data for saving.")
        return

    beta_rad = math.radians(config.SKLON_SKENERU)
    cos_beta = math.cos(beta_rad)
    sin_beta = math.sin(beta_rad)
    off_x = config.OFFSET_X
    off_z = config.OFFSET_Z

    try:
        with open(cesta_souboru, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {pocet_bodu}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")

            for zaznam in validni_data:
                t_profil = zaznam['t']
                pole_vyska = zaznam['x']
                pole_hloubka = zaznam['z']

                uhel_stupne = (t_profil - t_start) * omega
                alpha_rad = math.radians(-uhel_stupne)
                cos_alpha = math.cos(alpha_rad)
                sin_alpha = math.sin(alpha_rad)

                for i in range(len(pole_vyska)):
                    h_raw = pole_vyska[i]
                    r_raw = pole_hloubka[i]

                    r_rotated = r_raw * cos_beta - h_raw * sin_beta
                    h_rotated = r_raw * sin_beta + h_raw * cos_beta

                    r_final = off_z - r_rotated
                    h_final = h_rotated - off_x

                    x_3d = r_final * cos_alpha
                    y_3d = r_final * sin_alpha
                    z_3d = h_final

                    f.write(f"{x_3d:.4f} {y_3d:.4f} {z_3d:.4f}\n")

        print(f"[DATA] Data export successful. Processed {len(validni_data)} profiles.")
    except Exception as e:
        print(f"[ERROR] Failed to write to file: {e}")

def main():
    print("\n======================================================================")
    print(f"                        PROJEKT L.A.R.A. v1.4                        ")  
    print("======================================================================")
    print(f"[SYSTEM] Scan No.: {config.NAZEV_PROJEKTU}.")

    try:
        pocet_profilu = config.KONTINUALNI_POCET_PROFILU
        pocet_zahrivacich_kol = config.KONTINUALNI_ZAHRIVACI_KOLA
        kroky_na_otacku = config.KROK_NA_OTACKU
        frekvence_skeneru = config.SKENER_FREKVENCE
        deadzone_koef         = getattr(config, 'KONTINUALNI_DEADZONE_KOEF', 0.85)

    except AttributeError:
        print("[ERROR] Missing parameters in config module for continuous measurement.")
        sys.exit(1)

    if frekvence_skeneru <= 0:
        print("[ERROR] Invalid scanner frequency.")
        sys.exit(1)

    t_otacky = pocet_profilu / frekvence_skeneru
    rychlost_motoru = kroky_na_otacku / t_otacky
    uhlovy_krok = 360.0 / pocet_profilu
    deadzone = t_otacky * deadzone_koef

    print("\n[PARAMETERS]")
    print(f" -> Number of profiles: {pocet_profilu}")
    print(f" -> Rotation time (T_ot): {t_otacky:.3f} s")
    print(f" -> Motor speed:          {rychlost_motoru:.1f} steps/s")
    print(f" -> Angular resolution:     {uhlovy_krok:.4f} °/profile")

    limit_rychlosti = 5000.0
    if rychlost_motoru > limit_rychlosti:
        print(f"[ERROR] Requested speed ({rychlost_motoru:.1f} steps/s) exceeds Arduino limit ({limit_rychlosti} steps/s).")
        sys.exit(1)

    try:
        motor = ArduinoDriver()
        scanner = ScannerDriverContinuous()
    except Exception as e:
        print(f"[ERROR] Hardware initialization failed: {e}")
        sys.exit(1)

    print("[ARDUINO] Finding home position...\n", end="")
    sys.stdout.flush()
    try:
        motor.home()
        print("[ARDUINO] OK.")
    except Exception as e:
        print(f"\n[ARDUINO_ERROR] Homing failed: {e}")
        if not config.REZIM_SIMULACE:
            sys.exit(1)

    print("\n----------------------------------------------------------------------")
    print("             INITIALIZATION COMPLETED. TABLE IS AT ZERO.               ")
    print("----------------------------------------------------------------------")
    try:
        input(">>> Press ENTER to start measurement (or Ctrl+C to exit) <<<")
    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")
        sys.exit(0)
    print("----------------------------------------------------------------------\n")

    t_start   = None
    t_end     = None
    data_buffer = []
      
    try:
        print("[ARDUINO] Motor start (SPIN)...")
        motor.spin(rychlost_motoru)
        #motor.serial.reset_input_buffer()
        motor.flush_buffer()

        #t_sleep = t_otacky * 0.8

        print(f"[SYSTEM] Starting warm-up cycles ({pocet_zahrivacich_kol}x).")
        for i in range(1, pocet_zahrivacich_kol + 1):
            time.sleep(t_otacky * 1.5 if i == 1 else t_otacky * 0.8)
            #if i == 1:
            #    time.sleep(3.0)
            #else:
            #    time.sleep(t_sleep)
            
            while not motor.async_komunikace():
                pass
            print(f" -> ZERO_CROSS detected (Cycle {i})")

        print("[SYSTEM] Start measurement. Awaiting data from scanner.")
        scanner.start_akvizice()

        #motor.serial.reset_input_buffer()
        motor.flush_buffer()
        print("[SYSTEM] Awaiting initial ZERO_CROSS for measurement start...")
        
        while not motor.async_komunikace():
            pass
        
        t_start = time.perf_counter()
        print("[SYSTEM] T_start recorded. Data acquisition in progress...")
        #t_end = None
        #data_buffer = []

        limit_buffer = 20000
        valid_count = 0
        empty_count = 0

        while True:
            t_profil = time.perf_counter()
            x, z = scanner.ziskat_profil()
            if x.size > 0:
                #t_profil = time.perf_counter()
                data_buffer.append({'t': t_profil, 'x': x, 'z': z})
                valid_count += 1
            else:
                empty_count += 1

            if motor.async_komunikace():
                t_aktualni = time.perf_counter()
                if (t_aktualni - t_start) > deadzone:
                    t_end = t_aktualni
                    print("[SYSTEM] T_end recorded. Ending measurement.")
                    break

            if valid_count >= limit_buffer:
                print(f"\n[WARNING] Buffer limit ({limit_buffer} profiles) reached. Ending acquisition.")
                t_end = time.perf_counter()
                break

        #drop_rate = (empty_count / (valid_count + empty_count)) * 100 if (valid_count + empty_count) > 0 else 0
        #print(f"[SYSTEM] Valid profiles: {valid_count} | Empty reads: {empty_count} | Drop rate: {drop_rate:.2f} %")
        oversample_ratio = empty_count / valid_count if valid_count > 0 else 0
        print(f"[SYSTEM] Valid profiles: {valid_count} | Polling loop iterations without new data: {empty_count}")
        print(f"[SYSTEM] Polling oversample ratio: {oversample_ratio:.1f}x (Loop is {oversample_ratio:.1f}x faster than scanner)")

        t_end = t_aktualni
        doba_mereni = t_end - t_start
        skutecna_frekvence = valid_count / doba_mereni
        print(f"[SYSTEM] Measurement duration: {doba_mereni:.2f} s")
        print(f"[SYSTEM] Effective scanner frequency: {skutecna_frekvence:.1f} Hz")

        motor.stop()
        scanner.stop()
        print(f"[SYSTEM] Rotation ended. Captured {len(data_buffer)} profiles before filtering.")

        print("[ARDUINO] Parking the table at zero (HOME)")
        try:
            motor.home() 
            print("   ->Parked. Ready for the next scan.")
        except Exception as e:
            print(f"[ERROR] Parking failed ({e})")  

        soubor = os.path.join(config.DATA_FOLDER, f"{config.NAZEV_PROJEKTU}.ply")
        if t_end is None:
            print("[WARNING] T_end not recorded, skipping save.")
        else:
            ulozit_do_ply(data_buffer, soubor, t_start, t_end)
        #ulozit_do_ply(data_buffer, soubor, t_start, t_end)

        print("End of program.")

    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        motor.stop()
        motor.close()
        scanner.stop()
        scanner.close()

if __name__ == "__main__":
    main()
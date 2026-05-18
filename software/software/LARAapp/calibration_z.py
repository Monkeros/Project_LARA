"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 * - Description: 
 *   Calibration script designed to determine the precise Z-axis offset 
 *   and the center of rotation of the rotary table. It utilizes the laser scanner 
 *   to measure a known calibration target (e.g., a reference cylinder) and computes 
 *   the spatial constants necessary for accurate 3D point cloud transformation.
"""



import time
import math
import sys
import numpy as np
import config
from drivers import ScannerDriverDiscrete, ArduinoDriver



# Z_OFFSET výpočet
def vyhodnotit_kalibraci(data_buffer, prumer_etalonu):
    """Zpracuje naměřené profily a vypočítá průměrný Z-offset"""
    beta_rad = math.radians(config.SKLON_SKENERU)
    cos_beta = math.cos(beta_rad)
    sin_beta = math.sin(beta_rad)
    polomer_etalonu = prumer_etalonu / 2.0

    stredy_z = []

    for zaznam in data_buffer:
        x_raw = np.array(zaznam['x'])
        z_raw = np.array(zaznam['z'])
        
        if len(x_raw) == 0:
            continue

        # narovnání profilu vůči sklonu skeneru
        # r_rotated je vodorovná vzdálenost bodu od snímače
        r_rotated = z_raw * cos_beta - x_raw * sin_beta
        
        # median bodů profilu
        vzdalenost_povrch = np.median(r_rotated)
        
        # absolutní střed osy Z = vzdálenost k povrchu + poloměr kal. válce
        stredy_z.append(vzdalenost_povrch + polomer_etalonu)

    if not stredy_z:
        return None, None

    avg_offset = np.mean(stredy_z)
    rozptyl = np.max(stredy_z) - np.min(stredy_z)
    return avg_offset, rozptyl

# Kalibrační algoritmus
def main():
    # nastavení kalibrace
    PRUMER_VALCE = 40.0      # průměr kalibrančího válce v mm (zadat parametry přesně)
    KROK_KALIBRACE = 15.0    # úhel kroku rotace 
    
    print(f"\n=== AUTOMATIC Z-OFFSET CALIBRATION ===")
    print(f"Parameters: cylinder {PRUMER_VALCE} mm, Step {KROK_KALIBRACE}°")

    # 1. Inicializace hardwaru
    try:
        motor = ArduinoDriver()
        scanner = ScannerDriverDiscrete()
    except Exception as e:
        print(f"[ERROR] Hardware error: {e}")
        sys.exit(1)

    # 2. Homing 
    print("[ARDUINO] Finding home position...", end="")
    sys.stdout.flush()
    try:
        motor.home() 
        print("[ARDUINO] OK.")
    except Exception as e:
        print(f"\n[ARDUINO_ERROR] Homing failed: {e}")
        sys.exit(1)

    # 3. Inicializace proměnných
    namerena_data = []
    aktualni_uhel = 0.0
    kroky_na_stupen = config.KROK_NA_OTACKU / 360.0
    aktualni_pozice_kroky_int = 0

    print("\n" + "-"*50)
    print("[ARDUINO] Table is at zero position.")
    print("-"*50)
    try:
        input(">>> Press ENTER to start calibration (or Ctrl+C to exit) <<<")
    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")
        sys.exit(0)

    faze = "ODJEZD"

    # Hlavní cykl
    try:
        start_time = time.time()
        
        while True: 
            # a) Měření profilu
            x_data, z_data = scanner.zmerit_prumer(pocet=config.POCET_PROFILU_NA_PRUMER)

            #if len(x_data) > 0:
            #    namerena_data.append({'uhel': aktualni_uhel, 'x': x_data, 'z': z_data})
            #    print(f"Úhel {aktualni_uhel:5.1f}° | Stav: {faze:8} | Měřeno OK", end="\r")
            if len(x_data) > 0:
                namerena_data.append({'uhel': aktualni_uhel, 'x': x_data, 'z': z_data})

                # 1. Převede Z_data na numpy pole
                z_array = np.array(z_data)
            
                # 2. Najde povrch válce pomocí mediánu
                aktualni_povrch = np.median(z_array)
                
                # 3. Vypočítá aktuální Z_střed 
                aktualni_z_stred = aktualni_povrch + (PRUMER_VALCE / 2.0)
                
                # 4. Vypíše hodnotu do terminálu 
                print(f"Angle {aktualni_uhel:5.1f}° | Status: {faze:8} | Current Z-center: {aktualni_z_stred:.2f} mm   ", end="\r")
            else:
                print(f"Angle {aktualni_uhel:5.1f}° | Status: {faze:8} | ERROR", end="\r")

            # b) Logika senzoru
            je_senzor_aktivni = motor.stav_senzoru()

            if faze == "ODJEZD":
                if not je_senzor_aktivni:
                    faze = "NA_TRATI" 
            
            elif faze == "NA_TRATI":
                if je_senzor_aktivni or aktualni_uhel >= 360.0:
                    print(f"\n\n-> Cycle completed (Sensor: {je_senzor_aktivni})")
                    break

            if aktualni_uhel > 400.0:
                print("\n[STOP] Safety limit exceeded.")
                break

            # c) Pohyb na další pozici
            aktualni_uhel += KROK_KALIBRACE
            cilove_kroky_int = int(aktualni_uhel * kroky_na_stupen)
            delta_kroky = cilove_kroky_int - aktualni_pozice_kroky_int

            if delta_kroky > 0:
                motor.move(delta_kroky)
                time.sleep(config.CAS_STABILIZACE)
            
            aktualni_pozice_kroky_int = cilove_kroky_int

        # Konec cyklu
        offset_z, rozptyl = vyhodnotit_kalibraci(namerena_data, PRUMER_VALCE)

        if offset_z:
            print("\n" + "="*60)
            print("                Z_OFFSET CALIBRATION RESULTS               ")
            print("="*60)
            print(f" OFFSET_Z: {offset_z:.4f} mm")
            print(f" Eccentricity: {rozptyl:.4f} mm")
            print(f" Total samples: {len(namerena_data)}")
            print("-"*60)
            print(" Please update 'Final OFFSET_Z' value in config.py.")
            print("="*60 + "\n")
        else:
            print("\n[ERROR] Failed to calculate any values.")

        print("[ARDUINO] Parking the table at zero (HOME)")
        try:
            motor.home() 
            print("   ->Parked. Ready for the next scan.")
        except Exception as e:
            print(f"[ERROR] Parking failed ({e})")

    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        motor.close()
        scanner.close()

if __name__ == "__main__":
    main()
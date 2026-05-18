"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 * - Description: 
 *   Discrete measurement method for 3D scanning of a rotating table using 
 *   a Micro-Epsilon ScanControl laser scanner and an Arduino-controlled 
 *   rotary platform. The method implements a "stop & go" algorithm, where 
 *   the table moves in increments and the scanner captures profiles at 
 *   each position. The collected data is then transformed into a 3D point 
 *   cloud and saved in PLY format.
"""



import time
import math
import sys
import os
import config
from drivers import ScannerDriverDiscrete, ArduinoDriver 



# Transformace dat a uložení do .ply formátu
def ulozit_do_ply(data_buffer, cesta_souboru):
    print(f"\n[DATA] Calculate the points and save to: {cesta_souboru}")
    pocet_bodu = sum(len(zaznam['x']) for zaznam in data_buffer)
    
    # matematické konstanty
    beta_rad = math.radians(config.SKLON_SKENERU)
    cos_beta = math.cos(beta_rad)
    sin_beta = math.sin(beta_rad)
    off_x = config.OFFSET_X
    off_z = config.OFFSET_Z

    with open(cesta_souboru, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {pocet_bodu}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for zaznam in data_buffer:
            uhel_stupne = zaznam['uhel']
            pole_vyska = zaznam['x'] 
            pole_hloubka = zaznam['z']
            
            alpha_rad_inv = math.radians(-uhel_stupne)
            cos_alpha_inv = math.cos(alpha_rad_inv)
            sin_alpha_inv = math.sin(alpha_rad_inv)

            for i in range(len(pole_vyska)):
                h_raw = pole_vyska[i]   # X - osa podél laserové čáry
                r_raw = pole_hloubka[i] # Z - vzdálenost/hloubka od snímače

                # 1. Korekce náklonu skeneru (Rotace 2D profilu) 
                # nejdříve data ze skeneru narovná rovnoběžně se stolem.
                # r_rotated směřuje přesně do středu stolu a h_rotated míří kolmo nahoru.
                r_rotated = r_raw * cos_beta - h_raw * sin_beta
                h_rotated = r_raw * sin_beta + h_raw * cos_beta

                # 2. Výpočet poloměru a výšky (Aplikace offset konstant)
                r_final = off_z - r_rotated  # výsledná vzdálenost bodu od osy rotace stolu
                h_final = h_rotated - off_x  # výsledná výška bodu od roviny stolu, off_x není nutné uvádět, v pointcloud se projeví pouze jako záporné či kladné posunutí 

                # 3. Transformace do 3D
                # rotace bodu okolo osy Z s využitím inverzního úhlu.
                x_3d = r_final * cos_alpha_inv
                y_3d = r_final * sin_alpha_inv
                z_3d = h_final

                # uložení bodu do .ply
                f.write(f"{x_3d:.4f} {y_3d:.4f} {z_3d:.4f}\n")

    print("[DATA] Data export was successful.")

# HLAVNÍ PROGRAM - algoritmus diskrétní metody (stop_go)
def main():
    print("\n======================================================================")
    print(f"                        PROJEKT L.A.R.A. v1.4                        ")  
    print("======================================================================")
    print(f"[SYSTEM] Scan No.: {config.NAZEV_PROJEKTU}.")
    
    # 1. Inicializace zařízení
    try:
        motor = ArduinoDriver()
        scanner = ScannerDriverDiscrete()
    except Exception as e:
        print(f"[ERROR] Hardware error: {e}")
        sys.exit(1)

    # 2. Příkaz HOME
    print("[ARDUINO] Finding home position...\n", end="")
    sys.stdout.flush()
    try:
        # volá příkaz HOME, který točí stolem(motorem), dokud stav != 1
        motor.home() 
        print("[ARDUINO] OK.")
    except Exception as e:
        print(f"\n[ARDUINO_ERROR] Homing failed: {e}")
        
        if not config.REZIM_SIMULACE:
            sys.exit(1)

    # 3. Inicializace úhlu = 0
    namerena_data = []
    aktualni_uhel = 0.0
    
    # příprava kroků (Matematika pohybu)
    kroky_na_stupen = config.KROK_NA_OTACKU / 360.0
    cilove_kroky_float = 0.0
    aktualni_pozice_kroky_int = 0

    # potrvzení uživatelem (bezpečnostní pojistka)
    print("\n----------------------------------------------------------------------")
    print("             INITIALIZATION COMPLETED. TABLE IS AT ZERO.               ")
    print("----------------------------------------------------------------------")
    try:
        # program čeká, dokud uživatel nepotvrdí Enterem
        input(">>> Press ENTER to start measurement (or Ctrl+C to exit) <<<")
    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")
        sys.exit(0)
    print("----------------------------------------------------------------------\n")

    # 4. Fáze = 'ODJEZD'
    # start na senzoru, stůl z něj musí nejdříve vyjet (clonka)
    faze = "ODJEZD" 
    print(f"[SYSTEM] State: {faze}\n")

    # 5. Hlavní cykl (Stop & Go)
    try:
        start_time = time.time()
        doba_motoru = 0.0
        
        while True:

            # Bezpečnostní pojistka (posunutá výše)
            if aktualni_uhel > 400.0:
                print("\n[WARNING] Limit exceeded 400.0°! Safety stop.")
                break

            # a) Měření na pozici (skener změří profil, data se uloží do bufferu)
            #print(f"Úhel {aktualni_uhel:.1f}° [{faze}] ... ", end="")
            sys.stdout.flush()

            x_min = x_max = z_min = z_max = 0
            pts = 0
            stav = "NO DATA"       #NaN

            t_mereni_start = time.time() # měření času pro diagnostiku
            # skener změří profil
            x_data, z_data = scanner.zmerit_prumer(pocet=config.POCET_PROFILU_NA_PRUMER)
            t_mereni_stop = time.time() # měření dokončeno
            doba_skeneru = t_mereni_stop - t_mereni_start

            if x_data.size > 0:
                x_min = x_data.min()
                x_max = x_data.max()
                z_min = z_data.min()
                z_max = z_data.max()

                pts = x_data.size
                stav = "OK"
                namerena_data.append({
                    'uhel': aktualni_uhel,
                    'x': x_data,
                    'z': z_data
                })

            print(
                f"[{aktualni_uhel:6.1f}° | {faze:8}] "
                f"{stav:7} | pts: {pts:4} | "
                f"X: {x_min:6.2f} → {x_max:6.2f} | "
                f"Z: {z_min:6.2f} → {z_max:6.2f} | "
                f"scan: {doba_skeneru:5.2f}s | "
                f"motor: {doba_motoru:5.2f}s"
            )

            # b) Rozhodovací logika (logika senzoru pro diskrétní metodu)
            je_senzor_aktivni = motor.stav_senzoru() # pošle GET_SENSOR

            if faze == "ODJEZD":
                # pokud stůl opustil senzor (nulu)
                if not je_senzor_aktivni:
                    print("[EVENT ] Sensor abandoned → transition to a phase 'NA_TRATI'")
                    faze = "NA_TRATI" 
            
            elif faze == "NA_TRATI":
                # konec, pokud je senzor aktivní nebo překročil 360 stupnu
                if je_senzor_aktivni or aktualni_uhel >= 360.0:
                    print(f"-> TARGET ACHIEVED (Sensor: {je_senzor_aktivni}, Angle: {aktualni_uhel:.1f}°)")
                    break 

            # c) Pohyb na další pozici (příprava pro příští kolo)
            dalsi_uhel = aktualni_uhel + config.KROK_MERENI
            
            # přepočet na kroky
            cilove_kroky_float = dalsi_uhel * kroky_na_stupen
            cilove_kroky_int = int(cilove_kroky_float)
            delta_kroky = cilove_kroky_int - aktualni_pozice_kroky_int

            t_pohyb_start = time.time() # stopky start pro pohyb motoru

            # pohyb motoru o delta_kroky
            if delta_kroky > 0:
                motor.move(delta_kroky)
                time.sleep(config.CAS_STABILIZACE)

            t_pohyb_stop = time.time()  # stopky stop pro pohyb motoru

            # výpis diagnostiky času do terminalu
            doba_skeneru = t_mereni_stop - t_mereni_start
            doba_motoru = t_pohyb_stop - t_pohyb_start
            #print(f"   >>> [STOPKY] Skener: {doba_skeneru:.2f}s | Motor: {doba_motoru:.2f}s")
            
            # aktualizace proměnných pro příští průběh cyklu
            aktualni_pozice_kroky_int = cilove_kroky_int
            aktualni_uhel = dalsi_uhel

        # 6. Konec cyklu -> Transformace dat
        cas_celkem = time.time() - start_time
        print("---------------------------------------------------")
        print(f"Scanning completed in {cas_celkem:.1f} s.")

        # pokud jsme přejeli 360 stupnů, zaparkujeme stůl opět na nulu (HOME)
        print("[ARDUINO] Parking the table at zero (HOME)")
        try:
            motor.home() 
            print("   ->Parked. Ready for the next scan.")
        except Exception as e:
            print(f"[ERROR] Parking failed ({e})")
        
        # 7. Uložení do souboru .ply 
        soubor = os.path.join(config.DATA_FOLDER, f"{config.NAZEV_PROJEKTU}.ply")
        ulozit_do_ply(namerena_data, soubor)

        # 8. Konec algoritmu
        print("End of program.")

    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        motor.close()
        scanner.close()

if __name__ == "__main__":
    main()
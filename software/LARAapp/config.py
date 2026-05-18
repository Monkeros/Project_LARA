"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 * - Description: 
 *   Configuration module serving as the central parameter repository 
 *   for the L.A.R.A. project. It contains static calibration constants (offsets, angles), 
 *   Region of Interest (ROI) limits, communication port settings, and user-defined 
 *   variables required for accurate scanning and hardware operation.
"""



import os



#-----------------------------------------------------------------------------------------------------------------------------------------

# ==============================================================================
# 1. NASTAVENÍ MĚŘENÍ
# ==============================================================================
NAZEV_PROJEKTU = "dd_mm_yy_scan_xx"       # pojmenování souboru s daty (bez přípony, bude přidáno .ply)
DATA_FOLDER = "namerena_data"       # adresář pro ukládání dat (vytvoří se, pokud neexistuje)

# Režim simulace
# True  = pouze pro účely testování (bez nutnosti připojení hardware)
# False = pro reálné měření (vyžaduje připojení hardware)
REZIM_SIMULACE = False

#-----------------------------------------------------------------------------------------------------------------------------------------

# ==============================================================================
# 2. ARDUINO (Rotační stůl)
# ==============================================================================
ARDUINO_PORT = "COM3"               # měl by automaticky detekovat (kdyžtak konfigurovat -> Device Manager/Arduino IDE)
BAUD_RATE = 115200                  # přenosová rychlost (musí odpovídat FW)
TIMEOUT = 30.0                      # kolik sekund čekat na odpověď (max)

# Parametry pohybu
MOTOR_RYCHLOST = 3000.0             # rychlost otáčení (kroky/s)
MOTOR_ZRYCHLENI = 1000.0            # zrychlení stolu (kroky/s^2)
KROK_NA_OTACKU = 9600.0             # kolik kroků motoru je 360 stupňů
KROK_MERENI = 1.0                   # krokování motoru pro měření (ve stupních)

#-----------------------------------------------------------------------------------------------------------------------------------------

# ==============================================================================
# 3. SKENER (Micro-Epsilon ScanControl 30xx)
# ==============================================================================
SKENER_IP = "169.254.72.105"        # pokud nenajde IP automaticky, lze nastavit ručně (aktuálně nefunkční)

# Nastavení kvality obrazu
SKENER_EXPOZICE = 500               # čas osvitu v mikrosekundách (výchozí hodnota 1000us -> zdá se použitelná pro většinu měření)
SKENER_FREKVENCE = 25               # Počet profilů za sekundu (Hz)
SKENER_LASER_POWER = 2              # Výkon laseru (0 = OFF, 1 = REDUCED, 2 = FULL) - pyllt.py (pro kontinuální metodu)
SKENER_BINNING = 0                  # Sloučení bodů (0 = 2048, 2 = 512) - vyšší binning zvyšuje kvalitu dat, ale snižuje rozlišení 
POCET_PROFILU_NA_PRUMER = 5         # udává počet profilů pro výpočet průměru

# ROI- Filtrování dat
# 1. Filtr vzdálenosti od skeneru (Z-osa)
ROI_Z_MIN = -500.0                  # minimální vzdálenost od senzoru (mm)
ROI_Z_MAX = 500.0                   # maximální vzdálenost od senzoru (mm)

# 2. Filtr výšky (X-osa)
ROI_X_MIN = -500.0                  # minimální výška (mm)
ROI_X_MAX = 500.0                   # maximální výška (mm)

CAS_STABILIZACE = 0.15              # T_stabilizace - Sekundy (FW má settling time 500ms, takže toto je navíc)

#-----------------------------------------------------------------------------------------------------------------------------------------

# ==============================================================================
# 4. PARAMETRY KONTINUÁLNÍHO MĚŘENÍ
# ==============================================================================
KONTINUALNI_POCET_PROFILU = 1000    # požadovaný počet profilů na 360° (3600 = rozlišení 0.1°)
KONTINUALNI_ZAHRIVACI_KOLA = 1      # počet otáček pro stabilizaci mechaniky před měřením
KONTINUALNI_DEADZONE_KOEF = 0.85    # koeficient pro výpočet deadzone (0.85 = 85% doby otáčky)


# ==============================================================================
# 5. KALIBRACE A GEOMETRIE (DOPLNĚNO)
# ==============================================================================
# A) Kalibrace (Osa X)
OFFSET_X = 0.0                      # udává posunutí středu skeneru od fyzického stolu (mm)

# B) Kalibrace (Osa Z)
OFFSET_Z = 272.2099                 # udává posun nuly skeneru od skutečné osy rotace stolu (mm)

# C) Náklon skeneru
SKLON_SKENERU = 0.0                 # udává sklon skeneru vůči vodorovné rovině (mm) (stupně)

#-----------------------------------------------------------------------------------------------------------------------------------------

if not os.path.exists(DATA_FOLDER):
    os.makedirs(DATA_FOLDER)
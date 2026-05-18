"""
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * -----------------------------------------------------------------------------------
 * - Author: Jan Boček 
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 *
 * - Type: Arduino Driver
 * - Method for communication with Arduino microcontroller, which controls the rotary table.
 *
 * - Description: 
 *   Hardware abstraction layer containing driver for Arduino microcontroller.
 *   It handles serial communication (UART/USB), command parsing, data exchange,
 *   and basic error handling between host PC and embedded Arduino firmware.
 *
 * - Dependency Notice: 
 *   This software communicates with Arduino hardware using standard
 *   serial communication protocol (USB CDC / UART).
 *   Arduino is an open-source electronics platform, and any firmware or
 *   libraries used on the microcontroller side remain subject to their
 *   respective open-source licenses (e.g., Arduino core libraries).
 *
"""



import serial
import time
import sys
import config



class ArduinoDriver:
    def __init__(self):
        self.simulace = config.REZIM_SIMULACE
        self.serial = None
        
        if self.simulace:
            print("[ARDUINO] Simulation mode.")
            return
        print(f"[ARDUINO] Connecting to {config.ARDUINO_PORT}...")
        try:
            self.serial = serial.Serial(config.ARDUINO_PORT, config.BAUD_RATE, timeout=config.TIMEOUT)
            time.sleep(2.5) # čeká na restart Arduina (DTR)
            self.serial.reset_input_buffer()

            # nastavení rychlosti a zrychlení pro fw
            self._sync_komunikace(f"SPEED:{config.MOTOR_RYCHLOST}", "SPEED_OK")  
            self._sync_komunikace(f"ACCEL:{config.MOTOR_ZRYCHLENI}", "ACCEL_OK")
            print("[ARDUINO] Connected.")
        except Exception as e:
            print(f"[ARDUINO_ERROR] Unable to open port: {e}")
            sys.exit(1)

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[ARDUINO] Port closed.")

    def _sync_komunikace(self, prikaz, ocekavana_odpoved, vlastni_timeout=None):
        if self.simulace: return
        self.serial.write(f"{prikaz}\n".encode())
        start = time.time()
        aktualni_timeout = vlastni_timeout if vlastni_timeout is not None else config.TIMEOUT   # zjistí který timeout použít (vlasntí nebo z configu)
        
        while (time.time() - start) < aktualni_timeout:       # připadně změnit z aktualni_timeout na config.TIMEOUT
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode(errors='ignore').strip()
                if line == ocekavana_odpoved: return
                if "ERROR" in line: raise Exception(f"[ARDUINO_ERROR] {line}")
            time.sleep(0.005)
        raise TimeoutError(f"[ARDUINO_ERROR] Arduino did not respond to {prikaz} within the time limit {aktualni_timeout} s.")

    def home(self):
        print("[ARDUINO] Sending HOME command...")
        self._sync_komunikace("HOME", "HOME_OK", vlastni_timeout=30.0)      # delší timeout pro homing, muže trvat dele
        print("[ARDUINO] Home position set successfully.")

    def move(self, kroky):
        self._sync_komunikace(f"MOVE:{int(kroky)}", "READY")

    def stav_senzoru(self):
        """Pošle GET_SENSOR, vrátí true (1) nebo false (0)."""
        if self.simulace: return False
        
        self.serial.write(b"GET_SENSOR\n")
        start = time.time()
        while (time.time() - start) < 5.0:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                if line == "1": return True
                if line == "0": return False
            time.sleep(0.005)
        return False
    
    def spin(self, rychlost):
        """Spustí kontinuální otáčení a počká na potvrzení startu."""
        if self.simulace: return
        self.serial.reset_input_buffer() # resetuje buffer před startem
        self._sync_komunikace(f"SPIN:{float(rychlost)}", "SPIN_OK")

    def stop(self):
        """Zastaví kontinuální otáčení a počká na potvrzení."""
        if self.simulace: return
        #self.serial.write(b"STOP\n")
        self._sync_komunikace("STOP", "STOP_OK")

    def async_komunikace(self):
        """
        Neblokující kontrola sériové linky v hlavní smyčce.
        Vrátí true, pokud zachytí 'ZERO_CROSS', jinak vrací false.
        """
        if self.simulace: return False
        
        # přečte všechny zprávy, které se nahromadily v bufferu (prevence přetečení)
        while self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if line == "ZERO_CROSS":
                    return True
            except:
                pass
        return False
    
    def flush_buffer(self):
        self.serial.reset_input_buffer()
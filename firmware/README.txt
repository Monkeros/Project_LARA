# L.A.R.A. — Firmware

Firmware řídicí desky Arduino Uno pro rotační platformu. Součást bakalářské práce *Návrh a realizace rotační platformy pro rozšíření funkcionality laserového profilového skeneru*.

## Požadavky

- Arduino IDE
- Arduino Uno R3
- Knihovny AccelStepper.h, LiquidCrystal I2C

## Komunikační protokol

Sériová komunikace UART (115 200 Bd), architektura Master/Slave. Podporované instrukce:

- `MOVE:n` - relativní pohyb o n kroků
- `HOME` - inicializace nulové polohy
- `SPIN:v` - kontinuální otáčení rychlostí v
- `STOP` - zastavení motoru 
- `SPEED:v` - konfigurace rychlosti
- `ACCEL:a` - konfigurace zrychlení
- `GET_SENSOR` - čtení stavu referenčního snímače

## Nahrání firmware

1. Otevřít projekt v Arduino IDE.
2. Připojit Arduino Uno přes USB.
3. Zvolit správný COM port a desku (Arduino Uno).
4. Nahrát firmware tlačítkem Upload.

## Poznámka

V zařízení je již nahraný aktuální firmware.

## Autor

Jan Boček — Brno University of Technology, Faculty of Mechanical Engineering
# L.A.R.A. — Software

Softwarová část bakalářské práce *Návrh a realizace rotační platformy pro rozšíření funkcionality laserového profilového skeneru*.

## Požadavky

- Python 3.x
- Knihovny: viz `requirements.txt`
- scanCONTROL SDK (LLT.dll) — [Micro-Epsilon](https://www.micro-epsilon.com/)

## Důležité upozornění

Soubory 'llt_datatypes.py' a 'LLT.dll' (wrapper pyllt) **nejsou součástí tohoto repozitáře**. Jedná se o proprietární software společnosti Micro-Epsilon Messtechnik GmbH & Co. KG. Pro jeho získání je nutné kontaktovat výrobce nebo jej stáhnout z oficiální distribuce SDK scanCONTROL.

## Konfigurace

Před měřením nastavit parametry v `config.py`:
- `SKENER_EXPOZICE` - expoziční čas senzoru
- `SKENER_FREKVENCE` - profilová frekvence (kontinuální metoda)
- `SKENER_BINNING` - rozlišení profilu
- `OFFSET_Z`, `OFFSET_X`, `SKLON_SKENERU` - kalibrační konstanty
- `ROI_*` - software ROI

## Použití

```bash
# Kalibrace osy Z
python calibration_z.py

# Diskrétní měření
python main_discrete.py

# Kontinuální měření
python main_continuous.py
```

## Autor

Jan Boček — Brno University of Technology, Faculty of Mechanical Engineering
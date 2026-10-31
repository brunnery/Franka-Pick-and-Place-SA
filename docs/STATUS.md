# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`, Theorie in `docs/THEORY.md`.

## Stand vom 31.10.2026 (Abgabetag)

**Endrunde: 9/10 aufgeschraubt, 1 teilweise** (links 5/5, rechts 4/5), Zyklus bis Iris öffnet 41.3 s. Das ist das Zielbild des Betreuers: keine `target_correction`, keine `tilt_correction`, keine Verschiebung pro Seite, keine Festzieh-Werte pro Seite. Korrigiert wird nur das Kameramodell (Neigung pro Seite, Drehung um z, Position y +3 mm), den Rest macht die Regelung: `tighten_stiffness` [1500, 1500, 600, 4, 4, 20] plus Spirale. Der frühere Stand mit Handkorrekturen (29/30) bleibt unter dem Tag `working-29of30`.

## Heute erledigt

- **Hand-Eye nach dem Umbau neu**, auf beiden Tischseiten: rechts [0.98, 0.06]° / 1.55°, links [1.14, −0.87]° / 1.44°. Neigung pro Seite übernommen, Drehung um z 1.5°. Der alte Wert [1.54, 0.79] passte nicht mehr.
- **Genauigkeit der ChArUco-Methode bewertet:** wiederholbar auf 0.1°, aber Teilmengen widersprechen sich um bis zu 1°, also eher ±0.5°. Die Kameraposition misst sie nicht.
- **Kameraposition y +3 mm** in `frames.flange_to_camera_housing_mm` (Gegenwert in `camera_housing_to_tcp_mm`). Der Vergleich Nah-/Fernmessung zeigt: Die Neigung stimmt, der Rest ist abstandsunabhängig.
- **Ablauf gestrafft:** Absenken bis 6 N (ersetzt Kontakt + Seat + Warten), Referenz 2 mm unter die Mutter, Spirale sofort, danach direkt drehen. Beim Festziehen geht die Referenz nur noch nach unten.
- **Spirale neu im `screw`-Preset** (K_z 600, Radius 3 mm, 3 Umdrehungen, bis 8 s), Stopp bei 0.6 mm Absinken oder 0.25 mm Anstieg. Wobble in Schritten und Eindrehen während der Spirale sind verworfen, Rückwärts-Variante nur angetestet.
- **Abheben geklärt:** Die Höhenaufzeichnung zeigt keinen Hub im Übergang; die Mutter läuft in den äusseren Runden auf die Schraube auf. Dafür der Anstiegs-Stopp.
- **Steifigkeitsreihe G31:** rot 4 27/30, rot 40 20/30.

## Messergebnisse 31.10.

| Runde | tighten K_xy / K_rot | Aufgeschraubt | Links | Rechts |
|---|---|---|---|---|
| G31 | 400 / 4 | 9/10 | 5/5 | 4/5 |
| G31 | 1000 / 4 | 8/10 | 3/5 | 5/5 |
| G31 | 1500 / 4 | **10/10** | 5/5 | 5/5 |
| G31 | 400 / 40 | 6/10, 2 teilweise | 1/5 | 5/5 |
| G31 | 1000 / 40 | 6/10 | 3/5 | 3/5 |
| G31 | 1500 / 40 | 8/10 | 5/5 | 3/5 |
| **final** | **1500 / 4** | **9/10, 1 teilweise** | **5/5** | **4/5** |

K_z 600, K_rz 20 fest. Rot 4 gegen rot 40: Fisher p ≈ 0.06. K_xy: Rauschen (18/20 gegen 15/20, p ≈ 0.4). Mit K_xy 1500 / rot 4 insgesamt 19/20. Nicht gezählt: 4 Abbrüche vor dem Schrauben (Erkennung, Entscheid Yannick) und der Kabel-Lauf. Der Lauf 11:52 hat nur einen Ersatz-Record (Bewertung ohne Messdaten).

## Vorläufige Werte

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_tasks/config/tasks.yaml:13` `flange_to_camera_housing_mm` y | 3.0 | mit `tcp_offset` direkt messen | aus dem Landefehler geschätzt, nicht gemessen |
| `software/src/sa_tasks/config/tasks.yaml:31` `camera_tilt_in_flange_deg` | links/rechts | klären | Wert pro Seite; Neigung um y unterscheidet sich um 0.9°, vermutlich Anteil des Arms |
| `software/src/sa_vision/config/vision.yaml:63` `targets.stud.size_tolerance` | 0.8 (TEMPORARY) | 0.4 | war für die provisorische Halterung |
| `software/src/sa_tasks/config/poses.yaml` | Stand der letzten Seitenwahl | wie gewünscht | Seitentausch per GUI |

## Offene Probleme

1. **Die Spirale liefert kein Erfolgssignal.** In den 69 gezählten Läufen von G31 und final endet sie 60-mal am Anstieg, nur 4-mal am Absinken, sonst nach Zeit. Ob sie die Quote hebt, ist nicht getrennt getestet (kein Vergleich ohne Spirale).
2. **Erkennungsfehler:** 4 Abbrüche (`no nut found`, `none … is a stud`). Für die Gesamtquote im Bericht entweder mitzählen oder klar nennen.
3. **Bericht ehrlich formulieren:** Die 3 mm wirken bei fester Orientierung wie ein konstanter `target_correction`; sie sind Teil des Kameramodells, keine Messung.
4. **Pick zu hoch (31.10., ca. 10:10):** Der Arm fuhr nicht tief genug, um die Mutter zu greifen; kein Record, später nicht mehr gemeldet. Ursache unbekannt, Verdacht: Platte stand auf der Mutter.

## Als Erstes, falls noch weitergearbeitet wird

1. Bericht: Ergebnisse aus der Tabelle oben, Abschnitte in `docs/THEORY.md` 1.5 und 5.
2. Optional `tcp_offset` je Seite (GUI: *Bolt approach only*, Handführen in PROG, *Robot state only*, *Camera -> iris offset*), Sollwert nahe (0, 0).
3. Optional ein Vergleich ohne Spirale: `turning_spiral.enabled: false`, 10 Läufe (Runde umbenennen).

## Nützliche Befehle

- Start: `sa_robot`, GUI `sa_full_all`.
- Auswertung einer Runde: `sa_env && ros2 run sa_tasks run_map --round "final 31.10 xy1500 rot4"`.
- Zurück zum alten Stand mit Handkorrekturen: `git checkout working-29of30 -- software/src`.
- Unit-Tests: `cd software/src/sa_tasks && PYTHONNOUSERSITE=1 python3 -m pytest test -q`.
- Hängt ein Start: zuerst `ls /dev/v4l/by-id` (Kamerakabel), dann `bash gui/clean_dds_shm.sh`.

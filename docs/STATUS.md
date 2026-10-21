# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`.

## Stand vom 21.10.2026

Der ganze Ablauf `full` läuft durch: Mutter greifen → Stiftschraube suchen und zentrieren → anfahren → absenken → M4 dreht → Iris auf → hoch. Geschraubt wird noch nicht, weil der gedruckte Mitnehmer von M4 leer dreht. Pick ist zuverlässig (13/14). Die Stud-Erkennung ist im Trockentest deutlich besser geworden (Kanten-Verfeinerung), am Roboter aber **noch nicht getestet**.

## Heute erledigt

- **Neue Halterung geprüft (Ringblatt `data/grid/kamera_ausrichtung.pdf`):** Kamera trifft 0.1–0.6 mm innerhalb des CAD-Radius, über ±30° Drehung nur 0.5 mm Streuung → ≤ 0.5° Neigung, Kamera-Halterung steif genug.
- **Untere Hälfte hängt schief:** frei hängend 45 / 49 mm (Unterkante rechts / links) → ≈ 1.8° um x. Spiel nicht weiter behebbar.
- **`target_correction_mm` = [0, +5]** empirisch eingestellt (Roboter bleibt in y ≈ 1 mm vor dem Ziel stehen, dazu die Schiefe).
- **Schritt 1 schneller, `full` in einem Zug:** Iris öffnet während der Fahrt, nach dem Greifen direkt zur Stud-Beobachtungspose, 250 mm/s, kürzere Wartezeiten.
- **Schrauben auch ohne Kontakt** (`place.screw_without_contact`), `max_descend_mm` 17.
- **Stud-Erkennung:** Trockentest (12 Bilder, `data/images/stud_dry_20261021/`): dunkle Fläche nimmt einseitig Schatten mit → Mitte 0.5–1 mm daneben, untere Kante fehlt. Neu: Mitte an den scharfen Sockelkanten verfeinert (`refine_square`, THEORY 2.0), zentriert 0.2–0.7 mm Korrektur.
- **Werkzeuge:** `gripper_test` (Iris/M4 von Hand), Arduino-Port per `/dev/serial/by-id`, `vision.launch.py target:=stud distance_mm:=425`, Bilder jeder Erkennung werden mit dem Run gespeichert, Debug-Bild = Maske | Ergebnis.

## Messergebnisse 21.10.

| Läufe | Anzahl | Ergebnis |
|---|---|---|
| `pick` / Pick-Teil von `full` | 14 Landungen | 13 flach auf dem Tisch (Mutter in der Iris), 1× auf der Mutter (14:01) |
| `sa_place_approach` | 13 | 12 Schraube gefunden; Position wiederholbar ±0.3 mm; Schraube zentriert 11.3–12.1 mm statt 10 |
| `full` | 11 | 4 bis zum Ende (Kontakt, M4, Iris auf); 5 an der Stud-Erkennung gescheitert (provisorische Halterung, Quadrat 12.6–16.7 mm), 1 kein Kontakt (vor `screw_without_contact`), 1 abgebrochen |

Ein `full`-Lauf dauert ≈ 45–50 s (davon 8 s Schrauben).

## Vorläufige Werte (zurückstellen!)

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_vision/config/vision.yaml` `targets.stud.size_tolerance` | 0.8 | 0.4 | provisorische Stud-Halterung vergrössert das Quadrat |
| `software/src/sa_tasks/config/tasks.yaml` `frames.target_correction_mm` | [0, 5.0] | neu messen | empirisch; ein Teil war evtl. der Schatten-Fehler der Stud-Mitte (jetzt verfeinert) |
| `arduino/iris_gripper/iris_gripper.ino` `WATCHDOG_OPENS_IRIS` | true | false | Bequemlichkeit beim Testen: Iris öffnet 2 s nach jedem Prozessende (Mutter fällt bei Abbruch) |
| `tasks.yaml` `place.screw.duration_ms` / `speed` | 8000 / 200 | messen | M4-Timing nie gemessen (Mitnehmer dreht leer) |

## Offene Probleme

1. **M4-Mitnehmer (3D-Druck) greift nicht** → dreht leer. Neu drucken.
2. **Nach dem Öffnen der Iris senkt der Arm nochmals ab** (weicher Modus, Ziel 10 mm unter Kontakt). Versuch „erst steif, dann öffnen“ war schlechter → zurückgenommen. Ursache noch offen.
3. **Stud-Verfeinerung am Roboter ungetestet.** Am Roboter war die Schraube zentriert 11.3–12.1 mm gross (Schatten); ob die Mitte jetzt stimmt, zeigt erst `sa_place_approach` + Handmessung.
4. **Untere Hälfte ≈ 1.8° schief** (Spiel) → steckt in `target_correction_mm`.

## Morgen als Erstes

1. Aliase in `~/.bashrc` eintragen (falls noch nicht):
   ```bash
   alias sa_arduino_test='sa_env && ros2 run sa_tasks gripper_test'
   alias sa_vision_bolt='sa && source software/install/setup.bash && ros2 launch software/launch/vision.launch.py target:=stud distance_mm:=425'
   ```
2. Am Roboter `sa_robot_build` (package.xml von sa_tasks hat sich geändert).
3. **Stud-Verfeinerung prüfen:** 3–5× `sa_place_approach`, Iris-Mitte gegen Schraubenachse von Hand messen. Danach `target_correction_mm` neu setzen (Startwert [0, 5]; wenn jetzt zu weit links → kleiner). Die Bilder liegen neben jedem Record in `data/runs/` (`…_stud_centred_debug.png`).
4. Wenn die richtige Stud-Halterung da ist: `size_tolerance` zurück auf 0.4.
5. M4-Mitnehmer neu drucken, dann mit `sa_arduino_test` → `c`, `r 2000 200` Umdrehungen zählen → `place.screw.duration_ms` ausrechnen (M8: ≈ 6–7 Umdrehungen).
6. Absenken nach dem Iris-Öffnen untersuchen (Problem 2).
7. Dann `sa_full_all` auf Erfolgsquote.

## Nützliche Befehle

- `sa_full_all` – Roboter + Vision + ganzer Ablauf
- `sa_pick_all` – nur Pick
- `sa_place_approach` – Stud suchen, über der Spitze halten (ohne Gripper)
- `sa_vision_bolt` – nur Kamera + Vision für die Schraube (Trockentest, ohne Roboter)
- `sa_arduino_test` – Iris/M4 von Hand (`o`, `c`, `r [ms] [speed]`, `l …`, `s`, `q`)
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test`

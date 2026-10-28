# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`, Ergebnisse in `docs/PLAN.md` Abschnitt 9.1.

## Stand vom 28.10.2026

**Hauptergebnis für den Bericht steht: 29 von 30 Läufen aufgeschraubt, 1 teilweise, 0 Abbrüche** (Runden 5–7, je 5 Läufe pro Tischseite, gleicher Stand). Links 15/15, rechts 14/15, 3–5 Umdrehungen, Zykluszeit Ø 34.5 s (bis die Iris öffnet). Die Schraube links, die am 27.10. noch in jedem Lauf klemmte, geht jetzt zuverlässig: Die Ursache war die Neigung der Bodenplatte gegen die Schraube. Behoben ist sie mit einem festen Korrekturwinkel pro Seite (Drehung um den TCP, ab dem Anfahren bis zum Loslassen). Die Gewindesuche ist aus, und nach jedem Lauf fährt der Arm selbst in die Startpose. Yannick muss vermutlich nicht mehr an den Roboter: Als Nächstes kommt ein grösseres Aufräumen, dann der Bericht.

## Heute erledigt

- **Neigung links gefunden und korrigiert:** Messwerkzeug `tilt_level` (gestern auf dem Laptop gebaut). Links sah die Platte bei +1…+1.5° gerade aus. Die Idee „Unterteil mit M4 um 180° drehen“ wurde getestet und verworfen: Die Platte blieb schief, die Neigung sitzt also nicht im drehenden Teil.
- **Fester Korrekturwinkel pro Seite** (`place.tilt_correction`): links +1.5°, rechts −1.0°. `tilt_level` sagte rechts +1°, −1° sah im Lauf aber besser aus. Das ist die einzige Ausnahme von „nur eine Orientierung“ und steht so im PLAN.
- **Festziehen pro Seite** (`tighten.depth_mm`): links 1 mm, rechts 3 mm unter der gemessenen Pose. Rechts verloren die Iris-Finger die Mutter.
- **Iris vor dem Festziehen 10 ms öffnen** (`screw.loosen_iris`). Danach hält die Iris nicht mehr aktiv, die Mutter kann sich ausrichten.
- **Gewindesuche aus** (`thread_search.enabled: false`), ≈ 2–4 s schneller, kein Nachteil.
- **Schraube links +0.5 mm in y** (`place.approach_shift_xy_mm`, nach Gefühl).
- **Nach jedem erfolgreichen `full`-Lauf zurück in die Startpose von `sa_robot`** (`robot.home_after_full_run`). Der Timer stoppt weiter bei `iris_opening`.
- **`run_map` über mehrere Runden** (`--round "round 5" "round 6" "round 7"`, Default in `tasks.yaml`): Karte der 30 Läufe in `data/runs/run_map.png`.
- **Pick-Landefehler erklärt** (THEORY 5): Der TCP landet konstant 4–5 mm hinter der Mutter (−x), weil der Arm der schnellen Ein-Zug-Bewegung nachläuft und die Reibung am Tisch den Rest festhält. Die Iris fängt das (30/30 gegriffen), deshalb bleibt es so.
- **Zwei PC-Probleme behoben:**
  - Der Ubuntu-Updater fraß nach dem Neustart ≈ 37 % CPU → Reflex `communication_constraints_violation`.
  - Ein kaputter DDS-Shared-Memory-Port (`/dev/shm/fastrtps_port7413`, 0 Byte) liess jeden Start stumm hängen. Die GUI räumt das jetzt vor jedem Befehl auf (`gui/clean_dds_shm.sh`).
- Tests: `test_precise_move…` (seit gestern rot) und `test_approach_only…` an den aktuellen Stand angepasst. Alle grün.

## Messergebnisse 28.10.

72 `full`-Läufe, 66 durchgelaufen, 6 Abbrüche in der Bilderkennung (keine Mutter / kein Objekt / kein Quadrat, vor allem in den Probeläufen).

| Runde | Stand | Läufe | Ergebnis |
|---|---|---|---|
| half turn | Halbdrehung links | 6 | Platte weiter schief, verworfen |
| tilt 1.5 | Neigung links +1.5° | 2 | funktioniert |
| round 4 | Neigung links, Gewindesuche an | 10 (+2 unbewertet) | 9/10, 1 teilweise, Ø 39.9 s |
| trial right 1.0 / no thread search / loosen iris | Probeläufe einzelner Änderungen | 15 | gut (Yannick), nicht bewertet |
| **round 5** | Endstand | 10 | **10/10**, Ø 34.2 s |
| **round 6** | + Rückfahrt Startpose | 10 | **9/10**, 1 teilweise, Ø 34.3 s |
| **round 7** | gleich | 10 (+4 Filmaufnahmen, nicht gezählt) | **10/10**, Ø 35.0 s |
| **round 5–7** | | **30** | **29/30, Wilson-95 %-Intervall 83–99 %** |

## Vorläufige Werte (zurückstellen oder bestätigen!)

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_tasks/config/tasks.yaml:227` `tilt_correction` | links +1.5 / rechts −1.0° | so lassen | von Auge, rechts gegen `tilt_level` (+1°); funktioniert (29/30) |
| `tasks.yaml:230` `approach_shift_xy_mm` | links [0, 0.5] | so lassen | nach Gefühl, nicht gemessen |
| `tasks.yaml:325` `loosen_iris` | 10 ms | so lassen | Wirkung nicht von „Iris hält nicht mehr“ getrennt |
| `tasks.yaml:213` `half_turn` | `sides: []` | beim Aufräumen entfernen | verworfen |
| `tasks.yaml:252` `thread_search.enabled` | false | beim Aufräumen: Code behalten oder entfernen | aus seit Runde 5 |
| `tasks.yaml:287` `catch.max_steps` | 0 (Fang-Phase aus) | beim Aufräumen entfernen | seit 27.10. aus |
| `tasks.yaml:184` `approach_from_mm` | [0, 0] | beim Aufräumen entfernen | seit 26.10. aus |
| `software/src/sa_vision/config/vision.yaml:63` `targets.stud.size_tolerance` | 0.8 (TEMPORARY) | 0.4 | war für die provisorische Halterung |

## Offene Probleme

1. **Ein teilweiser Lauf** (round 6, 11:30:55, rechts, 1.5 Umdrehungen). Positionierung gut (0.28 mm), der Arm kam 3.6 mm runter. Vermutlich ist die Iris über die Mutter gerutscht. Bei einem Lauf von 30 nicht weiter verfolgt.
2. **Pick 4–5 mm hinter der Mutter** (systematisch, erklärt, bleibt so).
3. **Vision-Aussetzer** in den Probeläufen (6×), in den gezählten Runden keiner.
4. **Der rechte Korrekturwinkel widerspricht dem Modell** (THEORY 5: rechts sollten sich die Beiträge aufheben). Im Bericht als Einschränkung erwähnen.

## Morgen als Erstes

1. **Grosses Aufräumen** (Roboter wird vermutlich nicht mehr gebraucht): verworfene Funktionen samt YAML und Tests raus (`half_turn`, Fang-Phase/Wobble, `approach_from_mm`, evtl. `thread_search`), Kommentar-Historie in `tasks.yaml` kürzen, `size_tolerance` klären, Code und Doku auf den Endstand bringen.
2. **Bericht:** Ergebnisse aus PLAN 9.1 (Tabelle, Wilson-Intervall), `data/runs/run_map.png`, THEORY-Abschnitte mit Status „used“ übernehmen. Pick-Landefehler und Neigungskorrektur als eigene Punkte.
3. **Präsentation:** Videos von heute (round 7 extra) und die Karte.

## Nützliche Befehle

- Karte und CSV der 30 Läufe: `sa_env && ros2 run sa_tasks run_map` (Default: round 5–7), andere Runden mit `--round "round 4"`.
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test -q` (sa_tasks, sa_vision), GUI: `cd gui && python3 -m pytest test_launcher.py`.
- Build: `sa_robot_build` (mit Franka-Workspace, sonst verliert `install/setup.bash` die Kette).
- Hängt ein Start stumm: `bash gui/clean_dds_shm.sh` (macht die GUI automatisch).

# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`, Ergebnisse in `docs/PLAN.md` Abschnitt 9.1 / 9.2.

## Stand vom 29.10.2026 (Mittag)

**`main` ist der funktionierende Stand (29/30, Tag `working-29of30`).** Heute Morgen lief ein Vorversuch: alle Handkorrekturen aus, dazu eine Reihe mit der Steifigkeit beim Festziehen. Ergebnis: Ohne Korrekturen und mit dem gleichen Controller kamen 2/4 aufgeschraubt heraus statt 29/30. Mit xy 100 N/m klappte es gar nicht (0/5), die übrigen Werte liegen im Rauschen. Die Reihe ist abgebrochen, weil am Nachmittag neue Druckteile eingebaut werden, die den Tilt im Gripper beheben sollen. Der Teststand liegt auf dem Branch `test-uncorrected`.

## Heute erledigt

- **Funktionierenden Stand gesichert:** Git-Tag `working-29of30` = `88faac8`. Zurück jederzeit mit `git checkout working-29of30 -- software/src`.
- **Vorversuch ohne Korrekturen** (Abschnitt 9.2 im PLAN). Auf 0 gesetzt waren: Kamera-Neigung [1.54, 0.79], Kamera-Drehung um z 1.3°, `target_correction`, `tilt_correction` (links +1.5 / rechts −1.0) und `approach_shift_xy_mm` (links +0.5 y).
- **`tighten_stiffness`-Reihe** mit je 2 Läufen pro Seite: T0 Basis 2/4, T1 xy 1500 3/4, T2 xy 100 0/5, T3 z 300 unvollständig.
- **Seitenversatz vor dem Absenken geklärt:** Das ist die Feinpositionierung (`robot.fine_positioning`), also Regelung auf das berechnete Ziel und keine gespeicherte Korrektur.
- **K_rot geklärt:** `screw` und `tighten` haben rx/ry 4 Nm/rad (effektiv ≈ 2). Die 40 gehören zu `float`/`contact`, die ein Lauf nicht benutzt.

## Messergebnisse 29.10.

26 `full`-Läufe, alle `result: ok` (keine Abbrüche). Kontaktfehler zum berechneten Ziel ≤ 1.1 mm, die Feinpositionierung erreicht ihr Ziel also auch unkorrigiert. Wie weit das Ziel neben der echten Schraube liegt, zeigt erst das Ergebnis.

| Runde | tighten K_xy / K_z / K_rot | Läufe | Aufgeschraubt |
|---|---|---|---|
| test uncorrected (…stiff 1500, rot 100/50, no z rot) | gemischt | 9 | 1 bewertet (ja); Yannick: „eigentlich ganz okay“ |
| **T0 base** | 400 / 600 / 4 | 4 | **2/4**, 1 teilweise, 1 nein |
| T1 xy 1500 | 1500 / 600 / 4 | 4 | 3/4, 1 teilweise |
| T2 xy 100 | 100 / 600 / 4 | 5 | **0/5**, 4 teilweise, 1 nein |
| T3 z 300 | 400 / 300 / 4 | 4 | 2 bewertet (nein, teilweise), **2 unbewertet** (10:44:32, 10:45:35) |

## Vorläufige Werte (zurückstellen oder bestätigen!)

Auf `main` stehen wieder alle Werte des funktionierenden Stands. Offen sind weiterhin:

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_tasks/config/tasks.yaml:226` `tilt_correction` | links +1.5 / rechts −1.0° | nach dem Umbau mit `tilt_level` neu, im besten Fall 0 | Neigung der Bodenplatte, neue Teile sollen sie beheben |
| `tasks.yaml:230` `approach_shift_xy_mm` | links [0, 0.5] | nach dem Umbau prüfen | nach Gefühl |
| `tasks.yaml:53` `target_correction` | Ebene aus 4 Ecken | nach dem Umbau evtl. mit `tcp_offset` neu messen | Kamera → Iris ändert sich mit den neuen Teilen |
| `tasks.yaml:325` `loosen_iris` | 10 ms | so lassen | |
| `tasks.yaml:212` `half_turn`, `:249` `thread_search`, `:280` `catch`, `:184` `approach_from_mm` | aus | beim Aufräumen entfernen | verworfen |
| `software/src/sa_vision/config/vision.yaml:63` `targets.stud.size_tolerance` | 0.8 (TEMPORARY) | 0.4 | war für die provisorische Halterung |
| `software/src/sa_tasks/config/poses.yaml` | `observe_nut` rechts, `observe_stud` links (lokal, nicht committet) | wie gewünscht | Seitentausch per GUI |

## Offene Probleme

1. **Wie viel bringt jede einzelne Korrektur?** Mit 4 Läufen nicht aufgelöst. Nach dem Umbau ändert sich ohnehin die Ausgangslage.
2. **T3 hat 2 unbewertete Läufe.** Für die Auswertung in der GUI nachtragen oder weglassen.
3. Von gestern: ein teilweiser Lauf in round 6. Pick 4–5 mm hinter der Mutter (systematisch, bleibt so). Der rechte Korrekturwinkel widerspricht dem Modell (THEORY 5).

## Heute Nachmittag als Erstes (nach dem Einbau der neuen Druckteile)

1. **CAD-Offsets prüfen:** Ändern die neuen Teile Flansch → TCP oder Kamera → TCP? Dann `frames.*` in `tasks.yaml` und CLAUDE.md anpassen, sonst lehnt `load_tasks` die Kette ab.
2. **Robot-Launch neu starten** (`sa_robot`): `controllers.yaml` ist auf `main` wieder bei xy 400.
3. **`tilt_level` links und rechts** (GUI-Knopf / `ros2 run sa_tasks tilt_level`): Sieht die Platte bei 0° gerade aus?
4. **Unkorrigiert, Controller wie im funktionierenden Stand** (wie T0, aber z. B. 5 Läufe pro Seite): auf `git switch test-uncorrected` arbeiten und dort `tighten_stiffness` auf [400, 400, 600, 4, 4, 20] und `round` auf z. B. „new parts uncorrected“ setzen. Oder auf `main` nur `tilt_correction` auf 0 setzen, wenn die Kamerakorrekturen bleiben sollen (Empfehlung: die Kamera-Neigung und die Drehung um z gehören zur Kamera, nicht zum Gripper).
5. Je nach Ergebnis nur die nötigen Korrekturen wieder an, dann eine gezählte Runde (10 Läufe).

## Nützliche Befehle

- Zurück zum funktionierenden Stand: `git restore software/src && git switch main` (oder `git checkout working-29of30 -- software/src`).
- Auswertung einer Runde: `sa_env && ros2 run sa_tasks run_map --round "T0 base"`.
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test -q`.
- Build: `sa_robot_build` (mit Franka-Workspace).
- Hängt ein Start stumm: `bash gui/clean_dds_shm.sh` (macht die GUI automatisch).

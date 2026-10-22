# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`.

## Stand vom 22.10.2026

Die Positionierung ist gelöst: Mutter greifen und die Schraube treffen klappt jetzt ohne Ziel-Offset und auf beiden Tischseiten, beim Anfahren der Schraube auf **(−0.2, +0.3) mm**. Die letzten 6 `full`-Läufe kamen alle bis zum Kontakt auf der Schraube. **Geschraubt wird aber noch nicht:** Am Ende liegt die Mutter jedes Mal nur oben auf der Schraube. Der M4-Antrieb ist der Engpass. Die Übersetzung geht ins Schnelle (Moment am Körper nur 39 %), und von Hand gedreht sperrt die Mechanik in eine Richtung.

## Heute erledigt

- **Feinpositionierung** (`robot.fine_positioning`, THEORY 5): Der Arm blieb **1.5–4.5 mm vor dem Ziel stehen, immer entgegen der letzten Fahrtrichtung**. Ursache ist Haftreibung gegen die Steifigkeit von 2.5 N/mm; der Integrator ist auf 2.5 N begrenzt. Jetzt wird vor der Pick-Spirale und beim Anfahren der Schraube die Referenz um den **TCP**-Fehler verschoben (Verstärkung 0.8, max. 3 Schritte, < 0.5 mm). Korrigiert wird der TCP aus der gemessenen Flanschpose inkl. Neigung: Der Flansch stand auf 0.1 mm genau, der TCP war trotzdem 2.7 mm daneben.
- **Reach-Test** (`ros2 run sa_tasks reach_test`, 10 Punkte): kein Reichweitenproblem, alle Gelenke ≥ 14° von den Grenzen. Fehler 1.5–4.5 mm gegen die Fahrtrichtung, nach der Feinpositionierung 8/10 Punkte ≤ 0.5 mm.
- **Kamera-Neigung im Modell** (`frames.camera_tilt_in_flange_deg: [1.25, 0]`, THEORY 3.1): Iris von Hand über die Schraube geführt und die Pose ausgelesen. Die Schraube lag 8.6 mm weiter in +y als berechnet (d = 393 mm) → 1.25° um x_F. **`target_correction_mm` = [0, 0].**
- **Beobachtungsposen getauscht:** Mutter links, Schraube rechts; Höhe unverändert 473.4 mm.
- **Firmware:** Vor jedem Bewegungsbefehl wird geprüft, ob der PWM-Chip des Shields schläft, und er wird dann neu initialisiert (`shield_resets` in `STATE`, sonst `ERR shield`). Ursache: Ein blockierender M4 brachte die Spannung zum Einbrechen → `DONE`, aber kein Motor drehte. **Hochgeladen und am Board getestet? → prüfen** (siehe unten).
- **Schrauben:** 4000 ms bei Speed 255; die letzten 1500 ms mit Referenz 50 mm unter Kontakt (≈ 7.5 N Anpressen, `place.screw.final_press`).
- **Analyse M4-Antrieb:** gear3 (44 Zähne, an M4) → Ritzel (17 Zähne, am Körper) ergibt 2.6× schneller und 0.39× Moment. Motor laut CAD ein N20-Getriebemotor, Übersetzung unbekannt. Von Hand dreht der Antrieb nach links, nach rechts sperrt er (Mechanik, nicht Elektronik). Im CAD ist nichts richtungsabhängig (gerade Stirnräder, rotationssymmetrisch).

## Messergebnisse 22.10.

| Läufe | Anzahl | Ergebnis |
|---|---|---|
| `full` | 20 | 13 bis zum Ende; 2× keine Mutter gefunden, 2× keine Schraube im Arbeitsbereich (Teile nach dem Posentausch noch falsch gelegt), 1× Arduino antwortete nicht (Shield/Reset), 2 abgebrochen |
| davon nach Kamera-Neigung (15:46–15:56) | 6 | 6/6 Kontakt auf der Schraube, **Mutter jedes Mal nur aufgelegt, nicht eingeschraubt** |
| `place_screw --approach-only` | 7 | 6 ok, 1 ohne Vision gestartet. TCP − Schraube: vorher (−0.6 … −2.0, +2.9 … +5.9) mm, nach Neigung + Feinpositionierung **(−0.2, +0.3) mm** |
| Pick-Landungen | 20 | TCP 0.6–2.5 mm über dem Tisch (normal); 2× 5.4 mm = Platte auf der Mutter (15:09/15:11, Korrektur 0 und noch ohne Feinpositionierung) |
| `reach_test` | 1 | siehe oben |

Ein `full`-Lauf dauert ≈ 45–50 s.

## Vorläufige Werte (zurückstellen!)

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_vision/config/vision.yaml:59` `targets.stud.size_tolerance` | 0.8 | 0.4 | war für die provisorische Halterung; die richtige ist da (zentriert 10.6–11.2 mm) |
| `arduino/iris_gripper/iris_gripper.ino:46` `WATCHDOG_OPENS_IRIS` | true | false | Bequemlichkeit beim Testen |
| `software/src/sa_tasks/config/tasks.yaml` `frames.camera_tilt_in_flange_deg` | [1.25, 0.0] | y_F-Anteil messen | nur y von Hand geprüft; in x waren es −2.4 mm (ungeprüft) |
| `tasks.yaml` `place.screw` (`duration_ms`, `speed`, `final_press`) | 4000 / 255 / 1500 ms, 50 mm | messen | M4-Drehzahl und Moment nie gemessen |

## Offene Probleme

1. **Mutter wird nicht eingeschraubt, liegt nur auf.** Vermutungen, nach Wahrscheinlichkeit:
   - Der Körper dreht viel zu schnell und mit zu wenig Moment (Übersetzung 44 → 17 ins Schnelle), die Mutter springt über den Gewindeanfang.
   - Die Mechanik sperrt in Anzugsrichtung (von Hand nach rechts blockiert, Ursache noch nicht gefunden).
   - Die Mutter dreht in der Iris durch (Halten nur über Reibung, `hold_speed` 60).
   - Falls es eine Sicherungsmutter mit Nylonring ist: Liegt der Ring unten, kann sie nicht anfangen; ausserdem braucht der Ring bis ≈ 6 Nm.
2. **Board resettet sich, wenn M4 blockiert** (heute ≈ 16 USB-Neuanmeldungen). VIN-Jumper entfernen und den Motorstrom direkt an die Klemme des Shields legen.
3. **`gripper_test` merkt einen Board-Reset nicht** (Port wechselt ACM0 ↔ ACM1) → dann neu starten.

## Morgen als Erstes

1. **Firmware hochladen**, falls noch nicht geschehen: `arduino/iris_gripper/iris_gripper.ino` in der IDE öffnen → Upload → IDE schliessen.
2. **M4 messen:** `sa_env && ros2 run sa_tasks gripper_test` → `r 2000 255`, Umdrehungen des Körpers zählen. Daraus Motor-Drehzahl und N20-Übersetzung bestimmen (Motor = Körper × 17/44).
3. **Mutter prüfen:** normale oder Sicherungsmutter (Nylonring)? Wie liegt sie in der Iris?
4. **Sperre in Anzugsrichtung finden:** schrittweise zerlegen (M4 mit gear3 ab → Körper allein drehen → Kabel durch den Schleifring lösen).
5. **Übersetzung umdrehen** (17 Zähne an M4, 44 am Körper, gleicher Achsabstand 33.1 mm) → 6.7× Moment, 6.7× langsamer. Im CAD prüfen, ob das grosse Rad am Körper Platz hat.
6. **Ohne Neudruck testen:** vor dem Anziehen kurz rückwärts drehen (Gewindeanfang finden) und langsam anziehen. Muss noch gebaut werden.
7. `size_tolerance` der Schraube auf 0.4 zurück, dann `sa_place_approach`. Bei der Gelegenheit x prüfen → zweite Kamera-Neigung.
8. Dann `sa_full_all` auf Erfolgsquote.

## Nützliche Befehle

- `sa_full_all`: Roboter + Vision + ganzer Ablauf
- `sa_place_approach`: Schraube suchen, über der Spitze halten (ohne Gripper)
- `sa_env && ros2 run sa_tasks gripper_test`: Iris/M4 von Hand (`o`, `c`, `r [ms] [speed]`, `l …`, `s`, `q`). Der Alias `sa_arduino_test` steht **noch nicht** in `~/.bashrc`
- `sa_env && ros2 run sa_tasks reach_test`: Positioniergenauigkeit (braucht `sa_robot`)
- Pose von Hand auslesen: Prog-Modus führen → Exec → `sa_env && ros2 launch software/launch/robot_state.launch.py` + `ros2 run sa_tasks teach_poses` → Enter (zeigt jetzt den ganzen TCP)
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test`

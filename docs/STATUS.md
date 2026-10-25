# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`.

## Stand vom 25.10.2026

Der neue Greifer mit 1:1-Antrieb ist montiert, die Geometrie ist gleich geblieben. **Das Zittern ist weitgehend weg.** Die Ursache waren verlorene Regelzyklen, weil der Regler keine Echtzeit-Priorität hatte. Der Pick läuft zuverlässig: TCP 13.6–14.4 mm, auch mit vertauschten Beobachtungsposen. **Die Schraube trifft je nach Tischposition 1–3 mm daneben, und das ist nicht wiederholbar.** Vermutung (Yannick): die Absolutgenauigkeit des Arms je nach Gelenkkonfiguration. **Geschraubt wird höchstens 1–3 Umdrehungen.** Die Mutter verkantet, wenn sie nicht genau konzentrisch sitzt. Eine Kamera-Kalibrierung mit ChArUco ist gebaut und gemessen, aber nicht übernommen: Damit landete ein Pick auf der Mutter.

## Heute erledigt

- **Zittern:**
  - Echtzeit-Priorität für den User: Gruppe `realtime`, `rtprio 99`, Datei `/etc/security/limits.d/99-realtime.conf`. Danach läuft der Regler mit FIFO-Priorität 98, und es gibt keine „Failed to lock“ mehr (vorher 8–186 pro Lauf). `control_command_success_rate` zeigt nur noch einzelne 0.99 statt Einbrüchen bis 0.95.
  - Debug-Topic aus (`debug.publish_rate` 0), keine Konsolenausgabe pro Referenz mehr.
  - **Eigenes Tempo-Limit für z: 150 mm/s**, xy bleibt 250. Gemessen mit `motion_test`: z-Fahrten zitterten bei 250 mit 0.2–0.9 mm RMS, bei 150 nur mit ≈ 0.01 mm. In xy bei 150 blieb der Arm hängen und sprang.
  - Ausprobiert und verworfen: Referenz-Tiefpass τ 0.5 → 0.1 s. Das zitterte stark, allerdings war dabei auch der falsche Greifer montiert.
- **Neues Werkzeug `ros2 run sa_tasks motion_test`:** fährt die Wege eines Laufs ohne Greifer und Vision und misst das Zittern pro Fahrt.
- **Anfahrt an die Schraube aus fester Richtung:** zuerst 10 mm daneben auf der −y-Seite, dann ein kurzer Zug in +y (`place.approach_from_mm` [0, −10]).
- **Feinpositionierung ohne Integrator:** Beim Warten wird die Referenz alle 50 ms neu gesendet. Vorher löschte der Abstieg den Integrator, und der Arm gab bis ≈ 1 mm nach.
- **Die eine Korrektur ist jetzt linear in y:** `frames.target_correction`, y −300 → [0, −3], y −130 → [0, −1], von Hand eingestellt. Ausprobiert und verworfen: eine pendelnde Bodenplatte (Kipp-Verstärkung) und eine zweite, separate Korrektur.
- **Vorzeichen der Kraft geklärt:** Andrücken auf die Schraube ist **negativ** in `force_*_n`. Die positiven Werte vom 23.10. waren Offset, kein Anpressen.
- **Anpressen erhöht:** Fangen 3 → 8 mm, Festziehen 15 → 30 → 60 mm. Die Kraft skaliert (−1 N bei 30, −6…−9 N bei 60), der z-Weg wird aber nicht grösser.
- **Kamera-Kalibrierung `ros2 run sa_tasks hand_eye`:** ChArUco `data/grid/charuco_9x13_20mm.pdf`, nur Translationen, 27 Posen. Zwei Läufe ergaben Neigung [1.53, 0.76] / [1.54, 0.81], z-Drehung 1.19 / 1.40°, RMS 1.0 / 1.1 mm. Übernommen und wieder zurückgenommen: Damit landete ein Pick 6.6 mm hoch, weil sich die Mutter um ≈ 4.8 mm verschob. Ob das ein Ausreisser war, ist offen.

## Messergebnisse 25.10.

| Läufe | Anzahl | Ergebnis |
|---|---|---|
| `full` | 35 | 27 `ok`, 7 von Hand abgebrochen, 1 „no object“ (Schraube nicht im Bild) |
| `place_and_screw` (approach only) | 11 | alle `ok`; TCP erreicht das berechnete Ziel auf ≤ 0.5 mm |
| Pick-Landungen nach 09:00 | 31 | TCP 13.0–14.4 mm (normal); Ausreisser 18.6 mm mit der Kalibrierung |
| Schrauben, Festziehen 30 mm (11:10–11:19) | 5 | z-Weg 0.79 / 1.75 / 2.8 / **3.71** mm (bisher bester), Kraft −0.6…−2.6 N |
| Schrauben, Festziehen 60 mm | 13 | z-Weg 0.3–2.0 mm, Kraft −5.8…−8.9 N; nie `caught` |
| `motion_test` z-Zittern RMS | 250 / 150 mm/s | 0.2–0.9 / ≈ 0.01 mm |
| `hand_eye` | 2 | [1.53, 0.76] / 1.19° und [1.54, 0.81] / 1.40° |

Ein `full`-Lauf dauert ≈ 55–65 s (am 23.10. ≈ 50 s). Mehr Zeit kosten das z-Limit, die Anfahrt aus fester Richtung und das stärkere Schrauben.

## Vorläufige Werte (zurückstellen!)

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_tasks/config/tasks.yaml` `place.screw.tighten.depth_mm` | 60 | wohl 30 | Test, ob die Kraft skaliert; mit 30 war das Schrauben besser (bis 3.7 mm) |
| `tasks.yaml` `place.screw.catch.wobble.radius_mm` | 2.0 | 0 testen | zieht die Mutter absichtlich 2 mm aus der Mitte, das verkantet |
| `tasks.yaml` `frames.target_correction` | −3 / −1 mm in y (von Hand) | aus Messung | nicht wiederholbar, je nach Position anders |
| `tasks.yaml` `frames.camera_tilt_in_flange_deg` | [1.25, 0.0] | Kalibrierung klären | hand_eye sagt [1.54, 0.79] + 1.3° um z |
| `software/src/sa_vision/config/vision.yaml:61` `targets.stud.size_tolerance` | 0.8 | 0.4 | war für die provisorische Halterung |
| `arduino/iris_gripper/iris_gripper.ino:49` `WATCHDOG_OPENS_IRIS` | true | false | Bequemlichkeit beim Testen |

## Offene Probleme

1. **Schraube 1–3 mm daneben, je nach Tischposition, nicht wiederholbar.** Der TCP erreicht das berechnete Ziel auf ≤ 0.5 mm, und die Schraube steht nach dem Zentrieren 2–5 mm neben der Kameraachse. Kamera und Kinematik stimmen also unter sich. Vermutung: Absolutgenauigkeit des Arms zwischen der hohen Kamerapose und der tiefen Anfahrpose, dazu das Spiel der Drehscheibe.
2. **Die Mutter verkantet beim Schrauben** (Yannick: leicht schief, wenn nicht konzentrisch), dann dreht sie schwer. Der Flansch steht dabei 1.05–1.3° schief. Der Wackelkreis von 2 mm verstärkt das vermutlich.
3. **Kalibrierung widerspricht der Praxis.** Zwei konsistente Läufe, aber mit den Werten verschob sich die Mutter so, dass ein Pick daneben lag. Kandidaten: die im Fit festgehaltene CAD-Position der Kamera und das Projektionszentrum (9 mm), der Hauptpunkt, die Absolutgenauigkeit.
4. **Board resettet, wenn M4 blockiert** (VIN-Jumper), noch offen.

## Morgen als Erstes

0. **Neu: Launcher-Fenster `gui/sa_gui`** (vom Laptop, 25.10. abends): Schraubenseite per Klick, ein Knopf pro Befehl, Kamera- und Debug-Bild. Checkliste zum Ausprobieren: `gui/NEXT_SESSION.md`. Temporär, vor der Abgabe `git rm -r gui`.

1. **Wackeln aus:** `place.screw.catch.wobble.radius_mm: 0.0` und **Festziehen zurück auf 30** (`tighten.depth_mm: 30.0`). Dann 3× `sa_full_all`, Stud links (y ≈ −130, dort sitzt sie gut). Anschauen: `drop_end_mm`, `caught`.
2. **Falls es weiter verkantet:** Kippen in den Schraub-Modi steifer, `controllers.yaml` `screw_stiffness`/`tighten_stiffness` rx, ry 4 → 40. Danach `sa_robot` neu starten.
3. **Zweite Messung der Schraube aus der Nähe** (Idee Yannick): nach dem Zentrieren die Kamera z. B. 150 mm tiefer fahren und nochmals messen. Dann sind Kamera- und Anfahrpose ähnlicher.
4. **Kalibrierung prüfen:** Werte probehalber wieder setzen (`git show 1410a56`) und 2–3 Picks machen. War der Fehlpick ein Ausreisser?
5. Nebenbei: VIN-Jumper, `size_tolerance` auf 0.4 zurück.

## Nützliche Befehle

- `sa_full_all`: ganzer Ablauf. Erst Ctrl+C, wenn `record: …` steht.
- `sa_place_approach`: Schraube suchen, 5 mm über der Spitze halten (Zentrierung prüfen).
- Schrauben im letzten Lauf: `sed -n '/  contact:/,/^result/p' $(ls -t data/runs/*_full.yaml | head -1) | grep -v "^    - - \|^      - "`
- Pick-Landung: `grep -A3 landed_tcp $(ls -t data/runs/*_full.yaml | head -1)`
- `sa_env && ros2 run sa_tasks motion_test`: Zittern messen (nur `sa_robot` nötig).
- `sa_env && ros2 run sa_tasks hand_eye`: Kamera-Kalibrierung (braucht `sa_robot` und `ros2 launch software/launch/camera.launch.py`).
- Zyklen-Verlust prüfen: `sa_env && ros2 topic echo /franka_robot_state_broadcaster/robot_state --field control_command_success_rate | grep -v -e '^---' -e '^1.0$'`
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test`

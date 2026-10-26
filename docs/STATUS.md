# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`.

## Stand vom 26.10.2026

**Der ganze Ablauf funktioniert: 5.5 von 6 Full-Runs am Schluss perfekt, auf beiden Tischseiten.** Zwei Ursachen waren gelöst worden. Erstens das Schrauben: Das starke Anpressen hat die Mutter verklemmt, jetzt wird nur noch mit ≈ 0.3 N gedreht. Zweitens die Präzision: Zwischen Kamera und Iris fehlten konstant ≈ 5 mm (gemessen [−1.8, −4.9] mm). Die ChArUco-Kalibrierung kann das prinzipiell nicht sehen, weil bei fester Orientierung die Kameraposition nicht bestimmbar ist. Dazu kommt eine zweite Messung aus der Nähe (170 mm statt ≈ 450 mm), damit Winkelfehler nur noch ≈ 0.4-fach wirken. Der Pick fährt ohne Spirale senkrecht mit Kontakterkennung runter.

## Heute erledigt

- **Launcher-GUI vom Laptop getestet:** läuft. Behoben: Ctrl-C/SIGTERM schloss das Fenster nicht (rclpy-Signalhandler). Bildfenster zeigen jetzt nur noch Binärbild und Resultat (die zwei Hälften von `/vision/debug`). Neuer Knopf „Camera -> iris offset“.
- **Schrauben:**
  - Neu: **Hochdrücken** (`place.seat`). Nach dem Kontakt 6 mm unter den Kontakt (≈ 9 N), damit die Mutter oben in der Iris anliegt. Weg der Mutter 0.16–0.42 mm.
  - **Fangen und Festziehen mit nur 2 mm statt 8/60 mm** (≈ 0.3 N). Das war der Durchbruch (Yannick): Das Anpressen verklemmte die Mutter.
  - Neu: **Spiralsuche auf der Gewindespitze** (`place.thread_search`, 0 → 4 mm, 3 Umdrehungen, 9 s, tiefste Stelle = Gewindeanfang). Reichweite in Wirklichkeit 2.8 mm, gefundene Senkung 0.06–1.17 mm.
- **Pick:** Spirale entfernt. Ohne Spirale blieb der Arm ≈ 5 mm über dem Tisch stehen (Reibung, Stillstand galt als gelandet). Jetzt **senkrechter Abstieg mit 10 mm/s bis Kontakt** (> 4 N), dann 2 mm nachdrücken (`pick.descent`). Landung 12.9–15.2 mm.
- **Nah-Messung** (`vision.close_up_distance_mm` 170) für Mutter und Schraube: Kameraachse über das erste Ergebnis, nochmals messen. Mutter links: erste Messung lag ≈ 6 mm daneben in x, die Nah-Messung korrigiert das. Schraube: 0.1–1.4 mm Korrektur.
- **Kalibrierung (ChArUco) wieder aktiv:** Neigung [1.54, 0.79], 1.3° um z. Die Handkorrektur ist ersetzt.
- **Ausprobiert und verworfen:** gemessene Neigung des Arms ignorieren. Das wurde schlechter, die Neigung von 1.0–1.7° ist also echt (Schalter wieder entfernt).
- **Neues Werkzeug `ros2 run sa_tasks tcp_offset`:** misst Kamera → Iris direkt. Ablauf: Approach, PROG, Gewinde von Hand mittig in die Iris, Execution, `robot_state`, Tool. Ergebnis y −274: [−1.91, −4.19], y −237: [−1.71, −5.70]. **Eingetragen als konstante `target_correction` [−1.8, −4.9]**. Probe ohne Handführung: mittig.
- **Seitliche Anfahrt aus −y abgeschaltet** (`approach_from_mm` [0, 0]). Mit der gemessenen Korrektur nicht nötig.

## Messergebnisse 26.10.

| Zeit | Stand | Läufe | Ergebnis |
|---|---|---|---|
| 11:30–11:37 | alt (Anpressen 8/60 mm) | 3 full, 1 approach | z-Weg 1.8–2.2 mm, Kraft −6…−7 N, nie `caught` |
| 12:42–12:48 | + Hochdrücken, Anpressen 2 mm | 4 full ok | „funktioniert“ (Yannick), einmal `caught` |
| 12:51–12:57 | + Spiralsuche Gewinde | 4 full | 2× Vision (keine Schraube im Bild / kein Quadrat), 1 abgebrochen, 1 ok |
| 13:00–13:09 | + Kalibrierung, Pick ohne Spirale | 6 full abgebrochen | Pick hörte bei TCP 19.2–19.8 mm auf (≈ 5 mm über dem Tisch) |
| 13:17–13:41 | + Pick mit Kontakt, Nah-Messung | 2 full, 8 place | Iris ≈ 5 mm daneben, `tcp_offset`-Messungen |
| **13:45–13:54** | **+ gemessene Korrektur** | **6 full ok** | **5.5/6 perfekt (Yannick)**, Landung 12.9–15.2 mm, ≈ 77–80 s pro Lauf |

`tcp_offset` (data/runs/tcp_offset.yaml): [−1.91, −4.19] / [−1.71, −5.70] / Probe [−1.82, −4.94].

## Vorläufige Werte (zurückstellen oder bestätigen!)

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_tasks/config/tasks.yaml:49` `frames.target_correction` | konstant [−1.8, −4.9] | bestätigen | aus 2 Messungen rechts; links nur über die Full-Runs bestätigt, nicht gemessen |
| `tasks.yaml:27,30` Kamera-Neigung / z-Drehung | [1.54, 0.79] / 1.3 | so lassen | Korrektur gilt nur zusammen mit diesem Modell. Wer es ändert, muss `tcp_offset` neu messen |
| `tasks.yaml:145` `place.approach_from_mm` | [0, 0] | [0, 0] bestätigen | Test von heute, lief gut |
| `tasks.yaml:200` `place.screw.catch.wobble.radius_mm` | 2.0 | 0 testen | kann die Mutter von der gefundenen Stelle wegziehen |
| `tasks.yaml:168` `place.thread_search` | 9 s Spirale | evtl. kürzer/weg | mit der Korrektur trifft es auch so; kostet ≈ 12 s |
| `software/src/sa_vision/config/vision.yaml:61` `targets.stud.size_tolerance` | 0.8 | 0.4 | war für die provisorische Halterung |
| `arduino/iris_gripper/iris_gripper.ino:49` `WATCHDOG_OPENS_IRIS` | true | false | Bequemlichkeit beim Testen |

## Offene Probleme

1. **Erfolg beim Schrauben ist im Record nicht messbar.** `drop_end_mm` bleibt in den guten Läufen bei etwa 0 (−0.05…0.42). Vermutung: Die Mutter läuft am Gewinde runter und aus der Iris heraus, statt den Arm mitzuziehen (sie wurde ja nach oben gedrückt). Beurteilung bisher nur von Auge.
2. **Ein halber Fehlversuch** von 6. Nicht im Record erkennbar, welcher.
3. **Lauf dauert 77–80 s** (gestern 55–65 s): Nah-Messung 2× ≈ 4 s, Spiralsuche ≈ 12 s, Hochdrücken 1.5 s.
4. **Vision-Aussetzer:** 2× „no object“ / „not a stud“ (12:51, 12:53, 13:27), vermutlich Schraube am Bildrand oder `size_tolerance`.
5. Board resettet, wenn M4 blockiert (VIN-Jumper), noch offen.

## Morgen als Erstes

1. **Erfolgsquote messen:** 10× `sa_full_all`, Stud- und Mutterposition jedes Mal ändern (beide Seiten). Pro Lauf notieren: Mutter aufgeschraubt ja/nein, wie viele Umdrehungen.
2. **Erfolgskriterium messbar machen:** z. B. nach dem Schrauben die Iris öffnen und prüfen, ob die Mutter auf dem Gewinde bleibt (Bild der Kamera beim Hochfahren), oder den z-Weg relativ zur Mutter statt zum Arm.
3. **Zeit sparen, je einzeln testen:** Spiralsuche aus (`thread_search` überspringen, falls es ohne genauso trifft), Wackeln aus (`catch.wobble.radius_mm: 0.0`).
4. **Links einmal `tcp_offset` messen** (Approach, PROG, Gewinde mittig, `sa_robot_state`, `sa_env && ros2 run sa_tasks tcp_offset`) und mit [−1.8, −4.9] vergleichen.
5. Nebenbei: `size_tolerance` 0.4, VIN-Jumper, `WATCHDOG_OPENS_IRIS` false.

## Nützliche Befehle

- `sa_full_all`: ganzer Ablauf. Erst Ctrl+C, wenn `record: …` steht.
- `sa_place_approach`: Schraube suchen, nah messen, 5 mm über der Spitze halten.
- `sa_robot_state` + `sa_env && ros2 run sa_tasks tcp_offset`: Kamera → Iris messen (nach Approach + Handführung in PROG).
- `sa_gui`: Launcher-Fenster (aus einem normalen Terminal).
- Schrauben/Suche im letzten Lauf: `grep -A8 "thread_search:\|  seat:" $(ls -t data/runs/*_full.yaml | head -1)`
- Pick-Landung: `grep -A3 landed_tcp $(ls -t data/runs/*_full.yaml | head -1)`
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test`

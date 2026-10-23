# Stand der Dinge

> Wird von der Abend-Sequenz (`/abend`) jeden Abend überschrieben. Details und Begründungen: Entscheidungslog in `docs/PLAN.md`.

## Stand vom 23.10.2026

Positionieren, Greifen und Aufsetzen auf die Schraube laufen zuverlässig: 20 von 20 `full`-Läufen kamen heute bis zum Kontakt auf der Schraube. Die Präzision ist praktisch perfekt, die Mutter greift ins Gewinde, die Schraube kippt nicht, und die Teile sind kaum unkonzentrisch. **Eingeschraubt wird trotzdem nicht:** Nach höchstens ≈ 1.5 Umdrehungen bleibt die Mutter stehen (z-Weg am Ende 0–1.9 mm). Fazit: Es liegt am **Antrieb**. Die Übersetzung 44 → 17 (Zahnrad am Motor → Ritzel am Drehkörper) gibt nur 0.39× Motormoment. **Nächster Schritt: CAD neu mit 1:1-Übersetzung.** Die Software bleibt, wie sie ist.

## Heute erledigt

- **Schraub-Ablauf neu** (`sequences.screw_on`, THEORY „Screwing“):
  1. **Fangen:** Controller-Modus `screw` (4): xy 2000 N/m, z 150 N/m, Kippen 4 Nm/rad, Integrator aus. Langsam drehen in 12 × 250 ms bei Speed 100, Referenz 3 mm unter Kontakt. Pro Schritt springt die Referenz einen Punkt weiter auf einem Wackelkreis mit 2 mm Radius. Erkennung über die Höhe: gefasst, sobald der TCP ≥ 0.6 mm gesunken ist.
  2. **Festziehen:** zurück auf die Kontakt-xy, Modus `tighten` (5): wie `screw`, aber xy 400 N/m. Dann 5 s bei Speed 255 mit 15 mm unter Kontakt.
  3. **Loslassen ohne Absacken** (`stiffen_where_it_is`): Referenz = gemessene Pose + 3 mm, 2.5 s warten (die Referenz-Glättung im Controller hat τ ≈ 0.5 s), dann steif, erst dann Iris auf. Vorher fiel der weiche Arm auf die Referenz unter der Schraube, und einmal stürzte dabei `ros2_control_node` ab.
- **Controller** (`cartesian_impedance_control`): neue Modi 4 `screw` und 5 `tighten` (`modes.screw_stiffness`, `modes.tighten_stiffness` in `controllers.yaml`), ohne Integrator.
- **Record** `place.screw`: `caught`, z-Weg pro Schritt (`catch_drops_mm`), xy des TCP pro Schritt, Neigung und Kontaktkraft (Fz minus Wert in der Luft) am Start, nach dem Fangen und am Ende.
- **Bugfix:** numpy-Zahlen im Record liessen `yaml.safe_dump` scheitern. Folge: Prozess-Abbruch mit exit −6 und kein Record (zwei Läufe um 09:53 und 09:55).
- **Pick ohne Halt über der Mutter:** die Fahrt geht direkt in die Spirale, ohne Feinpositionierung dort.
- **Neues Werkzeug** `ros2 run sa_tasks goto_poses`: fährt `observe_nut`, dann `observe_stud` an (Teilebereiche auf dem Tisch markiert).
- **Ausprobiert und verworfen:**
  - Zurückfahren mit gegriffener Mutter, auch mit −1/−2 mm Versatz: blockiert, −0.36 statt −1 mm erreicht.
  - Nachgreifen (Iris auf, zurück, zu): die Mutter hielt nicht.
  - Kamera-Neigung 1.33° und `target_correction_mm` [0, −1].
  - Positionierung ist zurück auf dem Stand vom 22.10. (1.25°, [0, 0]).

## Messergebnisse 23.10.

| Läufe | Anzahl | Ergebnis |
|---|---|---|
| `full` gesamt | 20 (+2 ohne Record, abgestürzt) | 19 `ok`, 1 abgebrochen; **20/20 Kontakt auf der Schraube** |
| Pick-Landungen | 20 | TCP 0.8–1.9 mm über dem Tisch (normal) |
| mit neuem Schraub-Ablauf | 16 | Gewindefangen nie über die Höhe erkannt (max. 0.35 mm in den langsamen Schritten); z-Weg am Ende 0–1.9 mm, Median ≈ 0.5 mm |
| Neigung des Flanschs beim Schrauben | 16 | 1.1–1.7°, steigt beim Festziehen um ≈ 0.1° |
| Kontaktkraft (Fz − Luft) nach dem Fangen | 16 | 0.5–2.2 N |
| xy des TCP beim Wackeln (xy 400 N/m, Kreis 2–4 mm) | 5 | nur 0.2–0.5 mm Ausschlag, Reibung hält |

Ein `full`-Lauf dauert ≈ 50 s.

## Vorläufige Werte (zurückstellen!)

| Wo | Jetzt | Ziel | Warum |
|---|---|---|---|
| `software/src/sa_vision/config/vision.yaml:61` `targets.stud.size_tolerance` | 0.8 | 0.4 | war für die provisorische Halterung; die richtige ist da |
| `arduino/iris_gripper/iris_gripper.ino:49` `WATCHDOG_OPENS_IRIS` | true | false | Bequemlichkeit beim Testen |
| `software/src/sa_tasks/config/tasks.yaml:24` `frames.camera_tilt_in_flange_deg` | [1.25, 0.0] | y_F-Anteil messen | nur y von Hand geprüft |
| `tasks.yaml:132–146` `place.screw` (`catch`, `tighten`) | Fangen 12 × 250 ms bei 100, Kreis 2 mm; Festziehen 5000 ms bei 255, 15 mm | nach dem neuen Antrieb neu einstellen | mit dem 44→17-Antrieb getuned, mit 1:1 dreht der Körper 2.6× langsamer |

## Offene Probleme

1. **Mutter wird nicht ganz eingeschraubt:** stoppt nach ≈ 1.5 Umdrehungen. Ursache laut Fazit: zu wenig Moment am Drehkörper wegen der Übersetzung ins Schnelle (0.39×). → **1:1-Übersetzung im CAD.** Der Handtest vom 22.10. hat das verdeckt, weil sich eine von Hand gehaltene Schraube selbst ausrichtet.
2. **Flansch steht beim Schrauben 1.1–1.7° schief** (Rest-Fehler des steifen Modus, das Werkzeug-Moment ist grösser als Integrator-Grenze plus Feder). Mit dem beweglichen Unterteil vermutlich unkritisch. Falls es nach dem neuen Antrieb noch verkantet: Orientierungs-Feinkorrektur beim Anfahren (bewusste Ausnahme von „nur (1, 0, 0, 0)“).
3. **Board resettet sich, wenn M4 blockiert** (VIN-Jumper teilt die Versorgung). Mit mehr Moment wird das wichtiger: VIN-Jumper weg, Motorstrom direkt an die Klemme des Shields.
4. **Firmware mit Shield-Neuinitialisierung hochgeladen?** Nicht bestätigt. → im Zweifel `arduino/iris_gripper/iris_gripper.ino` in der IDE öffnen → Upload → IDE schliessen.

## Morgen als Erstes

1. **CAD: M4-Antrieb mit 1:1-Übersetzung** neu konstruieren und drucken. Vorher klären, welcher N20 drin ist: `sa_env && ros2 run sa_tasks gripper_test`, dann `r 10000 255`, Wellenumdrehungen zählen, × 6 = U/min.
2. **Neuen Antrieb ohne Roboter testen:** `sa_env && ros2 run sa_tasks screw_test`, Schraube fest einspannen statt von Hand halten (wie am Roboter). Dann Enter für die YAML-Werte, oder `<ms> <speed>`.
3. **`place.screw` neu einstellen** (Fangen, Festziehen: Zeit und Speed, weil der Körper jetzt 2.6× langsamer dreht), dann `sa_full_all` und den Record anschauen: `drop_end_mm` sollte ≈ Umdrehungen × 1.25 mm sein.
4. Nebenbei: VIN-Jumper (Problem 3), Firmware prüfen (Problem 4), `size_tolerance` auf 0.4 zurück.
5. **Geplant:** Kamera-Prüfung, ob der Unterteil dreht. Beim Schrauben füllt die Bodenplatte das ganze Kamerabild, Bilddifferenz zeigt Drehung oder Stillstand.

## Nützliche Befehle

- `sa_full_all`: Roboter + Vision + ganzer Ablauf. Erst Ctrl+C, wenn `record: …` im Terminal steht.
- `sa_env && ros2 run sa_tasks screw_test`: Iris zu → 5 s → Fangen am Stück → Festziehen → Iris auf
- `sa_env && ros2 run sa_tasks gripper_test`: `o`, `c`, `r [ms] [speed]`, `l …`, `s`, `q`
- `sa_env && ros2 run sa_tasks goto_poses`: Beobachtungsposen anfahren (braucht `sa_robot`)
- Record des letzten Laufs: `sed -n '/  screw:/,/^result/p' $(ls -t data/runs/*_full.yaml | head -1)`
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test`

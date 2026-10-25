# Semesterarbeit – Projektkontext & Gameplan

> Lebendes Dokument, Kontext für Claude und mich. Bei jeder Designentscheidung nachführen.
> **Version 2** (07.10.2026) – nach Beantwortung der Fragen F1–F20 (Antworten im Anhang A).
> Abgabe: **31.10.2026** → alles, was nicht direkt die Erfolgsquote erhöht, wird gestrichen.

---

## 1. Ziel

Ein FR3 mit eye-in-hand USB-Kamera und neuem Iris-Gripper soll eine **Sechskantmutter** (Testteil M8, Methode für gängige metrische Grössen, blank, auf weissem Untergrund) innerhalb eines definierten Bereichs finden, greifen, auf eine fixierte Stiftschraube setzen und aufschrauben.
Erfolgskriterium: **höhere Erfolgsquote als in der BA**, Vergleich mit der BA im Bericht (Englisch).

Kernänderungen gegenüber der BA:
1. **Direkte Zielpose** aus Kamerabild (Ansatz 2) statt iterativer Pixel-Regelung.
2. **Iris-Gripper + rotierender Gripper** statt Backen + Schraubendreher.
3. **Keine schwarze Lackierung** mehr – Detektion von blankem Metall auf Weiss.
4. **Clean Slate:** neue ROS2-Software, aus der BA wird nur das Prinzip der Vision-Pipeline übernommen.

---

## 2. Ausgangslage aus der BA (Kurzfassung)

| Phase | Steady-state Fehler BA [mm] |
|---|---|
| Grasp | 0.16 ± 0.14 |
| Vision centering | 1.57 ± 0.57 |
| Seating | 1.78 ± 0.18 |
| Release | 0.86 ± 0.14 |
| Screwing | 1.35 ± 0.19 |

- 8 Runs, Ø Zykluszeit 62 s, 1 Run abgebrochen (Zentrierung oszillierte bis Timeout).
- Gemessen wurde Tracking (Ist vs. Soll), **nicht** Genauigkeit relativ zum Objekt.
- Schwächen → Antworten in der SA: iterative Zentrierung → Ansatz 2; feste Offsets → definierte TF-Kette; Serial ohne Quittung → ACK/DONE-Protokoll; Steifigkeit nicht variiert → 2 Presets.

---

## 3. Hardware (fix)

```
Workstation ──USB-C (serial)──► Arduino UNO R4 Minima + Adafruit Motor Shield v2.3
     │                               ├─ M1   GESPERRT, nie einen Motor anschliessen
     │                               ├─ M2 ┐ Iris (gegensinnig)
     │                               ├─ M3 ┘
     │                               └─ M4   Rotation ganzer Gripper (Schleifring)
     ├──USB-A──► HutoPi 720p USB-Kamera (OV9726), eye-in-hand
     └──Ethernet──► FR3 (franka_ros2, Cartesian Impedance Controller aus BA)
```
Portbelegung ist **vorläufig** → in Firmware nur an einer Stelle definiert (Konstanten).

| Grösse | Wert | Quelle |
|---|---|---|
| **Flansch-Mitte → Kamera (Mitte, unterster Punkt des Gehäuses; Linse 5 mm höher)** | (44.4, 0, 57.223) mm im Flansch-Frame {F} (= EE-Achsen); in der festen Pose in Basis-Achsen (44.4, 0, −57.223) | CAD, Stand 16.10.26 (neue Halterung), Achsen korrigiert 18.10. |
| **Feste EE-Orientierung (einzige)** | Quaternion xyzw (1, 0, 0, 0) = Controller-Startpose (q7 = 0.80): Werkzeug nach unten, Kamera vorne (+x_robo) | Yannick 18.10. ✅ |
| Kamera-Rotation | x_robo = −y_cam, y_robo = −x_cam, z_robo = −z_cam (Gripper nach unten) | F27 ✅ |
| Kamera (unterster Punkt Gehäuse) → Mitte Bodenplatte (TCP), nur Plausibilität | (−44.505, 0.105, 61.6) mm in {F}; Fusion in Basis-Achsen (−44.505, −0.105, −61.60). Kette Flansch→Kamera + Kamera→TCP = (−0.105, 0.105, 118.823) ≈ Flansch→TCP ✅ | CAD 19.10.26 (`cad/export/Configuration_1.*`) |
| **Flansch-Mitte → Mitte Bodenplatte Gripper (TCP, liegt auf Tisch auf)** | Δx = 0, Δy = 0, Δz = 118.823 mm (CAD: 0.105/0.105 → 0) | CAD, Stand 16.10.26 ✅ |
| Flansch-Mitte (Bezugspunkt) | Mitte der Auflagefläche am FR3-Flansch; der Zentrierzylinder des Adapters ragt 2 mm in den Arm | CAD 16.10.26 |
| Iris-Zähne über Gripper-Unterseite | 3.5 mm | CAD |
| Masse Gripper komplett (für Desk, End-Effector „Generic Device“) | 0.275 kg; Schwerpunkt aus Fusion (Schätzung aus STL bei gleichmässiger Dichte: auf der Achse, ≈ 70 mm unter dem Flansch) | Yannick 18.10. |
| Mutter | **M8 Sicherungsmutter mit Nylonring** (SW 13 mm), nur bis zum Greifen des Nylonrings aufschrauben. Ziel: Detektion grössenunabhängig (M4–M12) | nachgemessen 15.10.26 (F15 sagte M6) |
| Stiftschraube | **M8**, quadratische Basis 10×10 mm, H 20 mm, Gewinde 19 mm | F15 (Höhen → F22), M8 bestätigt 15.10. |
| Kamera-Intrinsics | 1280×720: f ≈ 2094 px, (c_x, c_y) = (695, 404), FOV 34°×20° → `calibration/hd_camera.yaml` | Mittel aus 3 Kalibrierungen 19.10.26 ✅ |

Gripper-Verhalten: **alles zeitgesteuert, kein Feedback**. Halten = Iris-Motoren laufen weiter in Schliessrichtung (in der BA ohne Überhitzung erprobt).

---

## 4. Ablauf (Soll)

Arbeitsbereiche für Mutter (**Area N**) und Schraube (**Area S**) sind fest. Zu jeder Area gibt es eine fest definierte **Beobachtungspose**, von der aus die ganze Area im Bild liegt.

### Phase A – Pick
| # | Schritt | Steifigkeit | Abbruch/Übergang |
|---|---|---|---|
| A1 | Iris öffnen, zur Beobachtungspose N fahren | HIGH | Pose erreicht |
| A2 | Bild aufnehmen (n Frames mitteln), Mutter detektieren | – | Detektion gültig, sonst Retry/Abbruch |
| A3 | Pixel → Punkt im Basis-Frame (§5.1), Ziel = Gripper-Mitte über Mutter | – | – |
| A4 | **Eine durchgehende Bewegung** von der Beobachtungspose bis Gripper-Unterseite flach auf dem Tisch (TCP 2 mm unter Tischhöhe), xy zuerst fertig, letzter Teil senkrecht (THEORY §5) | HIGH (normal) | Pose erreicht (Toleranz + 2 mm) |
| A5 | Iris schliessen (t_close), danach Halten (Motoren laufen weiter) | – | Zeit |
| A6 | Zurück zur Beobachtungspose (z zuerst) | HIGH | Pose erreicht |

### Phase B – Place & Screw
| # | Schritt | Steifigkeit | Abbruch/Übergang |
|---|---|---|---|
| B1 | Beobachtungspose S | HIGH | Pose erreicht |
| B2 | Schraube detektieren, Pixel → Basis | – | Detektion gültig |
| B3 | Eine Bewegung zur Anfahrpose: Mutter koaxial 5 mm über Gewindespitze | HIGH | Pose erreicht |
| B4 | Senkrecht absenken (5 mm/s) bis Mutter aufsitzt (Wiggle erst, wenn nötig) | HIGH | \|ΔFz\| > 4 N, sonst Abbruch nach 12 mm |
| B5 | Optional: kurz rückwärts drehen (Gewindeanfang finden) | LOW | Zeit |
| B6 | M4 vorwärts drehen, Roboter folgt in −z (Modus float, Soll 10 mm unter Kontakt) | LOW | Zeit t_screw |
| B7 | Iris öffnen, Soll = Ist, Modus normal, zurück zur Beobachtungspose S | HIGH | – |

Pick (Schritt 1, `ros2 run sa_tasks pick`) und Place & Screw (Schritt 2, `ros2 run sa_tasks place_screw`) sind **getrennt startbar**; ein kombinierter Ablauf kommt mit dem Bringup-Launch. Beobachtungsposen und Tischhöhe werden mit `teach_poses` am Roboter aufgenommen (`sa_tasks/config/poses.yaml`).

---

## 5. Methodik

### 5.1 Pixel → Basis-Frame (Ansatz 2)
Kamera schaut senkrecht nach unten, Objektebene bekannt (Tisch + Objekthöhe):

```
(u, v)        entzerrter Pixel des Objekt-Mittelpunkts
d             = z_cam − z_obj   (Abstand Kamera → Objektebene, aus FK + Offset)
X_c = (u − c_x) · d / f_x
Y_c = (v − c_y) · d / f_y
p_base = T_base_flange · T_flange_cam · [X_c, Y_c, d, 1]ᵀ
Ziel Gripper-Mitte (xy) = p_base(xy),  z aus Phase
```
- Allgemeine Umsetzung als **Strahl-Ebene-Schnitt** (funktioniert auch bei leicht schiefer Kamera, kostet nichts extra).
- `T_flange_cam` vorerst aus **CAD-Offset**. Eine volle Hand-Eye-Kalibrierung lassen wir aus Zeitgründen weg. Stattdessen **Verifikation**: Mutter an bekannter Position → Ziel anfahren → Restfehler messen → konstanten xy-Korrekturterm in YAML.
- Objektebenen: Mutter = Tisch + ~5 mm; Schraube = Oberkante der quadratischen Basis (Kontur), siehe F22.
- Fallback (Ansatz 1): 1–2 Korrekturiterationen mit einem zweiten Bild aus der Anfahrpose. Reiner Code-Pfad im selben Modul, per Parameter an/aus.

### 5.2 Detektion ohne Lackierung (blank auf Weiss)
Die blanke Mutter ist teils heller (Glanzlichter), meist dunkler als der weisse Tisch, das Loch erscheint dunkel (Tisch im Eigenschatten). Details und Messwerte: THEORY §2.
1. Belichtung und Weissabgleich **fest** (v4l2, `software/config/camera.yaml`). Grau → Gauss → **Division durch Referenzbild** der leeren Area (A', `capture_reference`), danach durch einen **lokalen Median** (A, auch Fallback ohne Referenzbild) → **zweiseitige Schwelle** (heller *oder* dunkler als der Tisch) → Morphologie. Otsu verworfen (0/27 auf den Testbildern).
2. Nur Konturen innerhalb einer **ROI = Area im Bild** (Areas sind ja bekannt).
3. **Mutter:** Silhouette (konvexe Hülle) mit **dunklem Kern** (Loch), grössenunabhängig: Schlüsselweite s in mm = s_px·d/f, Filter s/e (Sechskant 0.87, Quadrat 0.71), Kern vorhanden und mittig. Mittelpunkt = Schwerpunkt der Silhouette. Optional Ziel-Schlüsselweite (M8: 13 mm). Gewindegrösse nur, wenn eindeutig (ISO + DIN).
4. **Schraube:** Quadrat 10×10 (approxPolyDP → 4 Ecken, Seitenverhältnis ≈ 1) mit Kreis (Gewinde) in der Mitte. Wir nehmen den Mittelpunkt des Quadrats.
5. Erwartete Grösse in Pixeln aus d und f berechnen → Flächenfilter **automatisch** statt von Hand getunt.
6. Reflexionen von blankem Metall: Mittelung über n Frames, Median-Mittelpunkt; ggf. diffuses Licht.

Alles als reine Funktionen (`numpy`/`cv2`, kein ROS) → am Laptop mit Fotos testbar.

### 5.3 Regelung
- Bestehender CIC aus der BA, **2 Steifigkeits-Presets** (HIGH für freie Bewegung, LOW für Kontakt/Schrauben) in YAML.
- Kontakt: gefiltertes Fz < Schwelle (BA: −4 N, Tiefpass α = 0.1).
- Wiggle (B4): kleine Kreisbahn in xy (≈ 0.5–1 mm, ~2 Hz) während z sinkt; nur falls ohne Wiggle Fehlversuche auftreten.

---

## 6. Gripper-Firmware & Serial-Protokoll

Zeitsteuerung läuft **auf dem Arduino** (nicht-blockierend mit `millis()`). Der PC schickt nur Befehle und wartet auf DONE.

```
PC → Arduino:  <id> <CMD> [args]\n
Arduino → PC:  <id> ACK\n            sofort nach gültigem Befehl
               <id> DONE\n           Aktion fertig (Zeit abgelaufen)
               <id> ERR <grund>\n
               READY\n               nach Boot
```
| Befehl | Wirkung |
|---|---|
| `OPEN <ms> <speed>` | Iris öffnen für ms, dann Motoren aus |
| `CLOSE <ms> <speed> <hold_speed>` | Iris schliessen für ms → DONE → weiter mit hold_speed (Halten) |
| `ROT <+1/-1> <ms> <speed>` | M4 drehen für ms → DONE → aus. Iris hält weiter |
| `STOP` | alle Motoren aus |
| `PING` | → `<id> DONE` (Lebenszeichen + Watchdog-Reset) |
| `STATE` | → `<id> DONE iris=<open/closing/holding/opening/off> rot=<on/off> shield_resets=<n>` |

- **Watchdog:** kein Befehl/PING für > 2 s → alle Motoren aus. Die Bridge sendet PING mit 2 Hz. (Achtung: beim Halten essentiell, sonst lässt die Iris die Mutter fallen, wenn ROS hängt – das ist gewollt sicherer Zustand.)
- Konstanten (Ports, Richtungen, Default-Zeiten) oben in der Firmware, Zeiten sonst immer vom PC → Tuning nur in YAML, nicht neu flashen.
- PC-Seite (19.10.): **direkt seriell aus `sa_tasks`** (`serial_gripper.py`, nur Standardbibliothek/termios, Protokoll rein in `gripper_protocol.py`), kein eigener Bridge-Node. Ein Thread sendet PING alle 0.5 s. Pick + Place in einem Prozess: `ros2 run sa_tasks full` (Alias `sa_full`), weil der Watchdog die Mutter 2 s nach Prozessende fallen lässt.
- ~~ROS2: `gripper_bridge`-Node als Action Server~~ (verworfen 19.10., siehe Entscheidungslog).

---

## 7. Software-Architektur

ROS2-Workspace unter `software/` (Clean Slate). Distro-agnostisch (Humble/Jazzy), nur `rclpy`, keine exotischen Abhängigkeiten.

```
software/
├── src/
│   ├── sa_interfaces/      ament_cmake: GripperCommand.action, Detection.msg
│   ├── sa_gripper/         serial_protocol.py (rein), gripper_bridge node
│   ├── sa_vision/          vision_node (Sechseck-Finder, Funktionen rein), geometry.py (rein),
│   │                       capture_images, apply_camera_controls; config/vision.yaml
│   └── sa_tasks/           state machines pick/place/full, robot_interface.py (CIC-Wrapper),
│                           launch/, config/{robot,vision,gripper,areas}.yaml
└── tests/  (pro Paket unter test/, pytest, ohne Hardware)
arduino/iris_gripper/       neue Firmware (sketch_sep24a bleibt als Motortest)
calibration/                Intrinsics (yaml), Verifikationsmessungen
data/                       Logs der Runs (csv/rosbag)
tools/                      Auswertung + Plots für den Bericht
```
Prinzipien:
- **Reine Logik ohne ROS** (Detektion, Geometrie, Protokoll, State-Machine-Übergänge) → Unit-Tests am Laptop.
- **Alle Zahlen in YAML**, keine Magic Numbers im Code.
- Hardware hinter Interfaces (`RobotInterface`, `GripperInterface`, `Camera`) → **Mock-Varianten** für Trockenlauf.
- State Machine: einfache explizite Python-Klasse (Enum + Übergangstabelle), kein SMACH/BT – weniger Overhead.
- Jeder Run schreibt automatisch eine CSV (Zeit, Phase, Soll-/Ist-Pose, Fz, Detektion) → direkt BA-vergleichbare Auswertung.
- Code, Kommentare und Logs **auf Englisch** (Bericht ist Englisch).

---

## 8. Zeitplan (Abgabe 31.10.)

| Woche | Fokus | Ohne Workstation? |
|---|---|---|
| **KW41** (07.–11.10.) | Firmware + Protokoll, Workspace-Gerüst, Geometrie + Tests, Detektion an Fotos, Mocks, Logging | ✅ |
| **KW42** (12.–18.10.) | Workstation: Build, Kamera + Intrinsics, Offset-Verifikation, **Phase A läuft** | ❌ |
| **KW43** (19.–25.10.) | **Phase B läuft**, Zeiten/Fz tunen, Evaluation-Runs (N ≥ 10) | ❌ |
| **KW44** (26.–31.10.) | Auswertung, Plots, Bericht fertig | ✅ |

Bericht parallel schreiben (Methodik-Kapitel kann schon in KW41/42 entstehen).

### Vorbereitung – Checkliste (KW41, mit Claude)
- [ ] `CLAUDE.md` + Workspace-Gerüst (Pakete, package.xml, setup.py, Launch, YAML)
- [ ] Firmware `arduino/iris_gripper` + Test am Board (ohne Roboter) – Code steht (19.10.), Test am Board offen
- [ ] `serial_protocol.py` + `gripper_bridge` + Test-CLI
- [ ] `geometry.py` (Pixel → Basis) + Unit-Tests mit synthetischen Daten
- [ ] `detection.py` + Fotos von Mutter/Schraube auf Weiss (Handy oder die Kamera am Laptop) → `data/images/`
- [ ] Intrinsics: BA-Werte suchen, sonst Kalibrierskript (Schachbrett) am Laptop
- [ ] State Machines + Mock-Robot/-Gripper → Trockenlauf des gesamten Ablaufs
- [ ] Logging + Auswerteskript (BA-Metriken)

---

## 9. Evaluation
- **Erfolgsquote** (Hauptmetrik): N ≥ 10 Runs mit verschiedenen Positionen in Area N / Area S. Erfolg je Phase separat protokollieren (Detektion / Greifen / Aufsetzen / Schrauben).
- **BA-Vergleich:** gleiche Tracking-Metriken je Phase + Zykluszeit.
- **Neu:** Positionsfehler der Zielberechnung relativ zum Objekt (Objekt an bekannter Position, Abweichung Ziel ↔ Wahrheit).

---

## 10. Offene Fragen (Runde 2)

> Hinter `Antwort:` schreiben. Kurz reicht.

**F21.** Kamera-Offset: relativ zu welchem Punkt und in welchem Frame?
Antwort (08.10.): Flansch-Mitte → Kamera-Linsenmitte (unterster Part): Δx = 0, Δy = 44.4 mm, Δz = 60.223 mm. ✅
Update (16.10., neue Halterung): Bezugspunkt = unterster Punkt des **Kameragehäuses** (dort lag auch das Lineal am Bench), Linse 5 mm höher. Δx = 0, Δy = 44.4 mm, **Δz = 57.223 mm**. ✅

**F27.** Achsen / Frames.
Antwort (08.10., bestätigt): Blick von der Roboterbasis aus, Gripper schaut senkrecht nach unten:
- Basis: z_robo nach oben, x_robo nach vorne, y_robo nach links.
- Kamera (OpenCV): z_cam nach unten (ins Bild), x_cam nach rechts (+u), y_cam nach hinten (+v).
- ⇒ **z_robo = −z_cam, x_robo = −y_cam, y_robo = −x_cam**, d.h. Δx_robo = −Δv, Δy_robo = −Δu (gleich wie BA „Axis Mapping“).
- EE-Frame (BA-Bild): z nach unten, x wie Basis-x, y = −Basis-y (180° um x). Damit R_EE_cam = Rz(+90°).

**F28.** Kameraseite.
Antwort (08.10.): Kamera zeigt **immer nach vorne (+x_robo)**, d.h. EE-Gierwinkel fix so, dass die Kamera vor dem Flansch sitzt. ✅
~~⇒ EE-Orientierung: x_EE = +y_robo, y_EE = +x_robo, …, R_EE_cam = Rz(180°)~~ (falsch, korrigiert 18.10.). **Richtig:** EE-Quaternion (1, 0, 0, 0), x_EE = +x_robo, y_EE = −y_robo, z_EE = −z_robo; die Kamera sitzt in dieser Pose vorne, weil der Gripper entsprechend auf den Flansch geschraubt ist. R_EE_cam = Rz(+90°) (wie F27). Im Bild liegt der Gripper/TCP 44.4 mm „hinten“ = unterhalb der Bildmitte.

**F29.** Bestätigen: Wenn die Kamera vorne sitzt, ist der Gripper (bzw. die Halterung) im Kamerabild **unten**? (Damit gilt F27 genau in dieser Konfiguration.)
Antwort: 

**F30.** Sitzt die Kamera auf dem **drehenden** Teil (M4 dreht sie mit) oder auf dem fixen Teil am Flansch? Falls sie mitdreht: Orientierung nach dem Schrauben unbekannt → vor jedem Bild zurückdrehen nötig.
Antwort: 

~~**F22.** Stiftschraube: Ist die Gesamthöhe 20 mm Basis + 19 mm Gewinde = 39 mm, oder 20 mm insgesamt? Schaut das Gewinde oben aus der Basis heraus?~~ → **39 mm insgesamt** (Yannick 20.10.)
Antwort: 

**F23.** CIC-Schnittstelle: Wie hast du in der BA Soll-Pose und Steifigkeit gesetzt (Topic-Name, Msg-Typ, z.B. `PoseStamped` auf `/cartesian_impedance/target`)? Woher kommt die Ist-Pose bzw. Fz (`franka_robot_state`)? Ein Code-Snippet aus der BA reicht.
Antwort: 

**F24.** Ab wann genau hast du Zugriff auf die Workstation?
Antwort: 

**F25.** Gibt es BA-Kalibrierwerte der Kamera (fx, fy, cx, cy, Distortion)? Falls ja: hier einfügen oder als Datei in `calibration/` ablegen.
Antwort: 

**F26.** Wie gross sind Area N und Area S ungefähr (mm), und wie weit liegen sie auseinander? Von welcher Höhe aus siehst du die ganze Area?
Antwort: 

---

## 11. Entscheidungslog
| Datum | Entscheidung | Begründung |
|---|---|---|
| 25.10. | **Kamera-Rotation im Flansch kalibrieren, nur mit Translationen:** `ros2 run sa_tasks hand_eye` (ChArUco 9×13, 20 mm, `data/grid/charuco_9x13_20mm.pdf`; 27 Posen um `observe_stud`), Ergebnis als `camera_tilt_in_flange_deg` + Drehung um z, wird nur angezeigt | Yannick will neu kalibrieren. Volle Hand-Eye braucht Drehungen (verletzt „nur (1, 0, 0, 0)“) → Variante A: Rotation aus Translationen, Position bleibt CAD. y_F-Anteil der Neigung war nie gemessen; 1° ≈ 7 mm bei d = 400 mm |
| 25.10. | **Die eine Korrektur linear in y:** `frames.target_correction_mm` (konstant) → `frames.target_correction` (Stützpunkte y −300 → [0, −3], y −130 → [0, −1], linear dazwischen, aussen konstant; gilt für Mutter und Schraube) | Yannick, mehrere Läufe: konstante −2 ganz links zu viel, ganz rechts zu wenig. Weiterhin nur eine Korrektur |
| 25.10. | **`target_correction_mm` [0, 0] → [0, −2.0]** (gilt für Mutter und Schraube) | Mit fester Anfahrrichtung und Feinpositionierung ohne Integrator: Platte auf beiden Seiten zu weit in +y, links 1–2 mm, rechts 2–3 mm; TCP erreicht das berechnete Ziel auf ≤ 0.4 mm → Mittelwert. Rest ±0.5 mm zwischen den Seiten; offen: zweite Komponente der Kamera-Neigung nie gemessen |
| 25.10. | ~~Schrauben-Korrektur abhängig von y (`place.stud_correction_by_y`)~~ **wieder entfernt.** Einzige Korrektur bleibt `target_correction_mm` (= [0, 0]) | Yannick: keine zwei fest eingetragenen Korrekturen; Fokus auf die Ursache (Integrator-Verlust beim Abstieg) |
| 25.10. | **Feinpositionierung ohne Integrator:** beim Warten wird die Referenz alle 50 ms neu gesendet (`wait_until_arrived(hold=…)`), der Integrator bleibt so 0 wie beim Abstieg | Yannick: nach der Feinpositionierung gab der Arm vor dem Aufsetzen in y wieder nach, „als ob die Spannung weg wäre“; dadurch Mutter nicht konzentrisch → verkippt → dreht schwer. Ursache: jede Referenz-Nachricht setzt den Integrator zurück (bis 2.5 N ≈ 1 mm bei 2500 N/m). Lauf mit 8/60 mm: Kraft skaliert (−7.6 N), z-Weg aber gleich 1.99 mm |
| 25.10. | **Vorzeichen der Kraft im Record geklärt: Andrücken auf die Schraube = negativ.** Festziehen `depth_mm` 30 → 60 als Test, ob die Kraft mit der Tiefe skaliert | Von Hand auf den Greifer gedrückt: Fz positiv. Lauf mit 8/30 mm: −1.0/−0.3/−1.0 N, also nur ≈ 1 N statt ≈ 4.5 N, aber z-Weg 2.03 mm (≈ 1.6 Umdrehungen, bisher bester). Die positiven Werte vom 23.10. waren Offset, kein Anpressen |
| 25.10. | ~~Pendelnde Bodenplatte (`plate_tilt_gain`)~~ verworfen → **Schrauben-Korrektur abhängig von y** (`place.stud_correction_by_y`, linear zwischen Stützpunkten): Start y −294 → [0, −2.5], y −149 → [0, 0] | Kipp-Verschiebung wechselte zwischen Läufen an derselben Stelle (rechts +0.26 / −1.40, links −1.56 / +0.05 mm), hängt also nicht von der Lage ab. Yannick: Fehler hängt davon ab, wie weit links die Schraube liegt → empirisch korrigieren |
| 25.10. | **Anfahrt an die Schraube immer aus derselben Richtung:** zuerst 10 mm daneben in −y (`place.approach_from_mm` [0, −10]), dann der letzte kurze Zug in +y (Yannick: Platte hängt auf der −y-Seite tiefer) + Feinpositionierung. `target_correction_mm` zurück auf [0, 0], neu einstellen für diese Richtung | Yannick: Schraube links im Bild → Platte zu weit rechts, rechts im Bild → zu weit links. Records: TCP erreicht das berechnete Ziel auf ≤ 0.5 mm, Schraube nach dem Zentrieren 2–5 mm von der Achse → weder Kamera noch Kinematik. Vermutung: Spiel der Drehscheibe des Unterteils (Platte bleibt in Fahrtrichtung versetzt, sichtbar leicht schief) |
| 25.10. | **Neuer Greifer (1:1-Antrieb, gleiche Geometrie): `target_correction_mm` [0, 0] → [0, −1.5] → [0, −2.5] (−1.5 reichte nicht, Yannick: noch 1 mm); Anpressen erhöht:** Fangen `depth_mm` 3 → 8, Festziehen 15 → 30 | Yannick: Bodenplatte sitzt konstant ≈ 1.5 mm zu weit in +y; kaum Druck in z, die Mutter kann das Gewinde so nicht fassen. Record 11:08: Kraft 0.5–1.7 N, am Ende −0.7 N, z-Weg nur 0.33 mm (≈ ¼ Umdrehung) in 5 s bei 255 |
| 25.10. | **Eigenes Tempo-Limit für z:** `robot.motion.max_speed_z_mm_s` = 150, xy bleibt 250. Neues Werkzeug `ros2 run sa_tasks motion_test` (Fahrten ohne Greifer/Vision, Zittern pro Fahrt als RMS) | motion_test: bei 250 mm/s z-Fahrten 0.2–0.9 mm RMS, xy 0.01–0.07; bei 150 z ≈ 0.01 mm, aber xy bleibt hängen und springt (Haftreibung). Vermutung: in z Gewicht + Dämpfung über dem Kraft-Clamp 18 N. Später möglich: Dämpfung auf den Geschwindigkeitsfehler im Regler, dann z wieder schnell |
| 25.10. | **Echtzeit-Priorität für den User:** Gruppe `realtime` mit `rtprio 99` und `memlock unlimited` (`/etc/security/limits.d/99-realtime.conf`), danach neu anmelden. Regler läuft jetzt mit FIFO-Priorität 98 | Log: „Successful set up FIFO RT scheduling policy“, keine „Failed to lock“ mehr (vorher 8–186 pro Lauf). `control_command_success_rate` nur noch einzelne 0.99 statt bis 0.95. Yannick: Zittern sichtbar besser, xy-Fahrten gut, bei z-Bewegungen noch etwas. Kein RT-Kernel; nächste Option `preempt=full` |
| 25.10. | **Regler entlastet:** Debug-Topic aus (`debug.publish_rate` 1000 → 0), keine Konsolenausgabe pro Sollpose mehr. **Ursache des Zitterns: verlorene Regelzyklen** → Echtzeit-Priorität für den User als Nächstes | Zittern bei grossen Fahrten und beim Abstieg zur Mutter, mit 150 statt 250 mm/s unverändert (Kraft-Clamp also nicht die Ursache). Log: kein RT-Kernel, „Could not enable FIFO RT scheduling“ (auch am 23.10.), `control_command_success_rate` bei Fahrten meist 0.99, bei grossen Bewegungen 0.95 und tiefer. Tipp von Claude Web (10 Hz / 10-mm-Sprünge, Sollpose filtern) trifft nicht zu: 100 Hz, Filter pro 1-ms-Zyklus ist schon drin (α = 0.002) |
| 23.10. | **Fazit des Tages: Schrauben scheitert am Antrieb, nicht an der Positionierung → CAD neu mit 1:1-Übersetzung am M4-Antrieb.** Software bleibt so (screw/tighten-Modi, Fangen mit Wackelkreis 2 mm, Festziehen 5 s bei 255, Abfangen beim Loslassen; Positionierung wie 22.10.) | 20 `full`-Läufe, alle mit Kontakt; Präzision praktisch perfekt, Mutter greift, Schraube kippt nicht, Teile kaum unkonzentrisch (Yannick), trotzdem max. ≈ 1.5 Umdrehungen (z-Weg 0–1.9 mm). Heutige Übersetzung 44 → 17 gibt nur 0.39× Motormoment. Der Handtest vom 22.10. täuschte: eine von Hand gehaltene Schraube richtet sich selbst aus |
| 23.10. | **Controller-Modus `tighten` (control_mode 5) fürs Festziehen:** wie `screw`, aber xy 400 N/m, Integrator aus (`modes.tighten_stiffness`). Fangen bleibt bei xy 2000 (Wackelkreis 2 mm) | Yannick: Mutter perfekt mittig, klemmte trotzdem; nicht perfekt konzentrisch → beim Drehen muss sie seitlich schwingen können |
| 23.10. | **Positionierung zurück auf den Stand vom 22.10.:** Kamera-Neigung 1.25°, `target_correction_mm` [0, 0] | Yannick: gestern war es eigentlich perfekt; 1.33° bzw. −1 mm haben nicht geholfen |
| 23.10. | **Nachgreifen + Anschrauben gestrichen; `target_correction_mm` = [0, −1]** (gilt für Mutter und Schraube; Kamera-Neigung bleibt 1.33°). Schrauben wieder: Fangen mit Wackeln → mittig festziehen | Yannick: Mutter hielt mit offener Iris nicht; die Mutter sitzt ≈ 1 mm zu weit in +y, mit −1 mm wäre sie zentriert → vorher genauer sein statt nachträglich korrigieren |
| 23.10. | **Anschrauben vor dem Nachgreifen** (`place.screw.engage`: 1000 ms bei 255, 15 mm, an der Kontakt-xy) | Lauf 10:56: nach den langsamen Fang-Schritten nur 0.2 mm eingesunken, mit offener Iris hielt die Mutter nicht. Die schnelle Drehung brachte bisher 1.0–1.9 mm |
| 23.10. | **Nachgreifen nach dem Gewindefangen:** steif machen ohne Absacken (`stiffen_where_it_is`, wie beim Loslassen), Iris auf, frei auf gleicher Höhe zurück über den Kontaktpunkt (Feinpositionierung), Iris zu, dann festziehen. Ersetzt das Zurückfahren mit gegriffener Mutter (`recentre_ms`, `recentre_offset_mm` entfernt) | Record 10:39: mit −1 mm Versatz kam der TCP nur auf −0.36 mm (Mutter auf der Schraube blockiert den Arm), −2 mm half auch nicht. Idee Yannick: kurz loslassen, damit der Arm frei ist |
| 23.10. | **Versuch: nach dem Gewindefangen auf Kontakt + (0, −1) mm statt auf den Kontakt** (`catch.recentre_offset_mm`, gilt auch fürs Festziehen) | Wunsch Yannick, ausprobieren |
| 23.10. | **screw-Modus xy steif (400 → 2000 N/m), Wackel-Radius 4 → 1 mm;** z und Kippen bleiben weich | Records 10:27/10:29: Flansch bewegte sich beim Wackeln nur 0.2–0.5 mm, Zurückfahren nicht sichtbar, Reibung bestimmte die Lage. Der Unterteil des Grippers ist beweglich (Yannick) → er liefert die Nachgiebigkeit, der Arm soll genau dorthin fahren. Neigung 1.5° beim Schrauben bleibt offen (Korrektur der Orientierung vorerst nicht, Yannick: xy sollte reichen) |
| 23.10. | **Nach dem Gewindefangen zurück auf die Kontakt-xy** und dabei 1.5 s langsam weiterdrehen (`catch.recentre_ms`), erst dann festziehen. Record: xy des TCP pro Fang-Schritt und nach dem Zurückfahren. Visuelle Prüfung der drehenden Bodenplatte als nächster Schritt offen | Yannick: Mutter greift jetzt, aber das Wackeln kippt die Schraube minim → schräg aufgeschraubt. Beim Schrauben verdeckt die Bodenplatte das ganze Kamerabild (Yannick) → Drehung visuell prüfbar. Record 10:21: TCP folgte dem 4-mm-Kreis nicht, blieb ≈ 0.3 mm daneben stehen |
| 23.10. | **Kamera-Neigung 1.25° → 1.33° um x_F** (= +0.5 mm in +y_B bei d = 393 mm) | Wunsch Yannick: Zielpunkte von Mutter und Schraube 0.5 mm weiter in +y |
| 23.10. | **Loslassen: 2.5 s warten vor dem steifen Modus** (`place.release_settle_s`), Wackel-Radius 2 → 4 mm | Record 10:02: beim Schrauben z-Weg 0.00 mm, Neigung konstant 1.12°, Kraft 0.15–0.5 N (Gewicht offenbar kompensiert) → nichts bewegt. Der Controller glättet die Referenz (τ ≈ 0.5 s): pro 250-ms-Schritt nur ≈ 40 % des Kreissprungs, und beim Umschalten auf steif lag die interne Referenz noch 15 mm unten → Absacken |
| 23.10. | **Wackeln + Abfangen beim Schrauben:** (1) beim Gewindefangen fährt die Referenz pro Schritt einen Punkt weiter auf einem Kreis r = 2 mm um den Kontakt (4 Punkte/Kreis, `catch.wobble`), screw-Steifigkeit xy 30 → 400 N/m (≈ 0.8 N seitlich); Festziehen wieder zentriert. (2) Vor dem Öffnen der Iris: Referenz = gemessene Pose + 3 mm (`place.release_lift_mm`), steifer Modus, erst dann öffnen. (3) Record-Werte als float (numpy-Zahlen liessen `yaml.safe_dump` scheitern → Prozess-Abbruch exit −6, kein Record) | Yannick: auf der Schraube bewegte sich in xy nichts (bei 30 N/m reicht der Zug nicht gegen die Reibung), und der Arm fiel nach dem Loslassen weit nach unten (09:39 starb dabei auch ros2_control_node) |
| 23.10. | **Weniger Anpressen beim Schrauben:** catch 10 → 3 mm, tighten 50 → 15 mm unter Kontakt. Record speichert jetzt die gemessene Kontaktkraft (F_z minus Wert in der Luft vor dem Abstieg) am Start, nach dem Fassen und am Ende | Erster Lauf im screw-Modus drückte sichtbar zu stark, Gewinde nicht gefunden. Ohne z-Integrator kommt das nicht kompensierte Werkzeuggewicht noch dazu, deshalb messen statt schätzen |
| 23.10. | **Eigener Controller-Modus `screw` (control_mode 4) + Gewinde-Erkennung:** beim Schrauben xy 30 N/m, z 150 N/m, Kippen 4 Nm/rad, Integrator aus (`modes.screw_stiffness` in `controllers.yaml`). Langsam in 250-ms-Schritten (Speed 100, 10 mm unter Kontakt) drehen, bis der TCP ≥ 0.6 mm gesunken ist (= Gewinde gefasst), dann 3000 ms bei Speed 200 mit 50 mm. Nach 12 Schritten ohne Fassen trotzdem festziehen (`caught: false`). Record: z-Weg pro Schritt, Neigung Start/nach Fassen/Ende. Ersetzt `place.screw.phases` durch `catch` + `tighten` | Mutter steckte mit festen Phasen schräg fest, nicht wegen zu wenig Moment. Im float-Modus hielten Kippsteifigkeit (40 Nm/rad, ≈ 0.7 Nm für 2°) und Integrator (bis 1.5 Nm) die Mutter schief; das Gewinde schafft nur ≈ 0.05 Nm. z-Weg statt Kraft, weil die Anpresskraft gleich bleibt, ob gefasst oder nicht. `gripper.stop()` lässt auch die Iris los → kurze Schritte. Risiko: Gripper-Last nicht im Desk → Arm kippt von selbst. Rückwärtsdrehen/Mini-Spirale erst, falls es weiter hängt |
| 23.10. | **Pick ohne Halt über der Mutter:** die Fahrt auf 10 mm über die Mutter geht direkt in die Spirale über, ohne Ankunft abzuwarten und ohne Feinpositionierung (diese bleibt beim Anfahren der Schraube) | Wunsch Yannick: der Gripper schwebte sichtbar über der Mutter und wartete. Der Spiralradius (6 mm) deckt den Haltefehler von 1.5–4.5 mm aus dem Reach-Test ab |
| 23.10. | **Schrauben in Phasen (`place.screw.phases`):** erst 2000 ms bei Speed 100 mit Referenz 10 mm unter Kontakt (≈ 1.5 N, Gewinde fangen), dann 3000 ms bei Speed 200 mit 50 mm (≈ 7.5 N, festziehen). Ersetzt `duration_ms`/`speed`/`follow_depth_mm`/`final_press`. `screw_test` (Enter) fährt dieselben Phasen; neues Werkzeug `ros2 run sa_tasks goto_poses` (fährt `observe_nut`, dann `observe_stud`, zum Markieren auf dem Tisch) | 22.10.: Mutter am Roboter jedes Mal nur aufgelegt, von Hand schraubt sie leicht auf → Vermutung: bei 255 springt sie über den Gewindeanfang bzw. wird zu früh flach angedrückt. Werte von Yannick |
| 22.10. | **Schraubtest ohne Roboter (`ros2 run sa_tasks screw_test`):** Iris zu → 5 s (`screw_test.wait_s`) → M4 mit `place.screw` → Iris auf; Dauer/Speed pro Lauf eingebbar. Ergebnis: Mutter von Hand an die Schraube gehalten schraubt sich leicht auf, **sogar mit dem N20 1:100** → Moment ist nicht das Problem. Nächster Ansatz: langsam anfangen (Gewinde fangen), dann schneller und erst dann nach −z drücken | Wunsch Yannick: Schrauben isoliert ausprobieren. Beim Roboter steht die Schraube fest und der Arm muss nachgeben, von Hand zentriert sie sich selbst |
| 22.10. | **Anpressen beim letzten Drehen:** die letzten 1500 ms der Drehung mit Referenz 50 mm unter Kontakt (float, ≈ 7.5 N), Drehung in zwei ROT-Befehle geteilt (`place.screw.final_press`). Drehung jetzt 4000 ms bei Speed 255 | Wunsch Yannick: Mutter am Ende wirklich auf die Schraube drücken |
| 22.10. | **Kamera-Neigung im Modell (`frames.camera_tilt_in_flange_deg`) statt Ziel-Offset:** 1.25° um x_F, um y_F noch 0. `target_correction_mm` bleibt [0, 0] | Hand-Messung 22.10.: Kamera über der Schraube zentriert, Iris von Hand zentriert (nur y geprüft), Schraube 8.6 mm weiter in +y als berechnet bei d = 393 mm. Bei zentrierter Kamera können Höhe (Gehäuse-Unterseite, 9 mm) und Intrinsik das nicht erklären (Höhe: 0.06 mm), nur die Achsrichtung. Halterung ist starr → konstant → gehört in die Kamera-Pose, gilt dann für Pick und Place und jede Höhe |
| 22.10. | **Feinpositionierung statt konstanter Zielkorrektur:** vor der Pick-Spirale und beim Anfahren der Schraube wird die Referenz um den verbleibenden xy-Fehler verschoben, bis < 0.5 mm (max. 3 Schritte, Verstärkung 0.8, `robot.fine_positioning`; korrigiert wird der TCP aus der gemessenen Flanschpose inkl. Neigung, nicht der Flansch: Flansch 0.1 mm genau, TCP trotzdem 2.7 mm daneben). `target_correction_mm` auf [0, 0] zurück, Rest neu messen | Reach-Test 22.10. (`ros2 run sa_tasks reach_test`): der Arm bleibt 1.5–4.5 mm vor dem Ziel stehen, immer entgegen der letzten Fahrtrichtung (Haftreibung gegen K = 2.5 N/mm, Integrator nur 2.5 N), alle Gelenke ≥ 14° von den Grenzen → kein Reichweitenproblem. In den Läufen kam die letzte Bewegung aus der Tischmitte → Fehler Richtung y = 0, Vorzeichen wechselt mit der Tischseite; deshalb passte +5 nur auf einer Seite (nach dem Tausch der Beobachtungsposen fehlgeschlagen). Wunsch Yannick: der Arm soll dorthin fahren, wo er soll, kein Offset |
| 22.10. | **Firmware prüft das Motor Shield vor jedem Bewegungsbefehl:** schläft der PWM-Chip (PCA9685, MODE1-Sleep-Bit), wird er neu initialisiert (`shield_resets` in `STATE`), antwortet er nicht → `ERR shield`. Eine haltende Iris wird danach neu gesetzt | 22.10. mehrmals `DONE`, aber kein Motor bewegte sich, bis Board + Netzteil ab- und angesteckt wurden: Spannungseinbruch (blockierender M4) setzt den Shield-Chip zurück, das Arduino läuft weiter |
| 22.10. | **Beobachtungsposen getauscht:** Mutter jetzt links (`observe_nut` (340, +222)), Schraube rechts (`observe_stud` (410, −233)); gemeinsame Höhe weiter 473.4 mm (`observation_height_from: observe_stud`). Neues Werkzeug `ros2 run sa_tasks reach_test` (Raster um zwei Punkte: Restfehler, F_ext, Gelenkreserve, Feinpositionierung) | Wunsch Yannick; der Tausch hat gezeigt, dass die konstante Zielkorrektur nur auf einer Tischseite stimmte |
| 20.10. | **Code aufgeräumt (vor dem Neudruck):** entfernt: Gripper-Neigung (`use_gripper_tilt`, `flat`, `ToolRobot`), Verifikation (`truth`, `compare_runs`), `--locate-only` (pick + place; `--approach-only` mit Check deckt es ab), Records `tcp_move`/`orientation`, Tilt-Prüfung geteachter Posen, `capture_images`, `sa_vision/geometry.py` (Strahl-Ebene jetzt in `frames`), unbenutzte `Detection`-Felder, `d_source`, Overlay-Modus, STOP/STATE auf PC-Seite, Backends in tasks.yaml (`--robot mock` bleibt). Vision-Parameter werden aus vision.yaml automatisch deklariert. Tests 76 → 21. Bleibt: CAD-Offset-Prüfung, Warten auf Stillstand | Wunsch Yannick: so wenig und kompakter Code wie möglich. `flat` hilft wegen des Spiels im drehenden Teil nicht |
| 20.10. | **`target_correction_mm` = (−1, +1) mm, dann (−1.5, +2.5), (−1.5, +3.5), dann (−1.5, +4.0) mm** (Mutter und Schraube; Werte 2–4 von Auge an der Schraube, Yannick) | 3 Approach-Läufe: zentrierte Position ±0.2 mm, Kamera schräg vs. zentriert 1.5 mm, Roboter immer −1.0 mm in y und +3.5 mm in z (kein z-Integrator); Iris von Hand gemessen ≈ 1 mm in −x und +y zu verschieben → konstanter Versatz Kamera → TCP (Montage/CAD). Neigung des Grippers (`flat`) erst messen, wenn Positionen anderswo auf dem Tisch abweichen |
| 20.10. | **Positions-Check im Approach-Modus:** Log + Record (`check`) mit Stud-xy 1) von der Beobachtungspose (schräg, gesperrt), 2) zentriert (Ziel, ohne Korrektur), 3) erreichter TCP; Differenzen 2−1 (Kameramodell) und 3−2 (Roboter). Danach Bodenplatte gegen Schraubenachse von Hand messen | Approach 09:24 lag einige mm in +y daneben; Roboter erreichte sogar 1.2 mm mehr −y als befohlen → Ursache vermutlich Kamera/Montage, nicht Roboter. Plan Yannick: Position in drei Stufen vergleichen |
| 20.10. | **Fleck-Grenze Stud 8 → 6 mm** zurück | Lauf 09:23: Schraube an der Beobachtungspose nur 8.06 mm (sonst 10.2–10.5), nach dem Zentrieren unter 8 mm → herausgefiltert → „kein Stud“. Flecken fangen jetzt Tracking/Lock und „grösstes Objekt nahe der Achse“ ab. Erster Approach-Lauf 09:24 ok: Stud 3.6 mm von der Achse, TCP xy −1.2 mm daneben, z 3.4 mm zu hoch (8.4 statt 5 mm über der Spitze, Regelabweichung) |
| 20.10. | **`place_screw --approach-only`** (`task.launch.py … approach_only:=true`): Stud-Suche, dann TCP über die Schraube auf Spitze + 5 mm (Tisch + 44 mm), erst xy, dann runter; hält dort, kein Kontakt, kein Schrauben, kein Gripper. Log/Record: erreichte TCP-Position und Abweichung zur Schraube | Wunsch Yannick: Zentrierung beim Runterfahren von Auge prüfen, bevor Kontakt/Schrauben getestet wird |
| 20.10. | **Lock nur für die Wahl an der Beobachtungspose** (Parameter `lock` pro Anfrage von `sa_tasks`); nach dem Zentrieren und für die Mutter wieder Median über die letzten 5 Bilder ohne Bestätigung/Lock | Wunsch Yannick: das Verhalten nach dem Zentrieren hatte vorher gut funktioniert |
| 20.10. | **Lock nach Tracking:** nach 10 verfolgten Bildern werden die Mediane der bestätigten Tracks fest gesperrt bis zur nächsten Anfrage (`tracking.lock_after_frames`, `Detection.locked`; Overlay „LOCKED“ auf dem Live-Bild); `sa_tasks` nimmt nur gesperrte Detektionen. Zuordnungsradius 15 → 40 px | Mit Tracking allein schlechter: 2 von 3 Läufen „0 Objekte“, 1× nach dem Zentrieren verloren – der Mittelpunkt der Schraube sprang weiter als 15 px, der Track riss ab. Wunsch Yannick: wieder sperren, aber richtig |
| 20.10. | **Tracking statt Einfrieren:** Objekte werden über die Bilder verfolgt (`tracking.*` in vision.yaml: Zuordnung ≤ 15 px, Median über die letzten 5 Bilder, in denen das Objekt gefunden wurde; erst dann zählt es; Track endet nach 3 Fehlbildern). Jede Anfrage von `sa_tasks` (nach `settle_s`) startet die Tracks neu. Einfrieren des Overlays (`freeze_stamp_ns`) wieder entfernt, Live-Bild läuft weiter | Präzisierung Yannick: „festhalten“ = gemittelte Position über die letzten 5 Funde, kein Standbild. Mit dem Einfrieren rastete die Suche in 2 von 3 Läufen auf Flecken ein (ein einzelnes Bild entschied) |
| 20.10. | **Stud-Suche fester Ablauf:** nach jeder Bewegung das grösste Objekt ≤ 20 mm von der Achse (statt das nächste), Zentrier-Toleranz 4 → 10 mm, max. 2 Züge; Overlay wird auf dem Bild der Entscheidung eingefroren (`freeze_stamp_ns`, „LOCKED“), bis die nächste Detektion angefordert wird | Lauf funktionierte, dauerte aber lang: Zittern des Arms + Regelabweichung 2–3 mm lösten Zusatz-Züge aus; dunkle Schattenflecken auf der Schraube selbst wurden als nächstes Objekt genommen. Wunsch Yannick: Beobachten → festhalten → zentrieren → festhalten |
| 20.10. | **Task-Launches ohne Start-Bewegung des Controllers** (`cartesian_impedance_controller.launch.py start_pose:=false` lädt `start_pose_off.yaml` per Spawner `--param-file`; `sa_robot` behält sie). **Kamera zurück auf 30 fps**, 10 fps wird separat getestet; Median über 5 Bilder bleibt | Wunsch Yannick: `sa_place_locate` soll direkt zur Beobachtungspose (bzw. dort bleiben), nicht erst zur Startpose. Das Warten auf Stillstand (`start_settle_s`) bleibt als Schutz |
| 20.10. | **Kamera 30 → 10 fps (zurückgenommen, siehe oben) + Median über 5 Bilder** (pro Objekt Mittelpunkt, Grösse, Breitenverhältnis; Zuordnung über nächsten Mittelpunkt ≤ 15 px). `Detection.window_start`/`frames_averaged`/`frames_window`: `sa_tasks` nimmt nur Detektionen, deren ganzes Fenster nach der Beruhigungszeit liegt | Overlay der Schraube zitterte trotz Fleck-Grenze 8 mm noch; Kamera wird heiss. Median statt Mittelwert gegen einzelne Ausreisser. Kostet 0.5 s pro Detektion |
| 20.10. | **Stud-Kandidaten nach Fläche ranken** (grösste zuerst) statt Quadrat-zuerst/Bildmitte; Fleck-Grenze 6 → 8 mm. Ziel (`target`) wird zu Beginn jedes Schritts gesetzt (`vision.prepare`) | Overlay sprang zwischen Bildern: ein zufällig quadratischer Fleck (~6 mm, w 0.5) kam auf Rang 1; Sechseck bis zum ersten Stud-Bild, weil `target` erst dann gesetzt wurde. Mittelung nicht nötig: das Problem war die Reihenfolge, nicht das Rauschen |
| 20.10. | **Jeder Schritt wartet vor der ersten Bewegung, bis der Arm steht** (`robot.start_settle_s` = 1 s, in `check_controller`) | `place --locate-only` (Mock-Gripper, keine Verzögerung) schickte die ersten Referenzen 0.7–1.7 s nach der Aktivierung, mitten in die Start-Bewegung des Controllers (5.7 s zu [0.5, 0, 0.5]); die sollte durch neue Referenzen abbrechen, tat es aber nicht immer → Arm fuhr zur Startpose statt zu `observe_stud`. Pick schickt wegen Serial-Connect + Öffnen erst Sekunden später |
| 20.10. | **Stud: nur dunkle Pixel segmentieren** (`targets.stud.segmentation: dark`, I/B < 0.6; Mutter bleibt `band`). Overlay Stud = Quadrat + Kreis 8 mm (Gewinde), Labels mit Grösse und Breitenverhältnis | Zentrierter Lauf: 3.4 mm von der Achse, aber 16.0 mm / w 0.61 → abgelehnt (> 14 mm); Schatten unten links blähte die Hülle auf und drehte das Quadrat weg von der scharfen Ecke |
| 20.10. | **Ranking der Stud-Kandidaten geändert:** grober Vorfilter (6–40 mm, Breitenverhältnis ≥ 0.4, `targets.stud.search_*`), dann Quadrat zuerst, dann **Abstand zur Bildmitte** statt Grössenfehler. Fehlermeldung „goal not reached“ nennt jetzt den Robot-Mode | Erster Lauf: Rang 1 war ein Strich am Bildrand (w 0.30) bzw. ein 4-mm-Fleck, die Schraube (schräg 17.7 mm, w 0.56–0.60) erst Rang 2. Arm fuhr beim Zentrieren gar nicht (Pfad korrekt gesendet, Geschwindigkeit 0) → vermutlich Roboter-Seite (User-Stop/Reflex), unklar |
| 20.10. | **Stiftschraube per Zentrieren statt aus einem Bild:** von `observe_stud` alle Objekte erfassen und ranken (`Detection.candidates`, neue Msg `Candidate`: Form/Grösse im Stud-Band zuerst, dann Grössenfehler; im Overlay nummeriert). Nacheinander Kamera-Achse über das Objekt fahren (Korrektur aus der gemessenen Pose, max. 3 Züge, zentriert ≤ 4 mm), dort Form prüfen; erstes Quadrat ≈ 10 mm = Schraube, Position aus dem zentrierten Bild. Max. 3 Objekte, verloren > 20 mm (`place.stud_search`) | Test 20.10.: schräg gesehen ergibt die 39 mm hohe Schraube keine Quadrat-Kontur (Form je nach Winkel) → Vorschlag Yannick: zentrieren und von oben prüfen, Ranking als Sicherheit bei mehreren Objekten (im Versuch nur die Schraube im Bild). Nebeneffekt: Parallaxe von oben ≈ 0.2 mm |
| 20.10. | **Schritt 2 begonnen, Vision für die Stiftschraube:** `vision_node` wertet `target` jetzt aus: `nut` → Sechseck, `stud` → engstes umschliessendes Quadrat (gleiche Methode, 2 statt 3 Linienpaare); Form-/Grössenfilter pro Ziel (`targets.nut`, `targets.stud`: Breitenverhältnis 0.60–0.82, 10 mm, noch geschätzt). Mittelpunkt auf der Ebene der Basis-Oberkante (20 mm, `centre_height_mm`): Wände der Basis und Gewindespitze verschieben sich off-axis gegenläufig, Fehler ≈ 0.3 mm bei 100 mm (THEORY 2.0). F22 beantwortet: **Gesamthöhe 39 mm**. Neu: `place_screw --locate-only` und `task.launch.py step:=… [locate_only:=true]` (`pick.launch.py` = step pick) | Plan Yannick: Schritt 2 zuerst separat testen (Vision, dann Aufsetzen/Schrauben), dann alles zusammen. Schraube steht fix und senkrecht auf dem Tisch |
| 20.10. | **Vorläufig: Watchdog öffnet die Iris** (`WATCHDOG_OPENS_IRIS = true`): war die Iris zu/haltend, schaltet der Watchdog alles aus und öffnet sie danach einmal (1500 ms, Speed 100 = `gripper.open`). Später wieder auf `false` (nur aus) | Wunsch Yannick: nach `sa_pick_all` die Iris nicht jedes Mal von Hand öffnen. Mutter fällt ohnehin 2 s nach Prozessende |
| 21.10. | **Neue Halterung gedruckt, `target_correction_mm` = [0, 0]** (vorher [−1.5, 4.0] für die schiefe Halterung). Kamera-Check mit Ringblatt (`data/grid/kamera_ausrichtung.pdf`, Bilder `data/images/rotation_20261021-1111/`): Hauptpunkt trifft 0.1–0.6 mm innerhalb des CAD-Radius 44.5 mm, über ±30° Drehung der oberen Hälfte nur 0.5 mm Streuung (= Wiederholbarkeit von Hand) → ≤ 0.5° Neigung, Konstruktion steif genug. Danach `sa_pick` normal (mit Spirale) zur Erfolgsquote | Korrektur gehörte zur alten Halterung; neu messen statt übernehmen |
| 21.10. | **`target_correction_mm` = [0, +1.5]:** 3× `sa_place_approach` (13:10–13:12): Stud-Position wiederholbar ±0.3 mm, Roboter erreicht in y jedes Mal 0.9–1.1 mm zu wenig (kein Integrator), x 0.0; von Hand fehlen 1–2 mm in +y. Pick vorher 2/2 ok mit [0, 0] | Konstanter Rest, grösstenteils Roboter-Regelfehler; gilt auch für den Pick (Iris-Toleranz reicht) |
| 21.10. | **`target_correction_mm` = [0, +2.5]:** mit +1.5 zwei Läufe (13:14, 13:15, Schraube an zwei neuen Stellen): Roboter setzt davon nur +0.4/+0.8 mm um (bleibt wieder 0.7–1.1 mm vor dem Ziel), von Hand fehlt weiter 1 mm in +y | Roboter-Restfehler in y ist konstant und fällt bei jedem Ziel an → mit in die Korrektur. Sauberer wäre ein Integrator im Controller, vorerst nicht |
| 21.10. | **`target_correction_mm` = [0, +5.0]:** mit +5 (13:20) von Hand zentriert (TCP 4.0 mm in +y neben der zentriert gemessenen Schraube). Ablesungen davor widersprüchlich. Ursache vermutlich: frei hängende untere Hälfte kippt ≈ 1.8° um x (Unterkante links/rechts 49/45 mm über 124 mm), Spiel nicht weiter behebbar; Kamera sitzt an der oberen, starren Hälfte (Ringblatt-Test ok). Test: `approach_clearance_mm` −5 (10 mm tiefer, nur approach_only) | Messung Yannick; Korrektur bleibt empirisch |
| 21.10. | **Schritt 1 schneller, `full` in einem Zug:** Iris öffnet während der Fahrt zu `observe_nut` (`move_while`); in `full` nach dem Schliessen kein Halt in `observe_nut`, sondern direkt vom Tisch nach `observe_stud` (z führt). `max_speed_mm_s` 150 → 250, Spirale 4.0 → 2.5 s, `arrival_settle_s` 0.3 → 0.15, Vision-`settle_s` 0.6 → 0.4, `start_settle_s` 1.0 → 0.5. Stud-Suche protokolliert jetzt alle Objekte nach jeder Zentrierfahrt (`attempts[].seen`) | Wunsch Yannick („1 fluidic motion“). Zwei `full`-Läufe (13:43, 13:46) brachen nach der Zentrierfahrt ab (Roboter 1.5 mm am Ziel, Vision: nichts innerhalb 20 mm), `approach_only` ohne Mutter klappte; Ursache offen → Diagnose im Record |
| 21.10. | **Schrauben auch ohne Kontakt** (`place.screw_without_contact: true`): erreicht das Absenken `contact_force_n` nicht, wird am tiefsten Punkt trotzdem geschraubt (vorher Abbruch). `max_descend_mm` 12 → 17 | Lauf 13:57: Schraube gefunden, Mutter rutschte ohne 4 N aufs Gewinde → Abbruch, M4 nie aktiv. M4 selbst läuft (Tester), dreht aber leer: 3D-Druck-Mitnehmer greift nicht mehr |
| 21.10. | **Stud-Grössentoleranz vorläufig 0.8** (2–18 mm, vorher 0.4), `dark_threshold` bleibt 0.6 | Zentriert gemessene Grösse: Approach-Tests 13:09–13:38 11.3–12.1 mm, `full`-Läufe ab 13:49 12.6–16.7 mm → 14.01 mm abgelehnt, Mutter fiel (sichtbar dank `attempts[].seen`). Ursache laut Yannick: provisorische Halterung der Schraube vergrössert das dunkle Quadrat. Mit der richtigen Halterung zurück auf 0.4 |
| 21.10. | **Stud-Mitte an den Kanten verfeinert** (`refine_square`, `targets.stud.refine_*`, THEORY 2.0): Linienpaar im Abstand der bekannten Sockelgrösse auf die scharfen Kanten geschoben. Kanten-Methode allein (Canny) **verworfen**: Ausläufer an Gewinde und Lücken, bis 1.4 mm daneben | Trockentest 21.10. (`data/images/stud_dry_20261021/`, Deckenlicht): dunkle Fläche nimmt einseitig Schatten mit (Rechteck 11.0 × 9.3 mm), Mitte 0.5–1 mm zum Schatten verschoben; feste Schwelle trennt Sockel (0.4–0.75) und Schatten (0.6–0.8) nicht. Verfeinerung zentriert 0.2–0.7 mm. Yannick: „untere Kante wird nicht erkannt“ |
| 21.10. | **Werkzeuge:** `ros2 run sa_tasks gripper_test` (Iris/M4 von Hand, Alias `sa_arduino_test`; neu `STOP` im Protokoll), Arduino-Port über `/dev/serial/by-id/…`, `vision.launch.py target:=stud distance_mm:=425` (Alias `sa_vision_bolt`), Debug-Bild nur noch Maske | Ergebnis (Kamerabild als Hintergrund). Jeder Lauf speichert Kamera- und Debug-Bild jeder Erkennung neben dem Record (`<record>_<n>_<label>_<raw|debug>.png`) | Trockentests ohne Roboter; Fehlersuche in `full` ging nur über Zahlen |
| 19.10. | **Iris-Richtung gemessen:** M2 FORWARD + M3 BACKWARD = **öffnen**, M2 BACKWARD + M3 FORWARD = schliessen. Firmware `CLOSE_IS_A_FORWARD = false` | Test Yannick mit `arduino/test_m2_m3` |
| 19.10. | **Gripper-Firmware + serielle Anbindung gebaut:** Firmware `arduino/iris_gripper` = Protokoll §6 (nicht blockierend, `CLOSE` → DONE → Halten mit hold_speed bis OPEN/STOP/Watchdog; Schliessrichtung = A FORWARD/B BACKWARD per Konstante `CLOSE_IS_A_FORWARD`, 115200 Baud). PC: `serial_gripper.py` direkt in `sa_tasks` statt Bridge-Node/Action; `gripper.backend: serial`. Watchdog bleibt: 2 s ohne PING → alles aus. Neuer Schritt `full` (Pick + Place in einem Prozess); `--locate-only` nutzt weiter den Mock | Entscheid Yannick: einfachste Anbindung; lieber Motoren aus als Dauerblockade, wenn der PC hängt. Folge: nach `sa_pick` allein fällt die Mutter 2 s nach Prozessende. pyserial nicht installiert (kein pip) → termios |
| 19.10. | **Translationsdämpfung `ratio_translational` 0.7 → 0.5** (Vorfaktor 2ζ in D = 2ζ√(KΛ) 1.4 → 1.0, für x, y, z; Rotation unverändert 0.5) | Entscheid Yannick: Zittern bei der Ankunft, weniger Dämpfung testen. Modell-Λ (FR3-URDF, ohne Gripper) an observe_nut: D_x, D_y, D_z 214/158/119 → 153/113/85 Ns/m |
| 19.10. | **Rotationsdämpfung `ratio_rotational` 0.7 → 0.5** (Steifigkeit rx, ry bleibt 300) | Mit 300 Nm/rad zitterte der Arm bei der Ankunft stärker. Mehr Dämpfung verstärkt das Zittern (1.0 am Roboter schlechter, 1.3 gab am 13.10. einen 23-Hz-Grenzzyklus): die Dämpfung koppelt das Geschwindigkeitsrauschen ein, also weniger Dämpfung (Entscheid Yannick) |
| 19.10. | **Rotationssteifigkeit rx, ry 130 → 300** (`normal_stiffness` + Start-K in `controllers.yaml`), nur dieser eine Wert geändert | Run-Records 19.10.: Flansch steht schon in der Beobachtungspose 1.3–1.5° schief, nach der Landung 2–2.8°. Der Controller nimmt den Vektorteil der Fehler-Quaternion (sin θ/2), also wirkt nur K/2 = 65 Nm/rad; der Rot-Integrator ist bei 1.5 Nm begrenzt. Offen: Last in Desk (Masse/Schwerpunkt Gripper), Zug der Kabel, Halter-Neigung (`sa_teach` → `flat`, noch nie gemessen), phasenabhängige Steifigkeit/Dämpfung gegen das Zittern bei Ankunft |
| 19.10. | **Zwischenhalt mit Ausrichten verworfen, ersetzt durch Spiral-Suchabstieg** (`pick.spiral`): eine Bewegung bis 10 mm über den Tisch senkrecht über die Mutter, dann ohne Halt eine nach aussen wachsende Spirale (Radius 0 → 6 mm, 2 Umdrehungen, 4 s, Minimum-Jerk) bis Tisch − 2 mm. Zusätzlich schneller: 150 mm/s, Ankunft 0.3 s / 3 mm/s, Vision-Settle 0.6 s | Entscheid Yannick (19.10.) am Roboter: mittig klappt der Pick gut, am Rand des Bereichs hält der Arm die Pose schlecht und steht schief. Das Ausrichten half dort nicht und kostete bis 5 s. Die Spirale deckt einen Zielfehler bis ca. Radius + Fangbereich der Iris ab |
| 07.10. | Ansatz 2 zuerst, Ansatz 1 als optionaler Korrekturschritt | BA: Zentrierung = grösste Streuung |
| 07.10. | Ein ROS2-Workspace, Pick/Place separat startbar | weniger Duplikat, trotzdem einzeln testbar |
| 07.10. | Serial mit ACK/DONE + Watchdog, Zeitsteuerung auf dem Arduino | kein Feedback vorhanden, BA: verlorene Befehle |
| 07.10. | Keine Hand-Eye-Kalibrierung, CAD-Offset + Verifikation/Korrekturterm | Zeitbudget (Abgabe 31.10.) |
| 08.10. | Kamera-Pose relativ zum **Flansch** definieren (`T_flange_cam`), TCP separat | Flansch-Pose kommt direkt aus FK, keine Kette über Gripper nötig |
| 08.10. | Kamera immer vorne (+x_robo), EE-Yaw fix | F28; Pixel-Mapping bleibt konstant, Beobachtungspose = Area-Mitte − 44.4 mm in x |
| 08.10. | Achsen-Mapping Kamera↔Basis aus BA übernommen (F27) | bestätigt, Grundlage für geometry.py |
| 08.10. | `T_flange_tcp` = (0, 0, 113.823 mm), Werte < 0.2 mm aus CAD = 0 | CAD; alter Kamera→Gripper-Offset (52.1) ist damit überholt |
| 14.10. | Neu kalibriert (BA-Werte nicht mehr verfügbar), Auflösung fix 1280×720 | Schachbrett 8×11 innere Ecken, 30 mm; enges FOV (≈185×104 mm bei d = 300 mm) → Beobachtungshöhe pro Area so wählen, dass die Area ins Bild passt |
| 14.10. | Workspace `software/` mit `sa_interfaces` (`Detection.msg`) + `sa_vision`; altes Gerüst (`main.py`, `franka_pick_place/`) entfernt | §7 |
| 14.10. | YAML **pro Paket** (`sa_vision/config/vision.yaml`) statt zentral in `sa_tasks/config` | Paket eigenständig start- und testbar, sa_tasks existiert noch nicht |
| 14.10. | `vision_node` liefert Pixel (roh + entzerrt) und Punkt im **Kamera-Frame** mit d aus YAML; Basis-Frame erst mit FK (sa_tasks) | Roboter/TF noch nicht verfügbar; `geometry.py` hat den allgemeinen Strahl-Ebene-Schnitt bereits |
| 15.10. | **Median des Mittelpunkts über n = 5 Frames** (u, v, d), gültig erst bei vollem Fenster, Neustart nach ungültigem Frame oder Sprung > 0.25 s_px. Maskenglättung und Sechskant-Fit verworfen | 60 Live-Frames, Kamera still: Einzelframe 0.9 px Std (Randpixel kippen an der Schwelle), Maskenglättung 0.8–0.9 px, Sechskant-Fit 0.6 px, Median 5 Frames 0.08 px. THEORY §2.7 |
| 15.10. | **Zielmutter = Sicherungsmutter M8 mit Nylonring** (SW 13). Sie wird nur so weit aufgeschraubt, bis der Nylonring greift, nicht ganz | Entscheid Yannick (15.10.) |
| 15.10. | Handtest ohne Roboter: `d_source` in vision.yaml, `yaml` = fester Wert (später FK), **`from_size`** = d = f·s_Ziel/s_px (Standard für jetzt). Grössenfilter dann nur Plausibilität von d (200–500 mm). Debug-Bild wie BA: Kamera-Frame oben links, Roboter-Frame unten rechts, Hauptpunkt als Kreuz mit Linie zur Mutter; Mutter grün, oben rechts s, d, dx_robo/dy_robo bzw. rot „NO NUT" + Grund; Infozeile Modus/fps/d-Quelle oben links. Optional 2×2-Ansicht der Zwischenschritte (`debug_view: steps`). Start mit einem Befehl `sa_vision` | Testen auf jeder Höhe ohne FK. Einschränkung: Grösse wird angenommen, nicht gemessen; andere Muttergrössen nicht unterscheidbar (→ „2 candidates"). Am Roboter auf `yaml`/FK zurückstellen |
| 15.10. | Detektion umgebaut (Entscheid nach Auswertung der 27 Handaufnahmen): **A'** = Referenzbild der leeren Area aus der Beobachtungspose (`ros2 run sa_vision capture_reference`), Fallback **A** = nur lokaler Median; danach zweiseitige Schwelle. Belichtung + Weissabgleich fest über v4l2 (`camera.yaml`, `apply_camera_controls` im camera.launch.py) | Mutter spiegelt (heller *und* dunkler als Tisch), Loch dunkel → Otsu/„helles Loch" fand 0/27. Referenzbild entfernt Statisches (Verlauf, Tischkante, Roboterteile); lokaler Median (Fenster 1.25·s_max) dämpft den weichen Eigenschatten. usb_cam 0.8 nutzt alte v4l2-Namen → wirkungslos auf Kernel 6.8 |
| 15.10. | Loch nur „vorhanden und mittig" (Kern/Silhouette 0.15–0.85, Versatz ≤ 0.25 s). **Mittelpunkt = Schwerpunkt der Silhouette** (statt Loch). **Filter s/e** (0.78–0.95) statt Kompaktheit | Gemessenes Kernverhältnis 0.21–0.70 je nach Muttertyp, nicht die theoretischen 0.20–0.29; Kern-Schwerpunkt bis 0.18 s neben der Mitte. Kompaktheit trennte den Schraubenfuss (0.87) nicht von Muttern (0.92–0.96), s/e schon (0.39–0.67 vs. 0.82–0.91) |
| 15.10. | Optionaler Parameter `target_across_flats_mm` (gesetzt: 13 mm = M8): gesetzt → nur diese Schlüsselweite (±15 %); 0 → beliebige Grösse M4–M12, mehrere Kandidaten = Fehler. Ausgabe immer s in mm; Gewindegrösse nur bei Eindeutigkeit (ISO 4032 + DIN 934, ±5 %). Zielmutter: Sicherungsmutter M8 (siehe oben) | Mutter und Stiftschraube sind M8; grosse Mutter in den Testbildern ≈ 19 mm (M12 DIN). 17 vs. 18 mm (M10 DIN / M12 ISO) nur 5.6 % auseinander → dort keine Zuordnung. Auswertung 20/23 M8 ohne Fehlalarm, **Schwellen auf denselben Bildern gewählt** → faire Rate mit zweitem Bildsatz am Roboter |
| 15.10. | Testmutter ist **M8**, nicht M6. Detektion soll **grössenunabhängig** sein: Sechskant + Loch erkennen, Schlüsselweite in mm aus s_px·d/f messen und der nächsten ISO-Grösse zuordnen | Verhältnis Loch/Aussenfläche ist über M4–M12 fast konstant (0.20–0.29) → ein Filter für alle Grössen; Grösse kommt als Messwert heraus statt als Annahme. Stiftschraube M8 und Greifen mit der Iris bestätigt (15.10.) |
| 16.10. | **Bench-Aufbau statt Roboter** (Roboter noch nicht nutzbar): Gripper + Kamera fest auf Stativ, Linse ≈ 410 mm über Tisch (Lineal), diffuse Lampe, Belichtung 10 ms (`exposure_time_absolute: 100`, Tisch-Median 180, kein Clipping; 25 ms clippte 60–94 %). Gedrucktes Raster 7×4, 30 mm (`tools/make_grid_sheet.py`, `data/grid/`), Auswertung `evaluate_grid` (Ähnlichkeits-Fit Blatt → Bild, THEORY §2.8). Datensatz `data/bench/diffuse/` (Referenz, 26 Rasterbilder, 11 Extras; Marken 24/25 vom Gripper verdeckt) | Gleiche Bedingungen wie am Roboter (feste Pose, feste Belichtung, Referenzbild, d bekannt) → faire Erkennungsrate und Positionsfehler in mm ohne Roboter |
| 16.10. | **Vordergrund über Gradient von I/B** (`segmentation: edges`, Sobel 3×3 > 0.35, Konturen füllen vor dem Opening) statt Helligkeitsschwelle; Schwelle bleibt als Option | Bench: Schwelle 4/26 + 7/11 Extras, Kanten **26/26 + 11/11**. Unter der Lampe sind die Flächen fast so hell wie der Tisch, der weiche Schatten ist dunkler → Schwelle segmentiert Loch + Schatten. Schatten einer ausgedehnten Lichtquelle hat kleinen Gradienten. Nachteil: Handbilder 15.10. (ohne Lampe/Referenz) 12/27 statt 20/27 |
| 16.10. | Positionsfehler Bench: RMS 1.45 mm, max 2.55 mm (inkl. Handplatzierung der Mutter), ohne systematisches Muster; s = 13.30 ± 0.34 mm. Kreuze des Blatts: Homographie-Rest 0.06 mm → Kalibrierung ok; ~0.5 mm durch Schiefstand der Stativkamera | Obergrenze für den Detektionsfehler; Iris-Fangbereich muss > ~2.5 mm sein (offen) |
| 16.10. | **Projektionszentrum ≈ 9 mm über der Linsen-Unterkante** (gemeint: Unterkante Kameragehäuse, dort lag das Lineal; siehe Eintrag neue Halterung) (Fit d = 419 mm, Lineal 410 mm) → d aus FK + CAD-Offset wäre ~2 % zu klein. Am Roboter einmal das Rasterblatt aus der Beobachtungspose aufnehmen, d_fit − d_FK als konstanten Offset in YAML | 2 % Massstabsfehler = 2 mm Fehler bei 100 mm Abstand von der Bildmitte (THEORY §2.8, §4) |
| 16.10. | Workstation: franka_ros2 **v2.2.0 + libfranka 0.19.0** nativ in `~/franka_ros2_ws` (wie Matteos Container, mit dem der Controller am 13.10. lief); `cartesian_impedance_control` + `messages_fr3` in `software/src`. `sa_vision` baut nur `--packages-up-to sa_vision`, Vision unabhängig vom Controller. Kamera über `/dev/v4l/by-id/...` (videoN wechselt beim Umstecken) | System-libfranka 0.13.2 zu alt; Workstation nur von mir genutzt. Am Ende ein Bringup-Launch, der Vision, Controller und Gripper einbindet |
| 16.10. | Kanten **nach Nähe gruppieren** (`edge_group_ratio` 0.1, Hülle nur der Kantenpixel) statt Schliessen + Füllen; **Referenzbild nur bei passender Beleuchtung** (Median I/R in [0.8, 1.25], sonst ignoriert + Warnung) | Live-Test mit anderem Licht (Lampe aus/anders, harter Schatten): Referenz passte nicht (Median 0.45), Umriss der Mutter offen → 0 Treffer. Gruppierung: Bench weiter 26/26 + 11/11, Handbilder 18/27 statt 12/27. Mutter neben ihrem harten Schatten bleibt unsichtbar → **diffuses Licht von oben ist Voraussetzung**, Lampenposition gehört zur Beobachtungspose |
| 16.10. | **Neue Gripper-/Kamerahalterung (CAD 16.10.):** `T_flange_cam` = (0, 44.4, 57.223) mm (vorher 60.223), `T_flange_tcp` = (0, 0, 118.823) mm (vorher 113.823). Bezugspunkt Kamera = unterster Punkt des Gehäuses (Linse 5 mm höher), Flansch = Auflagefläche (Zentrierzylinder ragt in den Arm). Kamera-Orientierung unverändert → Achsen-Mapping F27/F28 und `camera_to_robot_xy` bleiben. Fusion-Baugruppe jetzt in Basis-Achsen (Kamera bei +x), Messung als Delta Flansch → Punkt. CAD-Werte < 0.2 mm = 0 (CLAUDE.md vereinheitlicht). STEP + STL in `cad/export/`. 3D-Viewer (`visualization/`) nachgeführt: neues Gripper-Mesh, `GRIP_LEN` = 118.823 mm (vorher 156.3, alter Entwurf), IK erreicht alle Wegpunkte | Kamera 3 mm höher, Gripper 5 mm länger → Kamera bei aufliegendem Gripper 61.6 mm über Tisch (vorher 53.6). Gleicher Bezugspunkt wie das Lineal am Bench → Befund „Projektionszentrum ≈ 9 mm darüber“ bleibt gültig, Kalibrierung am Roboter wie geplant. Kontrolle im Mesh: Kameragehäuse unten bei z = −57.22, Bodenplatte bei −118.82 |
| 19.10. | **Mehrere Objekte im Bild:** nur dann Formfilter (Breitenverhältnis der Hülle 0.60–0.95, Sechseck 0.866, Bolzen tiefer, Scheibe ≈ 1) → ein Kandidat = Mutter; mehrere → Grössenfilter (s am nächsten bei 13 mm, ±40 %). Overlay: rot = Form falsch, orange = mutterförmig aber nicht gewählt, grün = gewählt. Ein Objekt im Bild: weiterhin kein Filter | Live-Szene 19.10.: liegender Bolzen + 2 Muttern → grösster Blob (Bolzen) wurde genommen. Toleranz weit, weil der Schatten s um ~30 % vergrössert. Handtest 19.10.: funktioniert einigermassen; Grössenfilter erst am Roboter prüfbar (d nur dort bekannt). Im Betrieb ist ohnehin nur eine Mutter im Bild |
| 19.10. | **Aufräumen: alte Vision-Pipeline gelöscht, `simple_vision` heisst jetzt `sa_vision`** (Node `vision_node`, `config/vision.yaml`, Start `vision.launch.py` bzw. Alias `sa_vision`). Behalten: `geometry.py` (sa_tasks), `camera_controls.py` (camera.launch.py), `capture_images`. Gelöscht: Detektion mit Referenzbild/Kanten/Filtern/Median, `evaluate_images`, `evaluate_grid`, `grid_fit`, `capture_reference`, `simple_vision.launch.py` | Entscheid Yannick: auf der einfachen Variante aufbauen, Überblick behalten. Alter Code bleibt in der git-Historie; THEORY §2.1–2.8 als „dropped“ für den Bericht |
| 19.10. | **Zurück zur einfachsten Variante:** `simple_vision` = Bandsperre um den Tisch \|I/B − 1\| > **0.18** → grösster Blob → Sechseck auf die Hülle. Inverse Binärschwelle, Lochmethode, Schwarz-/Bandfilter **verworfen** (Code entfernt). Debug-Ansicht `steps` = Kamera \| Maske \| Ergebnis bleibt | Entscheid Yannick nach Live-Test: 0.18 trennt Tisch (0.9–1.1) und Schatten (0.8–0.9) am besten; die Lochschwellen vom Bench passten live nicht (Loch dort 0.43–0.48 statt 0.35) |
| 19.10. | ~~**Lochmethode mit zwei Filtern:**~~ (verworfen 19.10.) „schwarz“ r < 0.4 (nur Scheibe) und „Band“ 0.4–0.9 (grauer Mutterkörper ohne Scheibe), Vereinigung → Blobs; Kreis aus der **konvexen Hülle** des Schwarzbereichs | Idee Yannick (strenge Schwelle). Ohne Hülle bleibt bei 0.4 oft nur eine Sichel (Reflex an der Bohrungswand) → 10/26; mit Hülle 26/26, RMS 1.36 mm, Extras 11/11. 0.35 → 20/26. Band 0.7–0.9 gleich gut, lässt aber 0.48–0.7 aus. Handaufnahmen 3/27 (0.5: 15/27) |
| 19.10. | ~~**`simple_vision`: Mutterzentrum aus dem Loch**~~ (verworfen 19.10.) (`nut_method: hole`, Standard für die Mutter; Stiftschraube bleibt `hull`, `hull` jederzeit zurückschaltbar). Dunkle Scheibe (Loch + Nylonring, r < 0.5) → Kreisfit → Zentrum; Sechseck nur konzentrisch gezeichnet. Prüfungen: (1) klarer Kreis (Abweichung, Füllung, nahe Hüllenzentrum, kleiner als Hülle), (2) Hülle sechseckig, nicht rund (Breitenverhältnis 0.65–0.93, Scheibe ≈ 1.0) | Idee Yannick: das Loch sieht immer gleich aus, Drehlage egal. Bench: 26/26, RMS 1.40 mm (hull 1.30), Extras 11/11 statt 9/11 (liegende Stiftschraube nicht mehr als Mutter erkannt). Handaufnahmen 15.10. nur 2/27 (Seitenlicht, Schatten) → am Roboter live prüfen (THEORY §2.0b) |
| 19.10. | ~~**`simple_vision`: inverse Binärschwelle**~~ (verworfen 19.10.) auf r = I/B (r < 0.75 → 1) als Standard, zweiseitige Schwelle bleibt wählbar (`segmentation`). Debug-Ansicht `debug_view: steps` = Kamerabild | Binärmaske | Ergebnis nebeneinander auf `/vision/debug` | Bench-Set 16.10.: 26/26, RMS 1.30 statt 1.35 mm, Schlüsselweite 14.2 statt 17.1 mm (weniger Schatten im Blob); Extras 9/11 wie vorher (THEORY §2.0). Am Roboter noch prüfen: helle Glanzstellen fehlen in der Maske |
| 19.10. | **Motorbelegung fix:** Iris = M2 + M3 (gegensinnig), Rotation = M4. **Port M1 ist gesperrt**, dort nie einen Motor anschliessen (ersetzt die provisorische Annahme M2 + M4 / M1 aus F5) | Entscheid Yannick (19.10.); Firmware prüft per `static_assert`, dass kein Port = M1 ist; Testsketch `sketch_sep24a` angepasst |
| 18.10. | **Stand Ende Session 18.10.:** mit Korrektur (−9.0, +8.2) mm + Zwischenhalt (30 mm, Ausrichten, senkrecht runter) sehr gute Treffer; **ohne** Korrektur nur 1/2 → Korrektur bleibt vorerst aktiv. Gemessene Neigung des Flansches: Beobachtung 0.65–1.35°, Zwischenhalt 1.0–1.5° (0.2° nie erreicht → Toleranz 1.2°), nach Landung 1.3–2.1°. Gripper wirkt sichtbar nicht gerade. **Nächste Schritte:** (1) Halter-Neigung messen (`sa_teach` → 4 × `flat`, Proben vergleichen), (2) `use_gripper_tilt: true` + Korrektur 0 + 5 Verifikationsläufe mit neuen `truth`-Punkten, (3) mechanisch prüfen, ob Gripper/Kamera am Adapter schief sitzen; (4) Gripper-Firmware + Serial-Bridge (Pick ist noch Mock), (5) Stiftschraube: Bilder der stehenden Schraube, Schritt 2 testen. `messages_fr3` noch nicht im Repo (keine Lizenz) | Für die nächste Session |
| 18.10. | **Versuch: Zwischenhalt über der Mutter** (`pick.align`, 30 mm über Tisch): xy-Bewegung bis senkrecht über die Mutter, warten bis gemessene = kommandierte Orientierung (0.2°, max 5 s), dann senkrecht runter. Neigung bei Beobachtung, Zwischenhalt und Landung im Run-Record | Yannick: einfacher Test, ob eine Schräglage beim Absenken die Fehlgriffe verursacht, vor dem Test mit gemessener Halter-Neigung |
| 18.10. | **Versuch: Gripper statt Flansch waagrecht** (`use_gripper_tilt`, Standard aus). Neigung des Grippers gegenüber dem Flansch wird mit `sa_teach` → `flat` gemessen (Bodenplatte flach auf den Tisch gedrückt, 3–4 Proben gemittelt, in `poses.yaml`); eingeschaltet kommandieren alle Bewegungen den Gripper-Frame {G} auf (1, 0, 0, 0), Kamera und TCP in {G}. Wahrheitspunkte speichern jetzt die Flansch-Pose, `compare_runs` rechnet sie mit der aktuellen Neigung neu. A/B-Test: Schalter an + `target_correction_mm` = 0 | Yannick sieht den Gripper schräg auf den Tisch zufahren und will keine fest codierte Korrektur. Eine Halter-Neigung von ≈ 1.5° erklärt den Versatz (−9, +8) mm vollständig; bei den Wahrheitspunkten stand der Flansch 0.9–3.2° schräg |
| 18.10. | **Korrektur `target_correction_mm` = (−9.0, +8.2) mm** aus 4 Verifikationsläufen (Lauf 3 verworfen: Mutter nicht im Umriss, berechnete Position = Position 1). Ausführungsfehler des Roboters 0.2–0.7 mm, Rechenfehler fast konstant, Rest nach Korrektur 2.5–4.5 mm (Ähnlichkeits-Fit: Massstab 1.025, Drehung −1.6°, Rest 1.4–3.2 mm, mit 4 Punkten nicht belastbar). Belichtung ohne Lampe 300 (30 ms) | Fehler liegt in der Umrechnung Bild → Roboter (Kamera-Neigung ≈ 1.5° und/oder Linsenlage), nicht im Roboter; Pos. 1 mit `sa_locate` (−11.8, +3.9) und `sa_pick` (−11.1, +4.4) reproduzierbar |
| 18.10. | **Verifikation:** `sa_teach` → `truth` speichert die TCP-Position (Gripper von Hand flach über der Mutter) als Wahrheit, `sa_locate` die berechnete Position, `sa_compare --since …` paart sie in Reihenfolge und schlägt `target_correction_mm` (Mittelwert) vor; Streuung = was eine Konstante nicht korrigiert. Nach Nachjustieren der Kamera: 2/3 Erfolge. `sa_pick` speichert zusätzlich die gemessene Landeposition des TCP (bleibt die Bodenplatte auf der Mutter stehen, wird das protokolliert statt abgebrochen) → `sa_compare` trennt Rechenfehler (Wahrheit − Ziel), Ausführungsfehler (Landung − Ziel) und Gesamtfehler (Wahrheit − Landung). Erster Punkt: Rechenfehler x −11.8, y +3.9 mm, Wiederholbarkeit zweier Messungen 2.6 mm | Plan Yannick: 5 Läufe manuell vs. Pipeline vergleichen (THEORY §4) |
| 18.10. | **Erste Pick-Läufe:** ohne Lampe 1/3 Iris über Mutter, mit Lampe 0/1 (harter Schatten verschiebt den Blob-Schwerpunkt). Overlay sitzt gut auf der Mutter → Fehler in der Umrechnung Bild → Roboter, nicht in der Detektion. Ein Lauf endete mit z +9.9 mm über Ziel bei Stillstand = Bodenplatte stand auf der Mutter. Gripper-Schwerpunkt als Ursache ausgeschlossen (≈ 0.05 Nm → < 0.1 mm). Verdacht: Kamera-Neigung am Halter (1° ≈ 7 mm bei d = 400 mm), Linsenlage im Gehäuse. Neu: `sa_pick --locate-only` (`sa_locate`) und Rest-Fehler jeder Ankunft im Run-Record → Verifikationsmessung für `target_correction_mm` | Methode wie in §5.1 geplant (Mutter an bekannter Position, Rest-Fehler messen, konstanter Korrekturterm) |
| 18.10. | **Ankunft = Stillstand** (< 2 mm/s während 0.5 s, innerhalb 8 mm) statt 3 mm Positionstoleranz; Rest-Fehler pro Achse im Log. Geschwindigkeit 100 mm/s. Viewer zeigt d und Versatz der Mutter zur Bildmitte in Roboter-Achsen | Erster Roboterlauf: Beobachtungspose mit 3.6 mm Restfehler erreicht → Abbruch nach 15 s. Zielberechnung nutzt die gemessene Pose, Restfehler stört dort nicht; beim Absetzen zeigt das Log, ob er die Präzision begrenzt |
| 18.10. | **sa_tasks auf fixe Orientierung (1, 0, 0, 0) und {F} in Hand-Achsen umgestellt:** einzige Orientierung = Quaternion (1, 0, 0, 0) aus `tasks.yaml` (`flange_orientation_xyzw`), geteachte Posen liefern nur die Position. {F} = fr3_link8 um −45° um z (`link8_to_flange_yaw_deg`), ${}^{B}T_{F} = {}^{B}T_{E} ({}^{L}T_{E})^{-1} {}^{L}T_{F}$; Kamera (44.4, 0, 57.223) mm in {F}, Kamera-Achsen Rz(+90°). `orientation_from`/Gier-Logik entfernt. Vor jeder Bewegung fragt `pick`/`place_screw`, ob die Kamera nach der Controller-Aktivierung von der Basis weg zeigt | Festlegung Yannick (siehe Korrektur-Eintrag oben); Tests: Kamera in Startpose bei +44.4 mm in x, Achsen-Mapping wie `camera_to_robot_xy`, Ergebnis unabhängig von der EE-Einstellung in Desk |
| 18.10. | **Korrektur (ersetzt die zwei Einträge darunter): Arm bleibt in der Controller-Startpose, EE-Quaternion (1, 0, 0, 0), q7 = 0.80 – das ist der Aufbau und die einzige Orientierung. Der Gripper ist so auf den Flansch geschraubt, dass die Kamera in dieser Pose vorne (+x_robo) sitzt; keine Anpassung zu Beginn eines Runs.** `controllers.yaml` zurück auf (1, 0, 0, 0) und q7 = 0.80 (Commit 05a50f7 rückgängig). `T_flange_cam` = **(44.4, 0, 57.223)** mm in {F} (EE-Achsen), in fr3_link8-Achsen (31.4, −31.4, 57.223). R_EE_cam = Rz(+90°), Achsen-Mapping F27 und `camera_to_robot_xy` unverändert. 3D-Viewer: Gripper-Modell 90° um die Werkzeugachse gedreht, IK-Gier 0 | Entscheid Yannick (18.10.). Fehlerquelle: Viewer und Doku hatten den Gripper wie die Franka Hand auf dem Flansch angenommen (Kamera bei +y_EE), das stimmt nicht; daraus folgten die falsche Quaternion (0.7071, 0.7071, 0, 0) und die 90°-Drehung von Gelenk 7 |
| 18.10. | **Achsen des Flansch-Frames {F} korrigiert:** {F} hat die Achsen des Franka-Hand-/EE-Frames (fr3_link8 um −45° um z gedreht), nicht die von fr3_link8. `T_flange_cam` = (0, 44.4, 57.223) mm gilt in diesen Achsen; in fr3_link8-Achsen (31.4, 31.4, 57.223). Gripper sitzt wie die Franka Hand auf dem Flansch (wie im 3D-Viewer, von Yannick bestätigt). Soll-Orientierung „Kamera vorne“: EE-Quaternion xyzw = (0.7071, 0.7071, 0, 0), fr3_link8-Gier dann +45°; von der Startpose (1, 0, 0, 0) aus dreht Gelenk 7 um −90° (weit weg vom Limit) | THEORY §0 schrieb {F} = `fr3_link8` → eine Session las den Vektor in fr3_link8-Achsen und setzte die Kamera an der Startpose „hinten rechts“ (−31, −31) statt rechts (0, −44.4) und eine Drehung um +134° an. Der Fehler stammt aus meinem Update vom 16.10. (Label nicht geprüft) |
| 19.10. | **Kamera neu kalibriert, 3 unabhängige Läufe, Mittelwert in `calibration/hd_camera.yaml`:** f = (2096.0, 2092.3), c = (695.4, 404.1) px. Streuung zwischen den Läufen: c_x ±7.9 px, c_y ±5.1 px → auf dem Tisch (d = 416 mm) max. 2.1 mm vom Mittel; alte Kalibrierung (14.10.) max. 0.75 mm vom neuen Mittel | Prüfen, ob die Kalibrierung den konstanten Versatz (−9, +8) mm erklärt: nein, dafür bräuchte es ≈ 45 px Fehler im Hauptpunkt. Verdacht jetzt: Neigung (1° ≈ 7 mm auf 416 mm) → `teach_poses flat` + `use_gripper_tilt` |
| 19.10. | **CAD `Configuration_1` (STEP + STL) in `cad/export/`; Kamera→TCP als Kontrolle:** Flansch→Kamera (44.4, 0, 57.223) und Flansch→TCP (0, 0, 118.823) unverändert. Neu `camera_housing_to_tcp_mm` = (−44.505, 0.105, 61.6) in {F} in `tasks.yaml`; `load_tasks` prüft, dass Flansch→Kamera + Kamera→TCP = Flansch→TCP (Toleranz 0.2 mm pro Achse), sonst Abbruch. Pick/Place loggen und speichern den TCP-Versatz pro Bild als `tcp_move` = Anteil aus dem Bild (Objekt relativ zur Kameraachse) + Anteil Halterung (TCP → Kameraachse, ≈ +44.4 mm in x) | Zwei unabhängige CAD-Messungen derselben Halterung: Abweichung 0.105 mm in x und y (Kamera-Mittelpunkt in Fusion leicht unterschiedlich gewählt) → unter der 0.2-mm-Regel. Die Bewegung selbst rechnete schon über die Frame-Kette; neu sieht man im Run-Record, ob ein Fehler aus dem Bild oder aus dem Halterungs-Offset kommt |
| 16.10. | **BA wird ignoriert:** keine Werte, Frames, Parameter oder Methoden aus der BA übernehmen, alles aus der aktuellen Hardware und diesem Repo ableiten. BA-Zahlen nur noch als Vergleich im Bericht. Frühere Einträge mit „aus BA übernommen“ (z.B. Achsen-Mapping F27, Kontaktschwelle −4 N in §5.3) gelten nur, soweit sie am neuen Aufbau bestätigt sind | Entscheid Yannick (16.10.): aus der BA ist praktisch nichts verwendbar |
| 18.10. | **`sa_tasks`** (Schritt 1 `pick`, Schritt 2 `place_screw`, `teach_poses`): Ziel im Basis-Frame per Strahl-Ebene-Schnitt mit gemessener Flansch-Pose (`F_T_EE` aus dem Robot-State → unabhängig von der EE-Einstellung in Desk), Referenz an `/cartesian_impedance_controller/reference_pose` mit 100 Hz gestreamt. Beobachtungsposen + Tischhöhe werden geteacht (Arm von Hand führen, `robot_state.launch.py` ohne Controller), Orientierung immer fix nach unten (geteachte nur geprüft, max 5° Neigung). Gripper vorerst Mock (kein Serial-Bridge) | Vorgabe Yannick: feste Positionen anfahren, dort suchen, in einem Zug runter, zurück; Code in zwei Schritte getrennt |
| 18.10. | **Eine Bewegung nach unten** statt Anfahrpose + Absenken: Minimum-Jerk, xy fertig bei 60 % der Zeit, Rest senkrecht (umgekehrt beim Hochfahren). TCP-Ziel 2 mm unter dem Tisch (≈ 3 N Vorspannung bei k_z = 1500 N/m), damit die Bodenplatte flach aufliegt | Vorgabe „in einem Zug“; senkrechtes Ende verhindert, dass der Gripper über die Mutter streift (THEORY §5) |
| 18.10. | **Stiftschraube per Vision** (`detect_stud`: Quadrat 10 mm, s/e ≈ 0.71) statt geteachter Position, `vision_node`-Parameter `target` (nut/stud) wird von `sa_tasks` gesetzt; eigenes Referenzbild pro Beobachtungspose (`reference_image_stud`). **Noch nicht an echten Bildern getestet** (keine Bilder der stehenden Schraube) | Vorgabe Yannick (Zentrieren über der Schraube per Vision). Objektebenen: Mutter-Schwerpunkt 4 mm, Schraubenbasis 20 mm über Tisch |
| 18.10. | **Neue, einfache Vision `simple_vision` (eine Datei) als Standard, `sa_vision` bleibt für später.** Annahme: genau ein Objekt auf leerem Untergrund. Keine Grössen-/Formprüfung, kein Referenzbild: Hintergrund = stark geglättetes Bild (σ 60 px, auf 1/8 gerechnet), Vordergrund wo \|I − B\|/B > 0.15, Schliessen/Öffnen, grösster Blob, der den Bildrand nicht berührt → engstes regelmässiges Sechseck um die konvexe Hülle (Orientierung mit minimaler Summe der drei Breiten) → Mittelpunkt. Gleicher Node-Name/Topics wie `sa_vision` → `sa_tasks` läuft mit beiden. Start: `sa_simple` | Live-Aufbau (Kamera am Roboter, Licht) zu schwach für die strengen Filter von `sa_vision`, Vorgabe Yannick: erst „ist da etwas?“, Sonderfälle später. Bench 16.10.: 26/26, RMS 1.36 mm (sa_vision 1.40), leeres Blatt → nichts, abgeschnittene Mutter → nichts; s um Schatten zu gross (17 statt 13 mm), Mitte unbeeinflusst. Grenzen: mehrere Objekte → grösstes gewinnt; Schatten/Glanzlichter fliessen ein. Was `sa_vision` zusätzlich kann (Kanten statt Helligkeit, Referenzbild, Grössenfilter, Loch-Prüfung, Stiftschrauben-Form, zeitlicher Median), steht in THEORY §2 und kann einzeln zurückgeholt werden |
| 18.10. | ~~**Orientierung aus der geteachten Pose**~~ **verworfen, ersetzt durch den Eintrag „sa_tasks auf fixe Orientierung (1, 0, 0, 0)“**: (`orientation_from: observe_nut`, nur nivelliert, Gier bleibt) statt fixer Matrix `flange_rotation_down`. Teach-Tool zeigt Gierwinkel und wo laut CAD die Kamera relativ zum Flansch liegt | Erste Teach-Werte lagen ~135° von der angenommenen Flansch-Orientierung (x_F = +y_B …, Herkunft BA/F28, am Roboter nie geprüft) → sonst 135°-Drehung des Handgelenks. **Offen: am Roboter prüfen, ob Kamera-Richtung (CAD, +y_F) und Bildachsen (x_C = −x_F, y_C = −y_F) stimmen** |
| 14.10. | Detektion nur gültig bei **genau einem** Kandidaten; Mittelpunkt = Loch-Schwerpunkt; Zusatzfilter Konzentrizität Loch/Aussen | lieber Retry als falsche Mutter greifen |
| 07.10. | Keine Orientierungsbestimmung der Mutter | Iris zentriert selbst (F14) |
| 07.10. | Mutter über inneres Loch detektieren, ROI = bekannte Area | keine Lackierung mehr, robustes Merkmal |
| 07.10. | 2 Steifigkeits-Presets (HIGH/LOW) | reicht laut F2 |
| 07.10. | Clean Slate, Code auf Englisch | F3, F19 |

---

## Anhang A – Antworten Runde 1 (07.10.2026, gekürzt)
- **F1** Ubuntu, Version unbekannt; ROS2-Distro unbekannt.
- **F2** gleicher CIC wie BA, variabel pro Phase; 2 Stiffness-Stufen reichen.
- **F3** nichts übernehmen, max. Prinzip der Vision-Pipeline, evtl. Bridge → Clean Slate.
- **F4** alles über ROS2, `software/` war Platzhalter.
- **F5** Arduino R4 Minima + Adafruit Motor Shield v2.3; Iris vermutlich M2 + M4, Rotation M1 (änderbar).
- **F6** Iris über Zeit, kein Feedback, wird iterativ getunt.
- **F7** Halten = Motoren drehen weiter, in BA ohne Erwärmungsprobleme.
- **F8** Schleifring vorhanden, Rotation unbegrenzt.
- **F9** Schrauben über Zeit; Torque-Sensor am Arm theoretisch möglich, eher nicht.
- **F10** Übersetzungen unbekannt, egal – wird getestet.
- **F11** HutoPi 720p HD USB (OV9726), keine Specs, BA-Kalibrierung evtl. auffindbar.
- **F12** Offset Δx 0, Δy −44.505 mm, Δz 52.1 mm (Stand 07.10.).
- **F13** keine Lackierung mehr, Mutter auf weissem Untergrund.
- **F14** Orientierung nicht nötig, später ggf. nachrüsten.
- **F15** M6; Stiftschraube mit quadr. Basis 10×10×20, Gewinde 19 mm, fixiert.
- **F16** beliebige Position innerhalb fester Areas (eine Kamera).
- **F17** Gripper-Unterseite auf Tisch, Iris-Zähne 3.5 mm höher – unkritisch.
- **F18** Abgabe 31.10.26, keine Meilensteine, Ziel: höhere Erfolgsquote.
- **F19** Bericht Englisch, BA-Vergleich erwünscht.
- **F20** nichts weiter.

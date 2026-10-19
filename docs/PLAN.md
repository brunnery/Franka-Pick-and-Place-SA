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
| `STATE` | → `<id> DONE iris=<open/closing/holding/opening/off> rot=<on/off>` |

- **Watchdog:** kein Befehl/PING für > 2 s → alle Motoren aus. Die Bridge sendet PING mit 2 Hz. (Achtung: beim Halten essentiell, sonst lässt die Iris die Mutter fallen, wenn ROS hängt – das ist gewollt sicherer Zustand.)
- Konstanten (Ports, Richtungen, Default-Zeiten) oben in der Firmware, Zeiten sonst immer vom PC → Tuning nur in YAML, nicht neu flashen.
- ROS2: `gripper_bridge`-Node als **Action Server** `~/command` (`GripperCommand.action`: cmd, duration_ms, speed, hold_speed → success, message).

---

## 7. Software-Architektur

ROS2-Workspace unter `software/` (Clean Slate). Distro-agnostisch (Humble/Jazzy), nur `rclpy`, keine exotischen Abhängigkeiten.

```
software/
├── src/
│   ├── sa_interfaces/      ament_cmake: GripperCommand.action, Detection.msg
│   ├── sa_gripper/         serial_protocol.py (rein), gripper_bridge node
│   ├── sa_vision/          detection.py, geometry.py (rein), vision_node
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
- [ ] Firmware `arduino/iris_gripper` + Test am Board (ohne Roboter)
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

**F22.** Stiftschraube: Ist die Gesamthöhe 20 mm Basis + 19 mm Gewinde = 39 mm, oder 20 mm insgesamt? Schaut das Gewinde oben aus der Basis heraus?
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

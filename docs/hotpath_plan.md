# Plan: Hotpath, Viertel-IRQ und Hall-Sensorfusion

Stand: 2026-10-18. Alle Zeiten und Zyklen in diesem Dokument sind Schätzungen aus
dem Code, nichts davon ist auf dem Board gemessen (Meilenstein M0 ändert das).

Zeitbasis bei 20 kHz PWM und 170 MHz: eine Halbperiode (ein JEOS-Interrupt) sind
25 µs = 4250 Zyklen, ein Viertel-Zyklus (vier Halbperioden) 100 µs = 17000 Zyklen.

## 1. Ziel

| Kontext | Rate | Inhalt |
|---|---|---|
| Hotpath (JEOS, Zero-Latency) | 40 kHz | nur was vom aktuellen Strom abhängt: Park, PI, Inverse Park, SVM, CCR |
| Viertel-IRQ (TIM8-Update, Zero-Latency) | 10 kHz | alles Vorberechnen für die nächsten 4 Halbperioden, Beobachter, Hall-Fusion |
| Regelthread (Zephyr, normale Priorität) | 1 bis 2 kHz | Drehzahlregler, später Lageregler, MTPA, Feldschwächung, Sollwertbegrenzung |
| Hall-Flanken (EXTI, normale Priorität) | bei Flanke | nur Zeitstempel und Hall-Code erfassen |

Annahme für die Vorberechnung: Drehzahl konstant über die vier Halbperioden
(100 µs). Beschleunigung wird nicht modelliert.

## 2. Ist-Zustand im Code

- `CurrentControlIsr::on_jeoc()` (980 Byte) macht heute alles in einem Zug:
  ADC lesen über Funktionszeiger, Überstromcheck, Bereichsprüfung der Duties
  (liest die CCR über einen weiteren Funktionszeiger zurück), Clarke, Park,
  PI mit Entkopplung aus dem **gemessenen** Strom, Inverse Park, HFI-Spannung,
  Totzeitkompensation, SVM, Schreiben der Duties (nicht inline).
- Der Aufruf von `std::sqrt` im Spannungsbegrenzer wird als libm-Aufruf
  `sqrtf` übersetzt (errno-Behandlung), nicht als `vsqrt`.
- `SlowUpdate::run_once()` (928 Byte) läuft heute im TIM8-Update-Callback,
  pollt `samples_ready`, parkt die vier Proben neu, mittelt, ruft Flussbeobachter,
  Kalman-Prädiktion, HFI und berechnet `sc[0..3]` für den nächsten Zyklus
  (Doppelpuffer mit `active`-Flip).
- `MechanicalObserver` ist ein Kalman-Filter mit den Zuständen ω, θ und
  Lastmoment, nur eine skalare Winkelmessung pro Aufruf von
  `inject_angle_error()`. Dieser führt **jedes Mal auch** die
  Kovarianzprädiktion aus. `PmsmFluxObserver::calculate()` ruft
  `inject_angle_error()` selbst am Ende auf. Die Kommentare nennen Hall-Sensoren,
  im Code gibt es dafür nichts. Der Winkelfehler wird nicht auf (-π, π] gewrappt.
  Prädiktion und Korrektur rechnen `std::sin/cos` (zwei Aufrufe pro Zyklus).
- Phase zwischen Viertel-Zähler des Kerns (`sub_step_`) und TIM8-Update ist nicht
  synchronisiert. Das Update liegt am Peak, die CCR1 bis 3 sind ohne Preload.
- Keine Hall-Erfassung, kein Drehzahlregler im Kern (nur `PositionController`
  in der Lib).

## 3. Entscheidungen

**D1 Preload und synchroner Viertel-IRQ (Empfehlung: ja, wartet auf Bestätigung).**
Heute schreibt der Hotpath die CCR ohne Preload in den laufenden Zähler. Mit
RCR=0 und CCR-Preload wirken die neuen Werte erst an der nächsten Grenze (Peak
oder Valley, in Center-Aligned-Mode erzeugt RCR=0 an beiden ein Update-Ereignis).
Den Viertel-IRQ setzt der Hotpath im Sub-Schritt 3 per `NVIC_SetPendingIRQ`
selbst, damit ist die Phase deterministisch und das Polling von `samples_ready`
entfällt. Das ändert die frühere Vorgabe "Update-IRQ mit RCR=4" ab. Begründung
siehe Abschnitt 3.1.

**D2 Lib-Änderungen (freigegeben).**
- `MechanicalObserver`: Kovarianzprädiktion und Messupdate trennen
  (`PredictCovariance(dt)`, `UpdateAngle(ε, R)`). `inject_angle_error()` bleibt
  als Kombination erhalten.
- `PmsmFluxObserver::calculate()` gibt den Winkelfehler zurück (alte Variante
  als Wrapper).
- Optional: `std::sin/cos` im Beobachter durch die `SinCos`-Tabelle ersetzen.
- Der Flussbeobachter läuft **immer**, auch mit angeschlossenem Geber. Er wird
  nicht abgeschaltet, nur über sein R gewichtet. Messquellen für den Kalman-Filter:
  Fluss (immer), Hall (wenn vorhanden), optional Geber (kleines R). Mit Geber
  liefert der Flussbeobachter zusätzlich eine Plausibilitätsprüfung (Fluss-ε
  gegen Geber-ε). Die Drift des Spannungsmodells im Stillstand muss in M6
  abgesichert werden (Integratorhaltung oder Leckterm bei kleiner Drehzahl).

**D3 Entkopplung (entschieden):** aus dem Mittelwert des letzten Zyklus.

**D4 Eigene Winkeltabelle für die Ausgangsseite (folgt aus D1).** Mit Preload
wirkt die Spannung im Mittel 1,5 Halbperioden nach der Messung. Der Winkel
dafür ist θ + ω·(k + 1,5)·dt, die Verzögerung als Parameter, auf dem Board
abgestimmt. Kostet im Hotpath nichts.

**Hall (entschieden):** Standard 120°. Polpaare spielen keine Rolle, der
mechanische Beobachter und Hall arbeiten im elektrischen Winkel.

### 3.1 Hintergrund zu D1

**Ganzzahlarithmetik: nein.** Auf Cortex-M4F kosten Float-Multiplikation und
-Addition je einen Zyklus (wie ihre Ganzzahl-Pendants), nur Multiply-Accumulate
(3), Division und Wurzel (14) sind teurer. Q15/Q31 bringt in den Rechenzyklen
kaum etwas und kostet Skalierung, Überlaufprüfung und die Umstellung der
float-basierten Lib (`units`). Die Zeit steckt in den Funktionszeigern, dem
Zurücklesen der CCR, `sqrtf` und den Divisionen. Das fixt M3 ohne Ganzzahl.
Der Winkel ist schon Festkomma (`RotorAngle`, Sinustabelle).

**Direktes CCR-Schreiben mit der heutigen Strommessung.** Der früheste Zeitpunkt
für einen Schreibzugriff ist Trigger + Wandlungsdauer (etwa 1,3 µs) + ISR-Zeit,
also etwa 2,5 µs nach der Messung. Eine Phase mit 90 % Duty schaltet aber schon
2,5 µs nach dem Peak (25 µs · (1 − 0,9)), eine mit 95 % nach 1,25 µs. Dann:
- Nur ein Teil der drei Kanäle wirkt noch in dieser Halbperiode, die anderen
  erst in der nächsten. Das verfälscht die Leiterspannungen.
- Das Ergebnis hängt vom ISR-Jitter ab (Viertel-IRQ vor dem Hotpath, Flash-Wartezustände).
- Der nutzbare Duty-Bereich schrumpft auf etwa 10 bis 90 %.

Gewinn des direkten Schreibens: eine Halbperiode weniger Verzögerung (25 µs bei
20 kHz, 12,5 µs bei 40 kHz). Mit Preload gilt 1,5 Halbperioden bis zum Mittel
der Wirkung, direkt etwa 0,5 plus ISR-Zeit.

**Wirkung auf die Zielmotoren.** Hoverboard-Nabenmotoren (36 V, Phaseninduktivität
etwa im mH-Bereich): die Verzögerung spielt keine Rolle. Modellbau-Outrunner
(200 kV, Induktivität typisch 10 bis 50 µH, elektrisch bis über 1 kHz): bei
20 kHz sind 37,5 µs bei 1,5 kHz Regelbandbreite etwa 20° Phase, die D4 beim
Winkel ausgleicht. Bei so kleiner Induktivität ist 20 kHz ohnehin knapp: der
Stromrippel liegt größenordnungsmäßig bei 10 A Spitze-Spitze (25 V, 20 µH). Es
wird sich eine höhere PWM-Frequenz lohnen, und dann halbiert sich das Budget
einer Halbperiode. Das spricht zusätzlich für Preload, weil dort die Frist die
ganze Halbperiode ist.

**STM32H7.** Das Budget einer Halbperiode steigt von 4250 auf etwa 12000 Zyklen
(480 MHz) beziehungsweise 13750 (550 MHz bei H723/H725/H73x). Der Cortex-M7 hat
zwei Pipelines und eine FPU. Auf dem H7 würde ich Code und Hotpath-Daten in
ITCM/DTCM legen (deterministisch, keine Flash-Wartezustände) und aufpassen,
dass der Compiler keine Double-Konstanten erzeugt (`-fsingle-precision-constant`).
Das Konzept (Advanced Timer mit Break, ADC-Injected-Trigger über TRGO) ist
dasselbe. Auch dort ist Preload sinnvoll, aber wegen des Budgets weniger
zwingend.

## 4. Datenfluss

Der Viertel-IRQ füllt pro Zyklus einen Plan (Doppelpuffer wie `SubStepBuffer`,
Flip mit release/acquire) für die nächsten vier Halbperioden:

```
struct HotpathPlan {
  SinCos sc_in[4];          // Park des gemessenen Stroms
  SinCos sc_out[4];         // Inverse Park der Sollspannung (D4)
  Rotor<Voltage> ff;        // Entkopplung, konstant über den Zyklus (ω konstant)
  Stator<Voltage> v_inj[4]; // HFI-Einspeisung pro Sub-Schritt
  Rotor<Current> i_ref;     // Sollwert, hier gelatcht (keine zerrissene d/q-Paarung)
  float inv_vdc;            // 1/V_dc, geglättet, für SVM
  float v_limit_sq;         // (v_max * V_dc)^2 für den Kreisbegrenzer
  float ki_dt_d, ki_dt_q, kb_dt_d, kb_dt_q;  // Integratorfaktoren inkl. dt
  float kp_d, kp_q;
};
```

Der Hotpath liest nur den Plan und schreibt zurück: `i_ab[k]`, `u_dq_last`,
Duties. Der Regelthread schreibt Sollwerte in eine Mailbox, die der Viertel-IRQ
beim Planbau einliest (Seqlock oder Doppelpuffer). Aufgerufen werden aus den
Zero-Latency-Kontexten keine Kernel-Funktionen.

### Hotpath (pro Halbperiode, Ziel 400 Zyklen, 2,4 µs)

1. Zwei Shunt-Kanäle lesen, mit Offset und Gain skalieren, dritte Phase aus
   ia + ib + ic = 0.
2. Clarke und Park verschmolzen mit `plan.sc_in[k]`.
3. Fehler, PI mit `kp`, Integrator mit `ki_dt`, plus `plan.ff`.
4. Kreisbegrenzer über `v_limit_sq`, Wurzel nur bei Sättigung (`__builtin_sqrtf`,
   Build mit `-fno-math-errno`).
5. Inverse Park mit `plan.sc_out[k]`, plus `plan.v_inj[k]`.
6. Totzeitkompensation (Vorzeichen des Stroms, Parameter aus dem Plan), SVM mit
   `plan.inv_vdc` statt Division.
7. Duties begrenzen auf 5 bis 95 %, die CCR direkt schreiben. Kein
   Zurücklesen, keine Funktionszeiger. Das Hardware-Zugriffsobjekt wird über
   CMake gewählt und inline eingebunden.
8. Überstromcheck bleibt vor allem anderen (60 A, `power_stage.Trip()`).
9. Im Sub-Schritt 3: Viertel-IRQ pending setzen.

### Viertel-IRQ (alle 100 µs, Budget etwa 15 µs, Ziel 2500 Zyklen)

1. Proben des letzten Zyklus einlesen (mit `sc_in` des Zyklus erneut parken,
   mitteln wie heute).
2. Mechanische Prädiktion mit dem mittleren Strom (Drehmoment aus i_d, i_q).
3. Kovarianzprädiktion einmal, danach Messupdates: Flussbeobachter, Hall,
   optional HFI. Jedes mit eigenem R und auf (-π, π] gewrapptem Fehler.
4. Plan für den nächsten Zyklus bauen: θ_k = θ + k·ω·dt, `sc_in`, `sc_out`,
   `ff`, `v_inj`, `inv_vdc`, `v_limit_sq`, Integratorfaktoren, `i_ref`.
5. Zustandsüberwachung der Endstufe (Fault, Blocked), Zähler für Zeitmessung.

### Regelthread (1 bis 2 kHz)

Drehzahlregler (PI mit Anti-Windup), später Lageregler (`PositionController`),
MTPA, Feldschwächung, Begrenzung des Stromsollwerts (Betrag auf `motor_i_max`,
50 A). Ausgabe: `i_ref` in die Mailbox. Eingang: ω und θ aus einem Snapshot des
Beobachters.

## 5. Hall-Sensorfusion

Hardware: drei Hall-Signale auf PC13, PC14, PC15 (RC-gefiltert 3k3/2n2,
Zeitkonstante etwa 7 µs). Das sind keine Timer-Eingänge, also kein
Hall-Interface des Timers: GPIO mit EXTI (Leitung 13 bis 15) und Zeitstempel
über den DWT-Zyklenzähler.

**Erfassung.** Der EXTI-Handler (normale Priorität, Zero-Latency-IRQs
unterbrechen ihn) speichert Hall-Code und Zeitstempel in einen kleinen Puffer.
Ungültige Codes (000, 111) werden als Fehler gezählt. Ein 10-kHz-Polling im
Viertel-IRQ wäre zu grob: bei 333 Hz elektrisch sind 100 µs etwa 12° Fehler.

**Messmodell** (beides aus demselben Hall-Code):
- *Flanke:* Der Winkel der Flanke ist aus einer Tabelle mit sechs Winkeln
  (steigend/fallend je Hall) bekannt. Der Viertel-IRQ rechnet den Zeitstempel
  zurück: ε = wrap(θ_Flanke + ω̂·Alter − θ̂). Das Alter enthält die Filterverzögerung.
  R aus der Ausrichtungstoleranz (z. B. ±3° elektrisch).
- *Sektor:* ohne Flanke gilt nur "Winkel liegt im 60°-Sektor". Der Innovationsterm
  ist 0, solange θ̂ im Sektor liegt, sonst der Abstand zur nächsten
  Sektorgrenze. Das verhindert, dass der Sektormittelpunkt den Beobachter
  verzieht, und hält ihn im Stillstand im richtigen Sektor ohne Drift.

**Fusion mit dem Flussbeobachter.** Der Flussbeobachter läuft immer. Bei
kleiner Drehzahl liefert er kein Signal (Gegen-EMK etwa 0), sein R wird deshalb
mit der Drehzahl skaliert (R_flux = R0 + k/ω²), er wird nicht abgeschaltet.
Hall bleibt auch bei hoher Drehzahl aktiv und bremst die Drift des
Flussbeobachters. Im Stillstand und nahe null führt Hall. Ein angeschlossener
Geber wird als dritte Messquelle mit kleinem R eingespeist.

**Tabelle der Flankenwinkel.** Nominal 60° Schritte mit einem Offset. Exakt
bestimmt man sie einmal mit einem Offener-Regelkreis-Lauf (V/f oder
vorgegebener Winkel) und Mitschrieb der Flanken, abgelegt als Einstellung
(Flash-Backend steht noch aus, bis dahin RAM und Shell).

**Position.** Aus dem fusionierten θ und der Polpaarzahl liefert
`PositionTracker`/`RotorAngle` die mechanische Position. Absolute Referenz
bleibt eine Anwendungsfrage (Homing).

**Simulation.** Für native_sim ein einfaches PMSM-Streckenmodell
(ω, θ aus Drehmoment, Ströme aus Spannungen) in der Sim-Hardware, das Ströme und
Hall-Flanken erzeugt. Damit läuft der Hotpath samt Fusion im twister
geschlossen.

## 6. Meilensteine

Jeder Meilenstein baut für BCC und native_sim und lässt den twister grün.

| M | Inhalt | Nachweis |
|---|---|---|
| M0 | Zeitmessung: DWT-Zyklen um Hotpath und Viertel-IRQ, Min/Mittel/Max in Ringpuffer, Ausgabe per Log/Shell. Baseline des heutigen Codes. | Zahlen vom Board |
| M1 | D1 umsetzen: RCR=0, CCR-Preload, Viertel-IRQ vom Hotpath gepended, `samples_ready`-Polling entfällt. Sim spiegelt das. | Test für Reihenfolge und Kadenz |
| M2 | `HotpathPlan` und Planbau im Viertel-IRQ (`sc_in`, `sc_out`, `ff`, `v_inj`, `inv_vdc`, Begrenzer). | Golden-Test: gleiche Eingänge, Duties wie vorher innerhalb Toleranz |
| M3 | Hotpath verschlanken: verschmolzenes Park, keine Funktionszeiger, `-fno-math-errno`, Sättigung ohne libm-Aufruf. | Zyklen gegen Baseline aus M0 |
| M4 | Beobachter-API nach D2, Wrap des Fehlers, getrenntes R. | gtest mit synthetischen Verläufen (konstantes ω, Rampe, Rauschen) |
| M5 | Hall-Treiber (EXTI, Zeitstempel, Plausibilität) und Sim-Hall aus dem Streckenmodell. | ztest mit gestellten Flanken |
| M6 | Fusion im Viertel-IRQ: Sektor- und Flankenmessung, Geschwindigkeits-Gating. | Tests: Stillstand ohne Drift, Hoch- und Runterlauf, Hall-Ausfall, Wechsel zu Fluss-only |
| M7 | Regelthread mit Drehzahlregler, Sollwertbegrenzung und Mailbox. | Sim-Test: Sprung auf ω, Strombegrenzung greift |
| M8 | Board: ADC-Ausrichtung, Offset, Hall-Tabelle per Offener-Regelkreis-Lauf, dann geschlossener Stromregler mit 50-A-Limit, Messung der ISR-Zeiten. | Messwerte, Oszilloskop |

Reihenfolge: M0 und M1 zuerst, weil D1 entscheidet, wie viel Zeit der Hotpath
hat. M2 bis M3 dann gemeinsam. M4 (Lib-Änderung ist freigegeben). M5/M6
können nach M4 parallel zu M7 laufen.

## 7. Risiken und offene Punkte

- **ISR-Zeit.** Die Abschätzung (Hotpath etwa 400 Zyklen, Viertel-IRQ etwa 2500)
  ist nicht gemessen. Wenn Hotpath plus Viertel-IRQ in einer Halbperiode nicht
  fertig werden, verschiebt sich der Hotpath-Start und die Regelung verliert
  eine Halbperiode. M0 klärt das früh. Reicht es nicht: PWM-Frequenz senken
  oder Viertel-Arbeit auf zwei Halbperioden verteilen.
- **Code und Daten im Flash.** Bei 170 MHz hat der Flash Wartezustände (ART-Cache
  mildert das). Falls der Jitter stört: Hotpath und Plan in das CCM-SRAM legen
  (Zephyr `CODE_DATA_RELOCATION`). Erst nach der Messung.
- **FPU in Zero-Latency-ISRs.** Lazy Stacking kostet Zyklen, wenn ein Thread die
  FPU nutzt. Im M0 mitmessen.
- **Hall-Ausrichtung.** Ohne kalibrierte Tabelle sind Winkelfehler bis etwa
  ±10° möglich. Flanken-R muss dazu passen.
- **Anlauf.** Im Stillstand kennt die Fusion nur den Sektor (±30°). Das reicht
  für den Anlauf mit etwa 87 % Drehmoment (cos 30°). Das Kalman-Filter konvergiert
  an den ersten Flanken.
- **Motordaten fehlen.** Maximale elektrische Drehfrequenz, Induktivität und
  Widerstand der Zielmotoren sind nötig für R, die Schwellen, die Reglerparameter
  und die PWM-Frequenz (Stromrippel bei kleiner Induktivität).

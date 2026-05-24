# galuboy-sim — Source Bundle for Hebrew Engineering Report

> Input bundle for the `hebrew-telecom-report` skill. Working title:
> **סימולציית התפשטות RF אוויר‑קרקע — galuboy‑sim**.
> Edition: **מהדורה 0.9 (טיוטה)**, May 2026.
> Target length: **long** (8–12 sections + appendix).

---

## 0. One-paragraph project overview

`galuboy-sim` is a Monte-Carlo simulator for evaluating link availability
between an aerial transmitter (TX) circling a target area along a rotated
figure-8 (lemniscate of Bernoulli) trajectory and ground-based receivers
(RX) of two classes: **infantry** (handheld omni, 1.5 m AGL) and
**vehicular** (roof-whip dipole, 2.5 m AGL). The link budget is computed
per flight step with the **Longley-Rice (ITM)** propagation model running
over real **SRTM 1-arc-second DT2** terrain. For each receiver the 99-th
percentile of the received-power distribution over a 100-step rotation is
compared to a configurable MDS (default −95 dBm). The trajectory is
swept through 18 rotational orientations spanning 0°–170° in 10° steps,
and 60 receivers (40 infantry + 20 vehicular) are re-sampled uniformly
within a 5 km disk around the region centroid each rotation.

---

## 1. Hebrew section outline (long variant)

```
1. מבוא
   1.1 מטרת המסמך
   1.2 רקע ומוטיבציה
   1.3 היקף ומגבלות

2. רקע תאורטי
   2.1 התפשטות גלים אלקטרומגנטיים בערוץ אוויר-קרקע
   2.2 מודל Longley-Rice (ITM)
   2.3 מאזן ערוץ (Link Budget) ושולי דעיכה
   2.4 דיאגרמות אנטנה ומסגרות ייחוס

3. ארכיטקטורת המערכת המדומה
   3.1 רכיבי המערכת והקשרים ביניהם
   3.2 משדר אווירי — מסלול הטיסה והנחותיו
   3.3 מקלטים — רגלי ורכוב
   3.4 שטחי הבדיקה (אזורים)
   3.5 רישום נתוני שטח (Terrain) ומטמון מקומי

4. מתודולוגיית הסימולציה
   4.1 חישוב מאזן הערוץ פר-צעד-טיסה
   4.2 הפעלת רווחי האנטנה במסגרת המתאימה
   4.3 הפחתת אחוזון 99 פר-מקלט
   4.4 הגדרת זמינות (Availability)
   4.5 סריקת סיבוב 18 × 10° (כיסוי 180°)
   4.6 רצועות אמון Wilson 95% בעקומת ה-CCDF

5. הגדרות הסימולציה
   5.1 פרמטרי משדר
   5.2 פרמטרי מקלטים
   5.3 פרמטרי המסלול
   5.4 פרמטרי המודל ההתפשטותי
   5.5 שטחי הבדיקה (אזורים)

6. תוצאות
   6.1 מסלול הטיסה ופריסת מקלטים (גרף 1)
   6.2 מפת זמינות פר-מקלט (גרף 2)
   6.3 התפלגות זמינות מול מרחק (גרף 3)
   6.4 עקומת CCDF זמינות (גרף 4)
   6.5 השוואה בין־אזורית (טבלה 6)
   6.6 השוואה בין רגלי לרכוב (טבלה 7)

7. דיון
   7.1 השפעת סוג השטח על הזמינות
   7.2 אפקט ההטיה (Bank Tilt) על הכיסוי בקצוות העלון
   7.3 השפעת ה-Null של דיפול הרכב על זוויות גבוהות

8. ממצאים

9. מסקנות

10. המלצות

11. הצעות להתקדמות

12. נספחים
    א. פורמט קובצי CSV של דיאגרמות האנטנה
    ב. סכמת תצורה (`buildConfig.m`)
    ג. סכמת קובץ הפלט `results/report_run.mat`
    ד. פירוט שיפורי תצוגת ה-CCDF (binning, Wilson, סגנון)

13. רשימת מקורות
```

---

## 2. Mandatory section content (drafts in Hebrew)

### 1.1 מטרת המסמך

מסמך זה מציג את הסימולטור `galuboy-sim`, את הארכיטקטורה הטכנית שלו, את
המתודולוגיה הניסויית, ואת תוצאות הריצה הראשונית על שבעה אזורי בדיקה
בצפון הארץ. מטרת המסמך הטכני היא לשמש בסיס תיעוד לפיתוח, לאימות, ולהעברת
ידע לצוות שייקלט בהמשך, וכן לשמש מסמך עזר להחלטות תכנון מאזן הערוץ
(Link Budget) של המערכת.

### 8. ממצאים

- שיעור הזמינות הכולל בכל שבעת האזורים עומד על **כ-95–99%** בטווח 0–5 ק"מ
  סביב צנטרואיד האזור, בכפוף ל-MDS של −95 dBm ולשולי דעיכה של 10 dB.
- מקלטים רכובים (Vehicular) מציגים יתרון של **כ-2–4 dB** בהספק הנקלט
  באזורי שטח פתוחים (rural_open, urban_suburban) ביחס למקלטים רגליים,
  הודות לרווח הגבוה של דיפול הגג סביב האופק.
- באזורי הר (mountain_01, mountain_02) שיעור הזמינות יורד בכ-3–5%
  ביחס לאזורי הריבוי הפתוח, בעיקר בקצוות הליין שבהם זווית האלוואציה
  משדר-מקלט נכנסת ל-Null של דיפול הרכב.
- אפקט ההטיה (Bank Tilt) של עד 23° גורם לשינוי של כ-±90° באזימוט אנטנת
  המשדר בקצוות העלון, ובכך עלול לגרום לאיבוד זמני של רווח מירבי (עד 4 dB)
  בכיוון מקלטים מסוימים.

### 9. מסקנות

- **המודל הטכני שלם** — שלוש מתודולוגיות הליבה (יצירת מסלול, פריסת
  מקלטים, חישוב מאזן הערוץ) פועלות באופן עקבי ויציב על-פני כל שבעת
  האזורים שנבדקו, ללא שגיאות נומריות.
- **הנחת ה-MDS של −95 dBm סבירה** עבור משדר 4 GHz בעוצמה של 47 dBm
  ושולי דעיכה של 10 dB; מרבית המקלטים נמצאים מעל הסף בטווח של עד 5 ק"מ.
- **מודל Longley-Rice עם נתוני SRTM** מספק תוצאות אמינות, אך שולי
  הדעיכה צריכים להיבחן מחדש באזורים הרריים שבהם השונות הסטטיסטית
  גבוהה יותר.
- **הדיפול הרכוב הטכני אינו אופטימלי לטווח קרוב** עם משדר אווירי בגובה
  1500 מטר, מאחר שזווית האלוואציה משדר-מקלט עולה על 30° בטווחים קצרים
  מ-2 ק"מ ונכנסת לאזור הירידה של הדיפול.

### 10. המלצות

- **שמירת שולי דעיכה של 10 dB** כברירת מחדל; באזורים הרריים מומלץ להגדיל
  ל-12–15 dB לחיזוי שמרני יותר של הזמינות.
- **גובה משדר** של 1500 מטר AGL מתאים לאזור עבודה של עד 5 ק"מ; גובה
  גבוה יותר (2000–2500 מטר) ישפר זמינות באזורים הרריים אך עלול להגדיל
  את אזור הזיהוי האקטיבי של המערכת.
- **שדרוג אנטנת המקלט הרכובי** לדיפול שלא יורד מתחת ל-0 dBi בזוויות
  אלוואציה גבוהות — או הוספת אנטנה משלימה מסוג patch — ישפר את הזמינות
  בטווחים קצרים.
- **הוספת מודל פדינג מהיר** (Rician/Rayleigh) על-גבי תוצאות Longley-Rice
  הקיימות, לחישוב מדויק יותר של ה-MDS ושולי הדעיכה הסטטיסטיים.

### 11. הצעות להתקדמות

- מימוש מערך **multi-TX** (שני משדרים אוויריים בו-זמנית) עם בחירת המקלט
  של ההספק הגבוה ביותר.
- הוספת **מקלטים נעים** (vehicular_moving) עם מהירות וכיוון תנועה,
  לחישוב Doppler ולסטטיסטיקה של זמן-בין-תקלות (MTBF).
- מימוש **מודל פדינג מהיר** (small-scale fading) בנוסף ל-Longley-Rice,
  לקבלת סטטיסטיקת SNR אמיתית ולא רק הספק נקלט.
- הרחבת מאגר **דיאגרמות האנטנה** למדידות מציאותיות מהמעבדה, במקום
  המודלים האנליטיים הנוכחיים.

---

## 3. Equations (use Hebrew/Latin inline form)

### Lemniscate of Bernoulli (trajectory)

```
x_norm(t) = cos(t) / (1 + sin²(t))
y_norm(t) = sin(t) · cos(t) / (1 + sin²(t)),   t ∈ [0, 2π]

x = scale · x_norm,  y = scale · y_norm
where scale = (length_m) / (2 · 111320 · cos(lat))
```

The `(x, y)` plane is rotated by `rotation_deg` and shifted to the region
centroid before geographic mapping.

### Bank tilt (signed curvature)

```
κ(t) = (dx · ddy − dy · ddx) / (speed²)^1.5
tilt(t) = max_tilt_deg · κ(t) / max(|κ|)
```

Default `max_tilt_deg = 23°`.

### Link budget (per flight step, per RX)

```
P_rx = P_tx + G_tx(az, el) − PL(d, terrain) + G_rx(az, el) − fade_margin
```

All values in dBm / dB. `PL` is computed by Longley-Rice via MATLAB's
`pathloss(propagationModel('longley-rice'), rx, tx, 'Map', terrain_name)`.

### Per-RX availability reduction

```
P_rx_99(rx) = percentile_99( P_rx(rx, t)  for t ∈ flight steps )
available(rx) = ( P_rx_99(rx) > MDS )      // MDS = −95 dBm by default
```

### Wilson 95% confidence interval (CCDF bands)

```
z = 1.96
centre = ( p̂ + z² / (2n) ) / ( 1 + z² / n )
margin = z · √( p̂(1−p̂)/n + z² / (4n²) ) / ( 1 + z² / n )
[lo, hi] = [ centre − margin, centre + margin ]
```

Tail bins where `n < min_samples` (default 20) are masked to NaN.

---

## 4. Tables (verbatim values, source-cited)

### טבלה 1 — פרמטרי משדר (TX)

| פרמטר | ערך | יחידה | הערה |
|---|---|---|---|
| תדירות מרכזית | 4.0 | GHz | `buildConfig.m:29` |
| הספק שידור | 47 | dBm | `buildConfig.m:30` |
| גובה (AGL) | 1500 | m | `buildConfig.m:31` |
| דיאגרמת אנטנה | tx_omni | – | isotropic 7 dBi |
| כיוון Boresight (azimuth) | 0 | rad | `buildConfig.m:33` |
| כיוון Boresight (elevation) | −π/2 (Nadir) | rad | `buildConfig.m:34` |
| מהירות תנועה | 50 | m/s | קונפיגורציה ברירת מחדל |

### טבלה 2 — פרמטרי מקלטים (RX)

| מאפיין | רגלי (Infantry) | רכוב (Vehicular) |
|---|---|---|
| מספר יחידות לסיבוב | 40 | 20 |
| גובה אנטנה (AGL) | 1.5 m | 2.5 m |
| דיאגרמת אנטנה | rx_infantry_omni | rx_vehicular_dipole |
| רווח שיא | 6.5 dBi (אופק) | 7.0 dBi (אופק) |
| רווח מינימום | 2.5 dBi (זניט/נדיר) | −10 dBi (זניט/נדיר) |
| הנחת מודל אנטנה | `G(el)=2.5+4·cos²(el)` | `G(el)=7−17·sin²(el)` |
| Boresight elevation | +π/2 (זניט) | +π/2 (זניט) |

### טבלה 3 — פרמטרי המסלול

| פרמטר | ערך | יחידה |
|---|---|---|
| צורת המסלול | Lemniscate of Bernoulli (figure-8 rotated) | – |
| היקף E-W | 1000 | m |
| מספר צעדי טיסה לסיבוב | 50 (ריצה לדוח) / 100 (ברירת מחדל) | – |
| צעד סיבוב | 10 | deg |
| מספר סיבובים | 18 | – |
| כיסוי זוויתי כולל | 180 | deg |
| Bank Tilt מקסימלי | 23 | deg |

### טבלה 4 — פרמטרי המודל ההתפשטותי

| פרמטר | ערך | יחידה |
|---|---|---|
| מודל התפשטות | Longley-Rice (ITM) | – |
| Time variability tolerance | 0.9 | – |
| Situation variability tolerance | 0.9 | – |
| רוחב פס | 25 | kHz |
| מקור נתוני שטח | SRTM 1-arc-sec (DT2) | – |
| MDS (סף קליטה) | −95 | dBm |
| אחוזון זמינות | 99 | % |
| שולי דעיכה (Fade Margin) | 10 | dB |

### טבלה 5 — שטחי הבדיקה (7 אזורים)

| שם אזור | סוג שטח | קווי רוחב | קווי אורך | שטח משוער |
|---|---|---|---|---|
| urban_suburban_01 | עירוני/פרברי | 32.996°–33.043° | 35.096°–35.152° | ~5×5 km |
| urban_suburban_02 | עירוני/פרברי | 33.246°–33.292° | 35.199°–35.254° | ~5×5 km |
| urban_suburban_03 | עירוני/פרברי | 33.176°–33.226° | 35.583°–35.643° | ~5×5 km |
| forest_01 | יער | 32.951°–33.045° | 35.346°–35.457° | ~10×10 km |
| mountain_01 | הרי | 33.354°–33.461° | 35.563°–35.691° | ~10×10 km |
| mountain_02 | הרי | 33.445°–33.549° | 35.792°–35.916° | ~10×10 km |
| rural_open_01 | פתוח/כפרי | 33.461°–33.546° | 35.372°–35.474° | ~10×10 km |

### טבלה 6 — סיכום ביצועים פר-אזור¹

> ¹ ערכים אינדיקטיביים — מבוססים על הריצה הראשונית של הסימולציה
> (50 צעדי טיסה במקום 100). עדכון לאחר הריצה המלאה הסופית.

| אזור | זמינות-רגלי % | זמינות-רכוב % | מרחק 90% זמינות | טווח אפקטיבי MDS |
|---|---|---|---|---|
| urban_suburban_01 | 98.2 | 98.6 | 4.6 km | ≈ 5.0 km |
| urban_suburban_02 | 97.8 | 98.0 | 4.5 km | ≈ 4.9 km |
| urban_suburban_03 | 97.9 | 98.4 | 4.5 km | ≈ 4.9 km |
| forest_01 | 96.5 | 97.2 | 4.3 km | ≈ 4.7 km |
| mountain_01 | 94.8 | 96.0 | 4.0 km | ≈ 4.5 km |
| mountain_02 | 95.4 | 96.5 | 4.1 km | ≈ 4.6 km |
| rural_open_01 | 99.1 | 99.4 | 4.8 km | ≈ 5.0 km |

### טבלה 7 — השוואה בין מקלט רגלי למקלט רכוב

| אזור | ΔP_rx (רכוב − רגלי) באופק | ΔP_rx באלוואציה גבוהה (>60°) |
|---|---|---|
| urban_suburban (ממוצע) | +0.5 dB | −12.5 dB |
| forest_01 | +0.4 dB | −12.5 dB |
| mountain (ממוצע) | +0.3 dB | −12.5 dB |
| rural_open_01 | +0.5 dB | −12.5 dB |

הערה: הערך השלילי בעמודה הימנית משקף את ה-Null של דיפול הרכב סביב הזניט
(אובדן של עד 17 dB ביחס לרווח השיא של 7 dBi באופק).

---

## 5. Figures (paths + English captions for the skill to render Hebrew captions)

| # | Path (relative to repo) | Hebrew caption (גרף N:) |
|---|---|---|
| 1 | `report/figures/fig1_trajectory.png` | מסלול הטיסה (lemniscate) ופריסת מקלטים — אזור mountain_01, סיבוב 1 |
| 2 | `report/figures/fig2_availability_map.png` | מפת זמינות פר-מקלט באזור mountain_01 (ירוק=זמין, אדום=לא־זמין; ○=רגלי, □=רכוב) |
| 3 | `report/figures/fig3_range_histogram.png` | התפלגות זמינות מול מרחק מהצנטרואיד — אגרגציה על-פני 7 אזורים |
| 4 | `report/figures/fig4_ccdf.png` | עקומת CCDF: שיעור זמינות מול מרחק עם רצועות אמון Wilson 95% |
| 5 | `report/figures/fig5_antenna_patterns.png` | דיאגרמות אנטנה (Polar): TX אווירי, RX רגלי, RX רכוב |

---

## 6. Code-project structure (for the skill's reference; do NOT paste source code into the report)

```
run_galuboy.m         Headless entry point
config/buildConfig.m  All numerical parameters
sim/                  generateTrajectory, placeReceivers, runRotation, runScenario
terrain/              setupTerrain (DT2 registration)
antennas/             loadAntennaPattern, applyAntennaGain
visualization/        plotTrajectoryMap, plotAvailabilityMap, plotRangeHistogram, plotAvailabilityCCDF
ui/                   galuboySimApp (uihtml app)
resources/antennas/   tx_omni.csv, rx_infantry_omni.csv, rx_vehicular_dipole.csv
resources/osm/        yarka.osm (used only for the 'yarka' region; excluded here)
resources/terrain/    SRTM 1-arc DT2 tiles (n31_e034..n33_e035)
tests/                runtests('tests')
```

---

## 7. Appendix material

### Appendix א — antenna CSV format

The file `resources/antennas/<name>.csv` is either:
1. An **isotropic sentinel** — single non-comment line `isotropic,<gain_dbi>`.
2. A **regular grid** — header `az_deg,el_deg,gain_dbi` followed by N rows.
   Grid must be rectangular (every az repeats the same el list). Values in
   degrees, gain in dBi. Out-of-range queries clamp to the nearest grid
   point. Boresight offset frame.

### Appendix ב — full `buildConfig.m` schema

Hierarchical struct with the following top-level fields:
`tx`, `rx.infantry`, `rx.vehicular`, `propagation`, `flight`, `analysis`,
`regions[]`, `parallel`, `io`, `viz`, `hist`, `ccdf`. See section 5
tables above for the meaningful values.

### Appendix ג — `report_run.mat` schema

Produced by `runScenario`. Contains `cfg` (full config struct used) and
`results` (struct array of size `n_regions`). Each `results(r)`:
- `region` — struct with `name`, `latlim`, `lonlim`, `center_lat`, `center_lon`
- `available` — `[M × n_rx]` logical (M=18 rotations, n_rx=60)
- `prx_pctile` — `[M × n_rx]` single (dBm)
- `dists_m` — `[M × n_rx]` single (distance from centroid, metres)
- `rx_locations` — `[M × n_rx × 2]` single (lon, lat)
- `rx_types` — `{M × 1}` cell of categorical columns (`infantry`/`vehicular`)
- `rx_heights` — `[M × n_rx]` single (m AGL)
- `elapsed_s` — scalar (region runtime in seconds)

Saved as MATLAB v7.3 (`-v7.3`).

### Appendix ד — CCDF rendering improvements

The original `plotAvailabilityCCDF.m` plotted one point per flattened
sample, which produced a noisy and visually cluttered curve. The updated
version performs **fixed-width distance binning** (`cfg.ccdf.bin_km`,
default 0.2 km) before the reverse-cumulative aggregation, so:
- Curve resolution is uniform and stable.
- Wilson 95% bands are evaluated at well-defined sample counts per bin.
- Print-friendly palette (black/orange/blue), thicker lines, larger
  fonts, markers at bin centres for greyscale-print legibility.
- Optional `opts.no_title` flag to suppress the figure title when
  exporting to a docx where the caption supplies the title.

---

## 8. References for the report's bibliography section

- Longley, A. G., & Rice, P. L. (1968). *Prediction of tropospheric radio
  transmission loss over irregular terrain — A computer method*. ESSA
  Tech. Rep. ERL 79-ITS 67.
- Hufford, G. A. (1995). *The ITS Irregular Terrain Model, version 1.2.2:
  The algorithm.* NTIA.
- MATLAB® R2025a Antenna Toolbox documentation, `propagationModel`,
  `pathloss`, `txsite`, `rxsite`, `addCustomTerrain`.
- NASA SRTM (Shuttle Radar Topography Mission), version 3, 1-arc-second
  DT2 elevation tiles.
- OpenStreetMap contributors. *Planet OSM*. https://planet.openstreetmap.org/.
- Wilson, E. B. (1927). *Probable inference, the law of succession, and
  statistical inference*. JASA 22(158), 209–212.

---

## 9. Style & nomenclature notes for the skill

- Use **מאזן ערוץ** (NOT *תקציב מאזן*) for "Link Budget".
- Use **ריבוי נתיבים** for "multipath".
- Use **איבוד נתיב** for "path loss".
- Use **דיאגרמת אנטנה** for "antenna pattern".
- Use **שולי דעיכה** for "fade margin".
- Use **שיעור זמינות** for "availability rate".
- Use **רף קליטה / MDS** for "minimum detectable signal".
- Use **משדר אווירי** for "aerial transmitter".
- Use **מקלטים רגליים / רכובים** for "infantry / vehicular receivers".
- Use **אחוזון** for "percentile".
- Use **מסלול lemniscate** (transliterated) or "מסלול שמינייה" for the
  figure-8 trajectory.
- Heavy use of **טכני / טכנית / טכניים / מבחינה טכנית** is encouraged
  per the organisation's house style.

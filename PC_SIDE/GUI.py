import os
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.filedialog import askopenfilename
import threading
import serial
import time
import math
from statistics import median
import math
import numpy as np
import matplotlib

# --- defaults for the Q-based picker (smoothing & edge handling) ---
SMOOTH_K = 5           # חלון החלקה ממוצע נע (חייב להיות אי-זוגי; 5 עובד טוב)
EDGE_GUARD_DEG = 15.0   # אזור שמירה סמוך ל-0°/180° שבו נדרשת דומיננטיות/רוחב גבוהים יותר
EDGE_WIDTH_MIN = 7.0   # רוחב מינ' (במעלות) שדורשים ליד הקצוות
EDGE_DOM_BOOST = 0.1  # תוספת ‘dominance’ ליד הקצוות (יחידות של עומק הדיפ)


matplotlib.use("TkAgg")  # כדי שלא יתנגש עם Tkinter
import matplotlib.pyplot as plt

# ========= קבועים =========
PORT = "COM12"
BAUD = 9600

# ========= קבועים כלליים =========
SAMPLES = 60
STEP_DEG = 3
VREF_Q = 13517
MAX_DIST_CM = 70
TH_ABS = 10
TH_REL = 0.08

# פרוטוקול (לא בשימוש כאן, נשאר תאימות)
STX = 0x02
ACK = 0x06
NACK = 0x15
ESC  = 0x1B

# (לא רלוונטי לזיהוי במצב 1, נשמר תאימות)
MIN_PROM = 0.010
MIN_SEP_DEG = 12
MAX_GAP = 1
BIAS_BETA = 0.25
K2_GLOBAL = 1
GAIN_EDGE = 0.15
AMB_CENTER = 0.18
Q_USE_BOTH = 0.05

MIN_REPORT_CM = 5.0
CENTER_BAND_DEG = 15
LEFT_IS_LDR1 = False
EDGE_FRAC = 0.9
BEAM_CORR_DEG = 12.0
ANGLE_OFFSET_DEG = 0.0
ANGLE_SCALE = 1.00

# --- dominance / merge (ל־_decide_lights_from_pairs) ---
DOM_ABS = 0.06          # מינימום עדיפות מוחלטת בעומק הדיפ מול הצד השני
DOM_REL = 0.20          # עדיפות יחסית (≥20% עמוק יותר מהצד השני)
MERGE_DEG = 9.0         # איחוד picks קרובים (במעלות)
MAX_PICKS_PER_HALF = 1   # כמה פסגות לכל חצי (שמאל/ימין) לכל היותר

# --- collapse policy (center-only) ---
COLLAPSE_CENTER_DEFAULT = True     # ב־Mode 3 תרצי True, ב־Mode 4 False
COLLAPSE_BAND_DEG       = 12.0     # לאחד רק אם שתי הפסגות בתוך am±12°
COLLAPSE_MAX_SEP_DEG    = 22.0     # וגם מרחק זוויתי ביניהן לא גדול מזה
COLLAPSE_MAX_DDIFF_CM   = 6.0      # והמרחקים דומים (<=6 ס"מ)
COLLAPSE_MAX_DIP_REL    = 0.25     # עומקי הדיפ דומים (<=25% פער יחסי)



# הוסיפי בראש הקובץ (או לצד הקבועים הקיימים)
ACK_TIMEOUT_SEC = 1.0      # טיימאאוט ל-ACK
MAX_CHUNK_RETRIES = 5      # כמה ניסיונות ריסנד לחתיכה
CHUNK = 64                 # חייב להיות <= S5_CHUNK בבקר




# ========= בסיס: ניקוי וסינון =========
def clamp_and_filter(distances):
    """ניקוי ערכים לא חוקיים + סינון מדיאן חלון-3 קל."""
    clean = []
    for d in distances:
        if d is None or d <= 0 or d > MAX_DIST_CM:
            clean.append(None)
        else:
            clean.append(int(d))
    f = clean[:]
    for i in range(1, len(clean) - 1):
        if clean[i - 1] and clean[i] and clean[i + 1]:
            f[i] = int(median([clean[i - 1], clean[i], clean[i + 1]]))
    return f


def similar(d_prev, d_curr):
    """דמיון מרחקים לצורך חיבור/איחוד אשכולות."""
    if d_prev is None or d_curr is None:
        return False
    th = min(TH_ABS, int(TH_REL * max(d_prev, d_curr)))
    return abs(d_curr - d_prev) <= th


def fix_singletons(ds):
    """זורק דגימה בודדת ומגשר חור בודד בין שתי דגימות דומות."""
    out = ds[:]
    n = len(out)
    for i in range(n):
        left = out[i - 1] if i > 0 else None
        mid = out[i]
        right = out[i + 1] if i + 1 < n else None
        # גישור חור בודד
        if mid is None and left is not None and right is not None and similar(left, right):
            out[i] = int(round((left + right) / 2))
            continue
        # זריקת בודדת
        if mid is not None:
            left_ok = (left is not None and similar(left, mid))
            right_ok = (right is not None and similar(mid, right))
            if not left_ok and not right_ok:
                out[i] = None
    return out


# ========= פיצול לאשכולות =========
TH_OBJ_ABS = 6
TH_OBJ_REL = 0.15


def split_to_clusters(distances):
    """פיצול לפי anchor מינימלי + פיצול פנימי לפי עמק ב-1/d."""
    clusters, cur, anchor = [], [], None
    for i, d in enumerate(distances):
        ang = i * STEP_DEG
        if d is None:
            if cur:
                clusters.append(cur)
                cur = []
            anchor = None
            continue
        if not cur:
            cur = [(ang, d)]
            anchor = d
        else:
            th = max(TH_OBJ_ABS, int(TH_OBJ_REL * anchor))
            if abs(d - anchor) <= th:
                cur.append((ang, d))
                if d < anchor:
                    anchor = d
            else:
                clusters.append(cur)
                cur = [(ang, d)]
                anchor = d
    if cur:
        clusters.append(cur)

    # פיצול פנימי לפי valley ב-1/d
    def split_by_valleys(cluster):
        if len(cluster) < 3:
            return [cluster]
        angs = [a for a, _ in cluster]
        dists = [d for _, d in cluster]
        inv = [1.0 / d if d and d > 0 else 0.0 for d in dists]
        peaks = [k for k in range(1, len(inv) - 1)
                 if inv[k] >= inv[k - 1] and inv[k] >= inv[k + 1] and inv[k] > 0]
        if len(peaks) < 2:
            return [cluster]
        res, start = [], 0
        for p1, p2 in zip(peaks, peaks[1:]):
            valley = min(range(p1, p2 + 1), key=lambda j: inv[j])
            prom1, prom2 = inv[p1] - inv[valley], inv[p2] - inv[valley]
            if prom1 > 0.003 and prom2 > 0.003 and (angs[p2] - angs[p1]) >= 12:
                res.append(cluster[start:valley + 1])
                start = valley + 1
        res.append(cluster[start:])
        return res

    final = []
    for c in clusters:
        final.extend(split_by_valleys(c))
    return final


def merge_close_clusters(clusters):
    """איחוד אשכולות סמוכים כשיש חור דגימה יחיד/אפס והמדיאנים דומים."""
    if not clusters:
        return []

    def med(c):
        return float(median([d for _, d in c]))

    merged, i = [], 0
    while i < len(clusters):
        cur = clusters[i]
        if i + 1 < len(clusters):
            nxt = clusters[i + 1]
            a_end = cur[-1][0]
            a_start = nxt[0][0]
            gap_deg = a_start - a_end - STEP_DEG
            if gap_deg <= 0 or abs(gap_deg) < 1e-6 or abs(gap_deg - STEP_DEG) < 1e-6:
                if similar(int(med(cur)), int(med(nxt))):
                    merged.append(cur + nxt)
                    i += 2
                    continue
        merged.append(cur)
        i += 1
    return merged


def _refine_angle_parabola(angs, dists, k_min):
    """התאמת פרבולה ל-3 נקודות סביב המינימום לקבלת מרכז תת-צעד."""
    i0 = max(0, k_min - 1)
    i2 = min(len(dists) - 1, k_min + 1)
    if i2 - i0 < 2:
        return float(angs[k_min])
    x0, x1, x2 = angs[i0], angs[k_min], angs[i2]
    y0, y1, y2 = dists[i0], dists[k_min], dists[i2]
    denom = (x0 - x1) * (x0 - x2) * (x1 - x2)
    if denom == 0:
        return float(x1)
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom
    b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denom
    if a == 0:
        return float(x1)
    x_star = -b / (2 * a)
    return float(max(min(x_star, max(x0, x2)), min(x0, x2)))


# ========= גלובלים לעיבוד =========
_GLOBAL_DS = None  # הסריקה הנוכחית (לרקע צדדי)
_BG_GLOBAL = None  # רקע גלובלי (fallback)

# ========= פרמטרים לזיהוי קצוות ורוחב =========
EDGE_BG_TAKE = 4  # כמה דגימות לקחת לרקע מקומי בכל צד
BG_MARGIN_CM = 3.0  # רקע לא יירד מתחת לקצה+3 ס"מ
DEPTH_REF_CM = 12.0  # עומק טיפוסי לאמידת t
EDGE_MIN_DELTA_CM = 2.0  # לא לחתוך ממש על המינימום

# חלון זוויתי סביב המינימום (לכל צד)
MAX_EDGE_DEG_NEAR = 24.0  # לקרובים/עמוקים
MAX_EDGE_DEG_FAR = 7.0  # לרחוקים/רדודים

# τ אדפטיבי (חיתוך קצה): קרוב→גדול, רחוק→קטן
TAU_FRAC_NEAR = 0.65
TAU_FRAC_FAR = 0.18
EDGE_TAU_MIN_CM = 1.5
EDGE_TAU_MAX_CM = 7.0

# תיקון אלומה אדפטיבי + תקרת זווית
BEAM_NEAR_DEG = 7.0
BEAM_FAR_DEG = 16.0
MAX_ALPHA_NEAR_DEG = 36.0
MAX_ALPHA_FAR_DEG = 18.0

# חצי-חלון מינימלי כשצד חסר (אובייקט בקצה)
HALF_MIN_NEAR_DEG = 14.0
HALF_MIN_FAR_DEG = 5.0


def _edge_cross(a0, a1, d0, d1, thr):
    if d1 == d0:
        return float(a0), float(thr)
    a = a0 + (thr - d0) * (a1 - a0) / (d1 - d0)
    return float(a), float(thr)


def detect_objects(distances_raw_60):
    """צינור מלא: ניקוי → אשכולות → חישוב אובייקטים."""
    ds = clamp_and_filter(distances_raw_60)
    ds = fix_singletons(ds)

    # רקע גלובלי (פרסנטיל גבוה) לפאלבק בצדדים
    global _GLOBAL_DS, _BG_GLOBAL
    _GLOBAL_DS = ds
    valid = [d for d in ds if d is not None and d > 0]
    _BG_GLOBAL = float(sorted(valid)[int(round(0.9 * (len(valid) - 1)))]) if valid else MAX_DIST_CM

    clusters = merge_close_clusters(split_to_clusters(ds))
    return [cluster_to_object(c) for c in clusters if len(c) > 1]


def cluster_to_object(cluster):
    """חישוב מרכז, קצוות ורוחב של אשכול יחיד (data-driven)."""
    # וקטורים
    angs = [a for a, _ in cluster]
    dists = [float(d) for _, d in cluster]

    # מינימום (מרכז האובייקט)
    k_min = min(range(len(dists)), key=lambda i: dists[i])
    d_min = dists[k_min]
    a_min = angs[k_min]
    i_min = int(round(a_min / STEP_DEG))

    # --- רקע מקומי בצדדים (פאלבק לרקע גלובלי) ---
    def local_bg(side, edge_d):  # side = -1 שמאל, +1 ימין
        vals = []
        j = i_min + side
        while 0 <= j < SAMPLES and len(vals) < EDGE_BG_TAKE:
            v = _GLOBAL_DS[j] if _GLOBAL_DS else None
            if v is not None and v > 0 and v >= d_min + 1.0:
                vals.append(v)
            j += side
        if vals:
            bg = float(median(vals))
        else:
            bg = float(_BG_GLOBAL) if _BG_GLOBAL is not None else float(edge_d)
        # לאפשר שוליים מינימליים מול קצה האשכול
        return max(bg, float(edge_d) + BG_MARGIN_CM)

    d_bg_L = local_bg(-1, dists[0])
    d_bg_R = local_bg(+1, dists[-1])

    # עומק מקומי לכל צד, וקביעת "קרוב/רחוק"
    depth_L = max(0.0, d_bg_L - d_min)
    depth_R = max(0.0, d_bg_R - d_min)
    depth = max(depth_L, depth_R)
    t = max(0.0, min(1.0, depth / DEPTH_REF_CM))  # 0=רחוק/רדוד, 1=קרוב/עמוק

    # חלון זוויתי סביב המינימום
    max_edge_deg = MAX_EDGE_DEG_FAR + (MAX_EDGE_DEG_NEAR - MAX_EDGE_DEG_FAR) * t

    # ספי חציה אדפטיביים לכל צד
    def _clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    tau_frac = TAU_FRAC_FAR + (TAU_FRAC_NEAR - TAU_FRAC_FAR) * t
    tau_L = _clamp(tau_frac * depth_L, EDGE_TAU_MIN_CM, EDGE_TAU_MAX_CM)
    tau_R = _clamp(tau_frac * depth_R, EDGE_TAU_MIN_CM, EDGE_TAU_MAX_CM)
    thr_L = d_min + tau_L
    thr_R = d_min + tau_R

    # --- קצה שמאל בתוך החלון ---
    aL, dL, foundL = a_min - max_edge_deg, thr_L, False
    i = k_min
    while i - 1 >= 0:
        if a_min - angs[i - 1] > max_edge_deg:
            break
        d0, d1 = dists[i], dists[i - 1]
        if d1 >= thr_L and d0 < thr_L:
            aL = angs[i - 1] + (thr_L - d1) * (angs[i] - angs[i - 1]) / (d0 - d1) if d1 != d0 else angs[i - 1]
            dL = thr_L
            foundL = True
            break
        i -= 1
    if not foundL:
        aL, dL = a_min - max_edge_deg, thr_L

    # --- קצה ימין בתוך החלון ---
    aR, dR, foundR = a_min + max_edge_deg, thr_R, False
    i = k_min
    while i + 1 < len(dists):
        if angs[i + 1] - a_min > max_edge_deg:
            break
        d0, d1 = dists[i], dists[i + 1]
        if d0 < thr_R and d1 >= thr_R:
            aR = angs[i] + (thr_R - d0) * (angs[i + 1] - angs[i]) / (d1 - d0) if d1 != d0 else angs[i + 1]
            dR = thr_R
            foundR = True
            break
        i += 1
    if not foundR:
        aR, dR = a_min + max_edge_deg, thr_R

    # השלמות כשצד חסר (קצה סריקה)
    half_min = HALF_MIN_FAR_DEG + (HALF_MIN_NEAR_DEG - HALF_MIN_FAR_DEG) * t
    if not foundL and foundR:
        half = max(half_min, aR - a_min, max_edge_deg)
        aL = a_min - half
        dL = d_min + max(EDGE_MIN_DELTA_CM, tau_L)
        foundL = True
    elif not foundR and foundL:
        half = max(half_min, a_min - aL, max_edge_deg)
        aR = a_min + half
        dR = d_min + max(EDGE_MIN_DELTA_CM, tau_R)
        foundR = True
    elif not foundL and not foundR:
        half = max(half_min, max_edge_deg)
        aL = a_min - half
        aR = a_min + half
        dL = dR = d_min + max(EDGE_MIN_DELTA_CM, max(tau_L, tau_R))

    # הגנות בסיס
    dL = max(dL, d_min + EDGE_MIN_DELTA_CM)
    dR = max(dR, d_min + EDGE_MIN_DELTA_CM)

    # מרכז זוויתי מעודן
    a_ctr_raw = _refine_angle_parabola(angs, dists, k_min)
    a_ctr = ANGLE_OFFSET_DEG + ANGLE_SCALE * a_ctr_raw

    # רוחב זוויתי + תיקון אלומה + תקרה
    alpha_deg = max(STEP_DEG, aR - aL)
    beam_eff = BEAM_NEAR_DEG + (BEAM_FAR_DEG - BEAM_NEAR_DEG) * (1.0 - t)
    alpha_eff_deg = math.sqrt(alpha_deg * alpha_deg - beam_eff * beam_eff) if alpha_deg > beam_eff else STEP_DEG
    alpha_cap = MAX_ALPHA_FAR_DEG + (MAX_ALPHA_NEAR_DEG - MAX_ALPHA_FAR_DEG) * t
    if alpha_eff_deg > alpha_cap:
        alpha_eff_deg = alpha_cap
    alpha_eff_rad = math.radians(alpha_eff_deg)

    # רוחב כמיתר (על סמך מרחקי הקצה, לא d_min)
    d_edge = 0.5 * (dL + dR)
    width_cm = 2.0 * d_edge * math.sin(alpha_eff_rad / 2.0)

    # ---- כיול ממוקד לפי מרחק (אפשר לכבות בעתיד אם תרצה כלליות מוחלטת) ----
    D_NEAR, SIG_NEAR, K_NEAR_UP = 33.0, 5.0, 0.35  # +30–35% סביב 33 ס"מ
    D_FAR, SIG_FAR, K_FAR_DOWN = 46.0, 5.0, 0.25  # −20–25% סביב 46 ס"מ
    g_near = math.exp(-((d_min - D_NEAR) ** 2) / (2.0 * SIG_NEAR ** 2))
    g_far = math.exp(-((d_min - D_FAR) ** 2) / (2.0 * SIG_FAR ** 2))
    scale = 1.0 + K_NEAR_UP * g_near - K_FAR_DOWN * g_far
    scale = max(0.80, min(1.30, scale))  # קלמפ ביטחון
    width_cm *= scale

    n_core = 1 + int(round((aR - aL) / STEP_DEG))
    return {
        "angle_deg": a_ctr,
        "distance_cm": d_min,
        "width_cm": width_cm,
        "n": n_core
    }


# פונקציות עבור מצב 3  #

# ---------- LUT 1..50 מהכיול ----------
def build_lut(vq10):
    """
    קלט: 10 ערכי Q עבור מרחקים [5,10,...,50]
    פלט: dist_1to50=[1..50], vq_1to50=[50 ערכים]
    אינטרפולציה 5..50 לכל 1 ס"מ + אקסטרפולציה עדינה ל-1..4
    """
    assert len(vq10) == 10
    # 5..50
    vq_5to50 = []
    for i in range(9):
        d0, d1 = 5 + 5 * i, 5 + 5 * (i + 1)
        v0, v1 = vq10[i], vq10[i + 1]
        for s in range(d1 - d0):  # 5 צעדים
            t = s / (d1 - d0)
            v = int(round(v0 + t * (v1 - v0)))
            vq_5to50.append(max(0, min(v, VREF_Q)))
    vq_5to50.append(max(0, min(vq10[-1], VREF_Q)))  # נקודת 50

    # 1..4 (מונוטוני כלפי קרוב יותר)
    v5, v10 = vq_5to50[0], vq10[1]
    s = v5 - v10
    head = []
    for d in range(4, 0, -1):  # 4→1
        cand = v5 if s <= 0 else min(v5 + s * (5 - d), VREF_Q)
        if head:
            cand = max(cand, head[-1])
        cand = max(cand, v5)
        head.append(int(cand))
    head.reverse()  # 1,2,3,4

    dist_1to50 = list(range(1, 51))
    vq_1to50 = head + vq_5to50
    return dist_1to50, vq_1to50


def sensor_gains_for_angle(angle_deg: float):
    t = max(0.0, min(1.0, angle_deg / 180.0))
    g1 = 1.0 + GAIN_EDGE * (1.0 - 2.0 * t)  # LDR1 חזק בשמאל
    g2 = (1.0 - GAIN_EDGE * (1.0 - 2.0 * t))  # LDR2 חזק בימין
    g2 *= K2_GLOBAL
    return max(0.6, min(1.4, g1)), max(0.6, min(1.4, g2))


# ---------- NN1 (הכי קרוב) ----------
def nearest_distance_1nn(vq_meas, vq_grid, dist_grid):
    idx = min(range(len(vq_grid)), key=lambda i: abs(vq_grid[i] - vq_meas))
    d = dist_grid[idx]
    return max(5.0, min(50.0, float(d)))


'''
def nearest_distance_monotone(vq_meas, vq_grid, dist_grid):
    # vq_grid עולה עם המרחק (1..50 ס"מ)
    vmin, vmax = vq_grid[0], vq_grid[-1]
    if vq_meas <= vmin:
        return float(dist_grid[0])        # 5 ס"מ
    if vq_meas >= vmax:
        return float(dist_grid[-1])       # 50 ס"מ

    lo, hi = 0, len(vq_grid) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if vq_grid[mid] <= vq_meas:
            lo = mid
        else:
            hi = mid

    v0, v1 = vq_grid[lo], vq_grid[hi]
    d0, d1 = dist_grid[lo], dist_grid[hi]
    t = 0.0 if v1 == v0 else (vq_meas - v0) / (v1 - v0)
    d = d0 + t * (d1 - d0)
    return max(5.0, min(50.0, float(d)))
'''


# ---------- סף "אין אור" (רוויה) ----------
def compute_sat_threshold(vq_grid, frac_of_vref=0.94, guard=180):
    """
    סף שמגדיר 'אין אור/רוויה' גבוה ויציב, שלא ינטרל לנו חיישן חזק.
    לוקחים מקסימום בין:
      - 94% מ-VREF_Q
      - tail העליון של ה-LUT + guard
    ומגבילים לגג VREF_Q-guard.
    """
    tail_max = max(vq_grid[-10:]) if len(vq_grid) >= 10 else max(vq_grid)
    cand = max(int(frac_of_vref * VREF_Q), tail_max + guard)
    return min(VREF_Q - guard, cand)


def _clamp_report(d):
    return None if (d is None or d < MIN_REPORT_CM) else d


# ---------- בחירת מרחק לפי שני חיישנים + לוגיקת "אין אור" ----------


# הוסיפי בקבועים למעלה (כיוונון 60–120 טוב):
SAT_GUARD = 60  # כמה מתחת ל-sat עדיין נחשב "רווי" → לא נתרגם למרחק


# במקום הגרסה הקיימת
def _ldr1d(v_meas, vq_grid, dist_grid, sat, angle_deg, which):
    if v_meas is None:
        return (False, None, 0.0)

    # 1) קודם כל: רוויה על החומר הגולמי (בלי גיין)
    if v_meas >= (sat - SAT_GUARD):      # SAT_GUARD נשאר 60
        # זה "אין מידע" (לא "יש אור אמורפי")
        return (False, None, 0.0)

    # 2) עכשיו גיין למיפוי מרחק בלבד
    g1, g2 = sensor_gains_for_angle(angle_deg)
    g = g1 if which == 1 else g2
    v_corr = int(round(v_meas * g))
    v_corr = max(0, min(v_corr, VREF_Q))

    # 3) Q איכות – לפי המרחק מרוויה *בגולמי*, שלא ייפגע מהקלמפים
    q = max(0.0, min(1.0, (sat - v_meas) / max(1, sat)))

    # 4) LUT → מרחק
    d = nearest_distance_1nn(v_corr, vq_grid, dist_grid)
    d = max(5.0, min(50.0, float(d)))

    return (True, d, q)


    # 6) “יש/אין אור”
    has = (v_corr < (VREF_Q - 120)) or (q > 0.01)

    # 7) הצמדה לטווח 5..50 לפני החזרה
    if has and d is not None:
        d = max(5.0, min(50.0, float(d)))

    return (has, d if has else None, q)


def decide_distance(v1_meas, v2_meas, calib, sat1, sat2, angle_deg):
    """
    פלט:
      has_light, dist_cm (או None), side_bias, d1, d2, beta_eff, q1, q2
    הערה: bias מחושב על ערכים *מוגברים בזווית*, אבל המרחקים d1/d2 על המדידות הגולמיות.
    """
    ok1, d1, q1 = _ldr1d(v1_meas, calib["ldr1"], calib["dist_grid"], sat1, angle_deg, 1)
    ok2, d2, q2 = _ldr1d(v2_meas, calib["ldr2"], calib["dist_grid"], sat2, angle_deg, 2)

    if not ok1 and not ok2:
        return (False, None, 0.0, None, None, BIAS_BETA, 0.0, 0.0)


    # --- נעילה למגזרים ---
    # מתחת ל-(90-Δ): LDR1 (השמאלי)
    if angle_deg <= (90.0 - CENTER_BAND_DEG):
            has1 = ok1                     # היה: ok1 and d1 is not None
            return (has1, d1 if (ok1 and d1 is not None) else None, +1.0,
                    d1 if ok1 else None, None, BIAS_BETA, q1, 0.0)

    # מעל (90+Δ): LDR2 (הימני)
    if angle_deg >= (90.0 + CENTER_BAND_DEG):
        has2 = ok2                     # היה: ok2 and d2 is not None
        return (has2, d2 if (ok2 and d2 is not None) else None, -1.0,
                None, d2 if ok2 else None, BIAS_BETA, 0.0, q2)


    # --- אזור המרכז (90°±Δ): החלטה עדינה ---
    g1, g2 = sensor_gains_for_angle(angle_deg)
    v1b = 0 if v1_meas is None else v1_meas * g1
    v2b = 0 if v2_meas is None else v2_meas * g2
    denom = v1b + v2b
    side_bias = 0.0 if denom <= 0 else max(-1.0, min(1.0, (v1b - v2b) / denom))

    center_factor = 1.0 - abs(angle_deg - 90.0) / 90.0  # 0 בשוליים, 1 במרכז
    beta_eff = min(0.95, BIAS_BETA + AMB_CENTER * center_factor)

    # ספים עדינים לאזור המרכז (אפשר לכוונן אם תרצי)
    Q_STRONG   = 0.18   # איכות שנחשבת "חזקה"
    Q_DOM_DIFF = 0.08   # פער איכות כדי להעדיף צד
    D_SPLIT_CM = 7.0   # פער מרחק שמרמז על שני מקורות שונים

    # 1) אם שני הצדדים רואים חזק ושונים ממש במרחק → שברי רצף כדי לאחד שני מקורות
    if (ok1 and ok2 and d1 is not None and d2 is not None and
        q1 >= Q_STRONG and q2 >= Q_STRONG and abs(d1 - d2) >= D_SPLIT_CM):
        # מחזירים has_light=False כדי ליצור "חור" קטן בריצה ולפצל אותה
        return (False, None, side_bias, d1, d2, beta_eff, q1, q2)

    # 2) אין פער מרחק גדול → מעדיפים צד לפי איכות אם יש דומיננטיות ברורה
    if abs(side_bias) <= beta_eff:
        if ok1 and not ok2:
            return (True, d1, +abs(side_bias), d1, None, beta_eff, q1, q2)
        if ok2 and not ok1:
            return (True, d2, -abs(side_bias), None, d2, beta_eff, q1, q2)

        if (q1 - q2) >= Q_DOM_DIFF:
            return (True, d1, +abs(side_bias), d1, d2, beta_eff, q1, q2)
        if (q2 - q1) >= Q_DOM_DIFF:
            return (True, d2, -abs(side_bias), d1, d2, beta_eff, q1, q2)

        # 3) רק כשבאמת אין הכרעה – נשארים אמביוולנטיים (התנהגות ישנה)
        return (True, None, side_bias, d1 if ok1 else None, d2 if ok2 else None, beta_eff, q1, q2)

    # 4) מחוץ לאמביוולנטיות – בחירה לפי סימן ה-bias (ללא שינו
    # היה:
    # if side_bias > 0: d = d1 ... else: d = d2 ...
    # שיהיה:
    if side_bias > 0:
        d = d1 if ok1 else (d2 if ok2 else None)
    else:
        d = d2 if ok2 else (d1 if ok1 else None)

    return (True, d, side_bias, d1 if ok1 else None, d2 if ok2 else None, beta_eff, q1, q2)



# ---------- חלוקת רצפים של "יש אור" ----------
def segment_runs(light_flags, min_len=2):
    runs = []
    n = len(light_flags)
    i = 0
    while i < n:
        if not light_flags[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and light_flags[j + 1]:
            j += 1
        if j - i + 1 >= min_len:
            runs.append((i, j))
        i = j + 1
    return runs


def segment_runs_with_gap(flags, max_gap=1, min_len=2):
    """כמו segment_runs אבל מאפשר 'חור' של עד max_gap דגימות false בתוך ריצה."""
    runs = []
    n = len(flags)
    i = 0
    while i < n:
        # דלג ל-true הראשון
        while i < n and not flags[i]:
            i += 1
        if i >= n:
            break
        j = i
        gaps = 0
        last_true = i
        # התקדמות בריצה עם רשות לחורים קטנים
        while j + 1 < n:
            if flags[j + 1]:
                j += 1
                last_true = j
                gaps = 0
            else:
                if gaps < max_gap:
                    gaps += 1
                    j += 1
                else:
                    break
        if last_true - i + 1 >= min_len:
            runs.append((i, last_true))
        i = last_true + 1
    return runs


def split_run_by_valleys(angles, dists, i0, i1, min_prom=MIN_PROM, min_sep_deg=MIN_SEP_DEG):
    """
    בתוך ריצה אחת (i0..i1) מפצלת לתת-ריצות אם יש שתי פסגות שונות ב-1/d עם 'עמק' ביניהן.
    מחזירה רשימת (start_idx, end_idx) (בתוך המערכים angles/dists המקוריים).
    """
    # אוסף נקודות עם מרחק חוקי
    idxs = list(range(i0, i1 + 1))
    pairs = [(angles[k], dists[k]) for k in idxs if dists[k] is not None and dists[k] > 0]
    if len(pairs) < 3:
        return [(i0, i1)]

    angs = [a for a, _ in pairs]
    d = [x for _, x in pairs]
    inv = [1.0 / x for x in d]

    # מציאת פסגות ב-1/d
    peaks = [k for k in range(1, len(inv) - 1)
             if inv[k] >= inv[k - 1] and inv[k] >= inv[k + 1] and inv[k] > 0]
    if len(peaks) < 2:
        return [(i0, i1)]

    # חיפוש עמקים בין פסגות ובדיקת prominence + הפרדה זוויתית
    cut_angles = []
    for p1, p2 in zip(peaks, peaks[1:]):
        valley = min(range(p1, p2 + 1), key=lambda t: inv[t])
        prom1 = inv[p1] - inv[valley]
        prom2 = inv[p2] - inv[valley]
        sep = abs(angs[p2] - angs[p1])
        if prom1 > min_prom and prom2 > min_prom and sep >= min_sep_deg:
            cut_angles.append(angs[valley])

    if not cut_angles:
        return [(i0, i1)]

    # המרת זוויות חיתוך לאינדקסים מקוריים
    cut_angles.sort()
    ranges = []
    start = i0
    for ca in cut_angles:
        cut_k = min(range(start, i1 + 1), key=lambda k: abs(angles[k] - ca))
        if cut_k >= start + 1:
            ranges.append((start, cut_k))
            start = cut_k + 1
    if start <= i1:
        ranges.append((start, i1))

    # זריקת תתי-טווחים קצרים מדי
    ranges = [(a, b) for (a, b) in ranges if (b - a + 1) >= 2]
    return ranges if ranges else [(i0, i1)]

####################################################################################################
####################################################################################################
# ========= ה־GUI =========
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DCS Telemeter")
        self.geometry("820x620")
        self.ser = None

        # --- Blue theme & title ---
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        PRIMARY_BG = "#0B3D91"  # כחול כהה נעים
        PRIMARY_FG = "#FFFFFF"
        SUBTITLE_FG = "#E3F2FD"

        self.configure(bg=PRIMARY_BG)
        style.configure("Blue.TFrame", background=PRIMARY_BG)
        style.configure("Blue.TLabel", background=PRIMARY_BG, foreground=PRIMARY_FG)
        style.configure("TitleBig.TLabel", background=PRIMARY_BG, foreground=PRIMARY_FG,
                        font=("Arial", 18, "bold"))
        style.configure("Title.TLabel", background=PRIMARY_BG, foreground=SUBTITLE_FG,
                        font=("Arial", 12, "bold"))

        # (אופציונלי אך מומלץ) סטייל ללשוניות
        style.configure("TNotebook", background=PRIMARY_BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(14, 6), font=("Arial", 11, "bold"))
        style.map("TNotebook.Tab",
                  background=[("selected", "#1565C0")],
                  foreground=[("selected", "#FFFFFF")])

        # Header bar
        hdr = ttk.Frame(self, style="Blue.TFrame")
        hdr.pack(fill="x", padx=10, pady=(6, 0))
        ttk.Label(hdr, text="FINAL PROJECT", style="TitleBig.TLabel").pack(side="left")

        # מצב 1
        self.distances = [None] * SAMPLES
        self.objects = []
        # סף מרחק לאובייקטים (ברירת מחדל = כמו המקס' הכללי, 4 מ')
        self.max_obj_cm_var = tk.IntVar(value=MAX_DIST_CM)

        # מצב 2
        self.mode2_thread = None
        self.mode2_stop_ev = threading.Event()
        self._angle_after_id = None  # לדיבאונס של הסליידר
        self.mode2_active = False  # כדי לשלוח זוויות רק כשאנחנו במצב 2

        # מצב 4
        self.distances4 = [None] * SAMPLES
        self.objects4 = []
        self.sources4 = []

        # מצב 5 (Files)
        self.file_path = None
        self.file_name16_var = tk.StringVar(value="")
        self.file_type_var = tk.StringVar(value="text")
        self.mode5_thread = None
        self.script_cap_thread = None
        self.script_cap_run = False
        self.busy_sending = False

        # --- Top bar: חיבור סיריאלי ---
        top = ttk.Frame(self);
        top.pack(fill="x", padx=10, pady=10)

        self.port_var = tk.StringVar(value=PORT)
        ttk.Label(top, text="Port:").pack(side="left")
        ttk.Entry(top, width=10, textvariable=self.port_var).pack(side="left", padx=5)
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=5)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left", padx=5)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        # --- Notebook (טאבים למצבים) ---
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=10, pady=10)

        # --- Home tab (כפתורי ניווט אנכיים בתוך ה-Notebook) ---
        self.tabHome = ttk.Frame(self.nb, style="Blue.TFrame")
        self.nb.add(self.tabHome, text="Home")
        self._build_home_tab(self.tabHome)

        self.tab1 = ttk.Frame(self.nb);
        self.nb.add(self.tab1, text="Mode 1: ...")
        self._build_tab1(self.tab1)

        self.tab2 = ttk.Frame(self.nb);
        self.nb.add(self.tab2, text="Mode 2: ...")
        self._build_tab2(self.tab2)

        self.tab3 = ttk.Frame(self.nb);
        self.nb.add(self.tab3, text="Mode 3: ...")
        self._build_tab3(self.tab3)

        self.tab4 = ttk.Frame(self.nb);
        self.nb.add(self.tab4, text="Mode 4: ...")
        self._build_tab4(self.tab4)

        self.tab5 = ttk.Frame(self.nb);
        self.nb.add(self.tab5, text="Mode 5: ...")
        self._build_tab5(self.tab5)

        self.tab6 = ttk.Frame(self.nb);
        self.nb.add(self.tab6, text="Mode 6: ...")
        self._build_tab6(self.tab6)

        self.nb.select(self.tabHome)

    # ---------- UI builders ----------
    def _build_tab1(self, parent):
        ttk.Frame(parent, style="Blue.TFrame").pack(fill="x")  # פס רקע
        ttk.Label(parent, text="Mode 1 – Scan & Detect (objects)", style="Title.TLabel").pack(anchor="w", padx=6,
                                                                                              pady=(2, 0))
        ctrl = ttk.Frame(parent)
        ctrl.pack(fill="x", pady=5)
        ttk.Button(ctrl, text="ESC (Back to main)", command=self._send_esc).pack(side="right")

        # שדה הגדרת מרחק מקסימלי לאובייקטים (סינון מצב 1)
        ttk.Label(ctrl, text="Max distance (cm):").pack(side="left")
        ttk.Entry(ctrl, width=6, textvariable=self.max_obj_cm_var).pack(side="left", padx=(4, 12))
        ttk.Button(ctrl, text="Start Object Scan (60 samples)", command=self.start_scan).pack(side="left")
        ttk.Button(ctrl, text="Clear Output", command=lambda: self.output1.delete("1.0", "end")).pack(side="left",
                                                                                                      padx=6)

        self.output1 = tk.Text(parent, height=20)
        self.output1.pack(fill="both", expand=True, pady=5)

    def _build_tab2(self, parent):
        ttk.Frame(parent, style="Blue.TFrame").pack(fill="x")
        ttk.Label(parent, text="Mode 2 – Live Angle/Distance", style="Title.TLabel").pack(anchor="w", padx=6,
                                                                                          pady=(2, 0))
        # Controls: Start/Stop
        ctrl = ttk.Frame(parent);
        ctrl.pack(fill="x", pady=5)
        ttk.Button(ctrl, text="Start Mode 2", command=self.start_mode2).pack(side="left")
        ttk.Button(ctrl, text="Stop Mode 2", command=self.stop_mode2).pack(side="left", padx=6)
        ttk.Button(ctrl, text="ESC (Back to main)", command=self._send_esc).pack(side="right")

        # Angle slider + current value label
        angfrm = ttk.Frame(parent);
        angfrm.pack(fill="x", pady=10)
        ttk.Label(angfrm, text="Angle:").pack(side="left")

        self.angle_var = tk.IntVar(value=90)

        # תווית ערך זווית
        self.ang_val_lbl = ttk.Label(angfrm, text=f"{self.angle_var.get()}°")
        self.ang_val_lbl.pack(side="right")

        # סליידר
        self.angle_scale = ttk.Scale(
            angfrm, from_=0, to=180, orient="horizontal",
            command=self._on_angle_change
        )
        self.angle_scale.pack(fill="x", padx=5)
        self.angle_scale.set(self.angle_var.get())

        # Live distance
        live = ttk.Frame(parent);
        live.pack(fill="x", pady=5)
        ttk.Label(live, text="Live Distance:").pack(side="left")
        self.live_dist_var = tk.StringVar(value="—")
        ttk.Label(live, textvariable=self.live_dist_var, font=("Arial", 12, "bold")).pack(side="left", padx=8)

        # Log
        self.output2 = tk.Text(parent, height=16)
        self.output2.pack(fill="both", expand=True, pady=5)

    def _build_tab3(self, parent):
        ttk.Frame(parent, style="Blue.TFrame").pack(fill="x")
        ttk.Label(parent, text="Mode 3 – Scan & Detect (LDRs)", style="Title.TLabel").pack(anchor="w", padx=6,
                                                                                           pady=(2, 0))
        ctrl = ttk.Frame(parent);
        ctrl.pack(fill="x", pady=5)
        ttk.Button(ctrl, text="Start LDRs Scan (61 samples)", command=self.start_mode3).pack(side="left")
        ttk.Button(ctrl, text="Clear Output", command=lambda: self.output3.delete("1.0", "end")).pack(side="left",
                                                                                                      padx=6)
        ttk.Button(ctrl, text="ESC (Back to main)", command=self._send_esc).pack(side="right")

        self.output3 = tk.Text(parent, height=20)
        self.output3.pack(fill="both", expand=True, pady=5)

    def _build_tab4(self, parent):
        ttk.Frame(parent, style="Blue.TFrame").pack(fill="x")
        ttk.Label(parent, text="Mode 4 – Combined (Objects + LDRs)", style="Title.TLabel").pack(anchor="w", padx=6,
                                                                                                pady=(2, 0))
        ctrl = ttk.Frame(parent)
        ctrl.pack(fill="x", pady=5)
        ttk.Button(ctrl, text="Start Combined Scan (state 4)", command=self.start_mode4).pack(side="left")
        ttk.Button(ctrl, text="Clear Output", command=lambda: self.output4.delete("1.0", "end")).pack(side="left", padx=6)
        ttk.Button(ctrl, text="ESC (Back to main)", command=self._send_esc).pack(side="right")

        self.output4 = tk.Text(parent, height=20)
        self.output4.pack(fill="both", expand=True, pady=5)

    def _build_tab5(self, parent):
        ttk.Frame(parent, style="Blue.TFrame").pack(fill="x")
        ttk.Label(parent, text="Mode 5 – Files", style="Title.TLabel").pack(anchor="w", padx=6, pady=(2, 0))

        # שורת בחירת קובץ
        row1 = ttk.Frame(parent);
        row1.pack(fill="x", pady=6)
        ttk.Label(row1, text="Selected file:").pack(side="left")
        self.file_path_entry = ttk.Entry(row1, width=70)
        self.file_path_entry.pack(side="left", padx=6, fill="x", expand=True)
        ttk.Button(row1, text="Choose...", command=self._choose_file).pack(side="left")

        # הגדרות שם ותו קובץ
        row2 = ttk.Frame(parent);
        row2.pack(fill="x", pady=6)
        ttk.Label(row2, text="LCD name (≤16 ASCII):").pack(side="left")
        ttk.Entry(row2, textvariable=self.file_name16_var, width=20).pack(side="left", padx=6)
        ttk.Label(row2, text="Type:").pack(side="left", padx=(12, 2))
        self.type_combo = ttk.Combobox(row2, textvariable=self.file_type_var, values=["text", "script"], width=10,
                                       state="readonly")
        self.type_combo.pack(side="left")

        # כפתורים לשליטה ב-MCU
        row3 = ttk.Frame(parent);
        row3.pack(fill="x", pady=6)
        ttk.Button(row3, text="Send to MCU", command=self._start_send_file).pack(side="left")
        # ttk.Button(row3, text="MCU → Browse (5B)", command=lambda: self._send_bytes(b'5B')).pack(side="left", padx=8)
        # ttk.Button(row3, text="MCU → Receive (5R)", command=lambda: self._send_bytes(b'5R')).pack(side="left")
        ttk.Button(row3, text="ESC (Back to main)", command=self._send_esc).pack(side="right")

        # יומן מצב
        self.output5 = tk.Text(parent, height=18)
        self.output5.pack(fill="both", expand=True, pady=6)

    def _build_tab6(self, parent):
        ttk.Frame(parent, style="Blue.TFrame").pack(fill="x")
        ttk.Label(parent, text="Mode 6 – Calibration (LDR LUT)", style="Title.TLabel").pack(anchor="w", padx=6,
                                                                                            pady=(2, 0))
        ctrl = ttk.Frame(parent);
        ctrl.pack(fill="x", pady=5)
        ttk.Button(ctrl, text="Run Calibration (state 6)", command=self.start_mode6).pack(side="left")
        ttk.Button(ctrl, text="Clear Output", command=lambda: self.output6.delete("1.0", "end")).pack(side="left",
                                                                                                      padx=6)
        ttk.Button(ctrl, text="ESC (Back to main)", command=self._send_esc).pack(side="right")

        self.output6 = tk.Text(parent, height=20)
        self.output6.pack(fill="both", expand=True, pady=5)

    def _build_home_tab(self, parent):
        # כותרת משנה (יש לך כבר כותרת ראשית ב-Header)
        sec = ttk.Frame(parent, style="Blue.TFrame")
        sec.pack(fill="x", pady=(12, 6))
        ttk.Label(sec, text="בחר מצב", style="Title.TLabel").pack(anchor="center")

        # עמודת כפתורים אנכית—כל כפתור פותח טאב
        col = ttk.Frame(parent, style="Blue.TFrame")
        col.pack(fill="x", padx=16, pady=10)

        ttk.Button(col, text="Mode 1: Scan & Detect (objects)",
                   command=lambda: self.nb.select(self.tab1)).pack(fill="x", pady=6)
        ttk.Button(col, text="Mode 2: Live Angle/Distance",
                   command=lambda: self.nb.select(self.tab2)).pack(fill="x", pady=6)
        ttk.Button(col, text="Mode 3: Scan & Detect (LDRs)",
                   command=lambda: self.nb.select(self.tab3)).pack(fill="x", pady=6)
        ttk.Button(col, text="Mode 4: Combined (Objects + LDRs)",
                   command=lambda: self.nb.select(self.tab4)).pack(fill="x", pady=6)
        ttk.Button(col, text="Mode 5: Files",
                   command=lambda: self.nb.select(self.tab5)).pack(fill="x", pady=6)
        ttk.Button(col, text="Mode 6: Calibration (LDR LUT)",
                   command=lambda: self.nb.select(self.tab6)).pack(fill="x", pady=6)

    # ---------- Serial ----------

    # קבועים קיימים אצלך:
# ACK = 0x06
# NACK = 0x15

    def _read_payload_with_cs(self, n_bytes, total_timeout=1.0):
        """קורא בדיוק n_bytes + cs (בייט אחד), מאמת checksum-256, שולח ACK/NACK.
        מחזיר bytes של המטען (ללא cs) או None בשגיאה/timeout."""
        end = time.time() + total_timeout
        buf = bytearray()
        while time.time() < end and len(buf) < (n_bytes + 1):
            if not (self.ser and self.ser.is_open):
                return None
            need = (n_bytes + 1) - len(buf)
            chunk = self.ser.read(need)
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.005)

        if len(buf) != (n_bytes + 1):
            # לא קיבלתי הכל → NACK
            try: self.ser.write(bytes([NACK]))
            except: pass
            return None

        payload, cs = buf[:n_bytes], buf[-1]
        calc = sum(payload) & 0xFF
        try:
            if calc == cs:
                self.ser.write(bytes([ACK]))
                return bytes(payload)
            else:
                self.ser.write(bytes([NACK]))
                return None
        except:
            return None


    def _read_u16_with_cs(self, total_timeout=1.0):
        b = self._read_payload_with_cs(2, total_timeout)
        if not b: return None
        return (b[0] << 8) | b[1]

    def _read_u16x2_with_cs(self, total_timeout=1.0):
        b = self._read_payload_with_cs(4, total_timeout)
        if not b: return None
        v1 = (b[0] << 8) | b[1]
        v2 = (b[2] << 8) | b[3]
        return v1, v2


    def _read_u16x3_with_cs(self, total_timeout=1.0):
        b = self._read_payload_with_cs(6, total_timeout)
        if not b: return None
        a = (b[0] << 8) | b[1]  # למשל distance
        v1 = (b[2] << 8) | b[3]
        v2 = (b[4] << 8) | b[5]
        return a, v1, v2


    def _build_home_menu(self):
        # סטייל כללי
        self.configure(bg="#0B67C7")  # כחול רקע כללי לחלון
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("Blue.TFrame", background="#0B67C7")
        style.configure("Blue.TLabel", background="#0B67C7", foreground="white",
                        font=("Arial", 20, "bold"))
        # כפתור תפריט גדול (נשתמש ב-tk.Button בשביל צבע מלא)
        self.home = ttk.Frame(self, style="Blue.TFrame")
        self.home.pack(fill="both", expand=True)

        title = ttk.Label(self.home, text="DCS Final Project - Ron and Matan",
                          style="Blue.TLabel")
        title.pack(pady=(20, 10))

        btns = tk.Frame(self.home, bg="#0B67C7")
        btns.pack(pady=10)

        def mk_button(text, cmd):
            b = tk.Button(btns, text=text, command=cmd,
                          bg="#0A73D9", fg="white", activebackground="#095fb7",
                          font=("Arial", 16, "bold"), relief="flat",
                          padx=20, pady=10, width=28)
            b.pack(pady=8, fill="x")
            return b

        mk_button("Objects Detector System", lambda: self._open_mode_tab(0))  # Tab 1
        mk_button("Telemeter", lambda: self._open_mode_tab(1))  # Tab 2
        mk_button("Light Source Detector System", lambda: self._open_mode_tab(2))  # Tab 3
        mk_button("Light Source and Objects Detector System",
                  lambda: self._open_mode_tab(3))  # Tab 4
        mk_button("Script Mode", lambda: self._open_mode_tab(4))  # Tab 5
        mk_button("LDR Calibration", lambda: self._open_mode_tab(5))  # Tab 6

    def _show_home(self):
        # להסתיר את ה-tabs ולהציג את הבית
        if self.nb.winfo_ismapped():
            self.nb.pack_forget()
        if not self.home.winfo_ismapped():
            self.home.pack(fill="both", expand=True)

    def _open_mode_tab(self, idx):
        # לפתוח את הטאב המתאים ולעבור אליו
        if self.home.winfo_ismapped():
            self.home.pack_forget()
        if not self.nb.winfo_ismapped():
            self.nb.pack(fill="both", expand=True, padx=10, pady=10)
        self.nb.select(idx)

    def connect(self):
        try:
            self.ser = serial.Serial(self.port_var.get(), BAUD, timeout=100, write_timeout=1)
            self.status_var.set(f"Connected to {self.port_var.get()} @ {BAUD}")
        except Exception as e:
            messagebox.showerror("Serial", f"Failed to open port: {e}")

    def disconnect(self):
        try:
            self.script_cap_run = False
            self.stop_mode2()  # ודא שעוצרים קריאה ברקע
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.status_var.set("Disconnected")
        except Exception as e:
            messagebox.showerror("Serial", f"Failed to close: {e}")

    def _send_chunk_and_wait_ack(self, chunk: bytes) -> bool:
        """
        שולח חתיכה אחת (chunk) עם checksum-256 בסופה, ואז מחכה ל-ACK.
        מנסה עד MAX_CHUNK_RETRIES פעמים. מחזיר True אם ה-ACK התקבל.
        """
        if not (self.ser and self.ser.is_open):
            return False

        cs = sum(chunk) & 0xFF
        frame = bytes(chunk) + bytes([cs])

        for attempt in range(1, MAX_CHUNK_RETRIES + 1):
            try:
                # שליחה
                self.ser.write(frame)
                self.ser.flush()

                # המתנה ל-ACK/NACK עם טיימאאוט קצר
                prev_to = self.ser.timeout
                try:
                    self.ser.timeout = ACK_TIMEOUT_SEC
                    b = self.ser.read(1)
                finally:
                    self.ser.timeout = prev_to

                if not b:
                    self._append5(f"Chunk: timeout (attempt {attempt}/{MAX_CHUNK_RETRIES})\n")
                    continue

                if b[0] == ACK:
                    return True
                elif b[0] == NACK:
                    self._append5(f"Chunk: NACK (attempt {attempt}/{MAX_CHUNK_RETRIES})\n")
                    continue
                else:
                    self._append5(f"Chunk: unexpected 0x{b[0]:02X} (attempt {attempt}/{MAX_CHUNK_RETRIES})\n")
                    continue

            except Exception as e:
                self._append5(f"Chunk send error: {e} (attempt {attempt}/{MAX_CHUNK_RETRIES})\n")
                continue

        return False



    def _force_one_when_variation(self, Ls_v, Rs_v, angle1, angle2, calib):
        import numpy as np

        Ls = np.asarray(Ls_v, dtype=float)
        Rs = np.asarray(Rs_v, dtype=float)
        n  = len(Ls)
        if n == 0:
            return []

        angles_raw = np.linspace(float(angle1), float(angle2), n)

        def _smooth(x, k):
            if k <= 1 or x.size < 3: return x
            k = int(max(1, k | 1))
            w = np.ones(k, dtype=float) / k
            return np.convolve(x, w, mode='same')

        Ls_s = _smooth(Ls, SMOOTH_K)
        Rs_s = _smooth(Rs, SMOOTH_K)

        # בסיס גבוה + קלמפ לחיובי (לא לבחור דיפ "שלילי")
        baseL = np.percentile(Ls_s, 92.0)
        baseR = np.percentile(Rs_s, 92.0)
        dipsL = np.maximum(0.0, baseL - Ls_s)
        dipsR = np.maximum(0.0, baseR - Rs_s)

        # לפני החישוב של dips:
        satL = compute_sat_threshold(calib['ldr1' if LEFT_IS_LDR1 else 'ldr2'], frac_of_vref=0.98)
        satR = compute_sat_threshold(calib['ldr2' if LEFT_IS_LDR1 else 'ldr1'], frac_of_vref=0.98)
        SAT_GUARD_PICK = 120

        # אחרי שמחשבים base/dips לכל צד:
        maskL = (Ls_s >= (satL - SAT_GUARD_PICK))
        maskR = (Rs_s >= (satR - SAT_GUARD_PICK))

        dipsL[maskL] = 0.0
        dipsR[maskR] = 0.0


        dips  = np.maximum(dipsL, dipsR)

        # לא לאפשר בחירת קצה (0°/180°)
        deg_per_bin = 180.0 / max(1.0, n - 1)
        edge_bins = max(1, int(round(EDGE_GUARD_DEG / deg_per_bin)))
        if dips.size >= 2*edge_bins:
            dips[:edge_bins] = 0.0
            dips[-edge_bins:] = 0.0

        if dips.max() <= 0.0:
            return []  # באמת שטוח – אין מה להכריח

        # הגנה מרוחב – לא לבחור רעש נקודתי
        def _valley_width_bins(dv, idx, frac=0.5):
            h = float(dv[idx]) * float(frac)
            L = int(idx)
            while L > 0 and float(dv[L]) >= h: L -= 1
            R = int(idx)
            last = dv.size - 1
            while R < last and float(dv[R]) >= h: R += 1
            return max(1, R - L)

        i_star = int(np.argmax(dips))
        w_deg = _valley_width_bins(dips, i_star, 0.5) * deg_per_bin
        if w_deg < 2.0:
            return []  # דיפ צר מדי – להימנע מהכרחה

        ang_star = float(np.interp(i_star, np.arange(n), angles_raw))

        # בחר את הטבלה לפי איזה צד שקע יותר בנק' הזו
        use_left = (dipsL[i_star] >= dipsR[i_star])
        dist_grid = np.asarray(calib['dist_grid'], dtype=float)
        vqL = np.asarray(calib['ldr1' if LEFT_IS_LDR1 else 'ldr2'], dtype=float)
        vqR = np.asarray(calib['ldr2' if LEFT_IS_LDR1 else 'ldr1'], dtype=float)

        def _nearest_cm(v, vq_table):
            i = int(np.argmin(np.abs(vq_table - float(v))))
            return int(dist_grid[i])

        v_at = Ls[i_star] if use_left else Rs[i_star]
        dcm  = _nearest_cm(v_at, vqL if use_left else vqR)
        dcm  = max(5, min(50, int(dcm)))

        return [(ang_star, dcm)]


    def _decide_lights_from_pairs(self, Ls_v, Rs_v, angle1, angle2, calib, collapse_center=COLLAPSE_CENTER_DEFAULT):
        """
        מחליט מקורות אור מתוך שתי סדרות Q (שמאל/ימין) ע״י:
        - החלקה קלה
        - דיפ מול פרסנטיל גבוה (baseline)
        - רצועת הסכמה באזור המרכז
        - בדיקת רוחב, הגנות קצה, דומיננטיות מול הצד השני
        - NMS ופילטור picks סמוכים
        מחזיר [(angle_deg, dist_cm), ...]
        """
        import numpy as np

        n = len(Ls_v)
        if n == 0:
            return []

        Ls = np.asarray(Ls_v, dtype=float)
        Rs = np.asarray(Rs_v, dtype=float)

        angles_raw = np.linspace(float(angle1), float(angle2), n)
        span_raw = max(1e-6, float(angles_raw[-1] - angles_raw[0]))
        angles = (angles_raw - angles_raw[0]) * (180.0 / span_raw)
        angles = np.clip(angles, 0.0, 180.0)
        deg_per_bin = float(np.mean(np.diff(angles))) if n > 1 else 1.0

        def _smooth(x, k):
            if k <= 1 or x.size < 3:
                return x
            k = int(max(1, k | 1))
            w = np.ones(k, dtype=float) / k
            return np.convolve(x, w, mode='same')

        def _valley_width_bins(dips, idx, frac=0.5):
            h = float(dips[idx]) * float(frac)
            L = int(idx)
            while L > 0 and float(dips[L]) >= h:
                L -= 1
            R = int(idx)
            last = dips.size - 1
            while R < last and float(dips[R]) >= h:
                R += 1
            return max(1, R - L)

        # מרכז (Zero-cross של Rs-Ls)
        diff = Rs - Ls
        zc = np.where(diff[:-1] * diff[1:] <= 0)[0]
        m = int((zc[len(zc) // 2] + 1) if zc.size else np.argmin(np.abs(diff)))
        m = int(np.clip(m, int(0.25 * n), int(0.75 * n)))

        # פרמטרים
        P_LEFT  = dict(BASE_PCT=90.0, DIP_ABS=0.07, DIP_REL=0.03,
                    SEP_DEG=12.0, GHOST_ABS=0.22, GHOST_RATIO=0.70,
                    MIN_WIDTH_DEG=2.5, MAX_WIDTH_DEG=180.0)

        P_RIGHT = dict(BASE_PCT=90.0, DIP_ABS=0.07, DIP_REL=0.03,
                    SEP_DEG=12.0, GHOST_ABS=0.22, GHOST_RATIO=0.70,
                    MIN_WIDTH_DEG=2.5, MAX_WIDTH_DEG=180.0)
        # מרכז:
        CENTER_BAND_DEG = 10.0  # היה 15.0 – מצמצם את אזור הדרישה להסכמה דו-צידית
        AGREE_ABS, AGREE_REL, OTHER_REL = 0.09, 0.40, 0.05  # היה 0.09,0.45,0.05 (מרגיע קלות את היחס)
        SAT_NEAR = 120  # כמה מתחת ל-VREF_Q נחשב "כמעט רווי" לביטול דרישת הסכמה
        STRONG_SINGLE_GAIN = 1.8  # כמה חזק מעל סף ה-dip כדי לאפשר קבלה לבד באוברלאפ



        def _detect_half(vals, other, start, end, label, P):
            seg = vals[start:end]
            oth = other[start:end]
            if seg.size < 3:
                return []

            seg_s = _smooth(seg, SMOOTH_K)
            oth_s = _smooth(oth, SMOOTH_K)

            base = np.percentile(seg_s, P['BASE_PCT'])
            dips = base - seg_s
            thr = max(P['DIP_ABS'], P['DIP_REL'] * max(1e-6, base))

            base_o = np.percentile(oth_s, P['BASE_PCT'])
            dips_o = base_o - oth_s

            cand = np.where((dips[1:-1] >= dips[:-2]) & (dips[1:-1] >= dips[2:]))[0] + 1
            cand = cand[dips[cand] >= thr]
            order = cand[np.argsort(dips[cand])[::-1]] if cand.size else cand

            sep_bins = max(1, int(np.ceil(P['SEP_DEG'] / max(1e-6, deg_per_bin))))
            kept, taken = [], np.zeros(seg.size, dtype=bool)
            for idx in order:
                if taken[idx]:
                    continue
                ang = float(np.interp(idx, np.arange(seg.size), angles[start:end]))

                in_overlap = (label == "L" and ang > (angles[m] - CENTER_BAND_DEG)) or \
                            (label == "R" and ang < (angles[m] + CENTER_BAND_DEG))

                w_deg = _valley_width_bins(dips, idx, 0.5) * deg_per_bin
                min_width = P['MIN_WIDTH_DEG']
                dom_abs = DOM_ABS
                if ang < EDGE_GUARD_DEG or ang > 180.0 - EDGE_GUARD_DEG:
                    min_width = max(min_width, EDGE_WIDTH_MIN)
                    dom_abs += EDGE_DOM_BOOST
                if not (min_width <= w_deg <= P['MAX_WIDTH_DEG']):
                    continue


                if in_overlap:
                    thr_other = max(AGREE_ABS, OTHER_REL * max(1e-6, base_o))
                    # בדיקת "הצד האחר כמעט רווי" → נחשיב כהסכמה רכה
                    other_val = oth_s[idx]  # האות של הצד האחר (מוחלק)
                    other_is_sat = (other_val >= (VREF_Q - SAT_NEAR))

                    ok_other = (dips_o[idx] >= thr_other and dips_o[idx] >= AGREE_REL * dips[idx]) or other_is_sat

                    # פסגה מאוד חזקה בצד הזה? קבל גם בלי הסכמה (רק באזור האוברלאפ)
                    strong_single = (dips[idx] >= STRONG_SINGLE_GAIN * thr)

                    if not (ok_other or strong_single):
                        continue
                else:
                    dominates = (dips[idx] - dips_o[idx] >= dom_abs) or (dips[idx] >= (1.0 + DOM_REL) * dips_o[idx])
                    ghost_small = (dips_o[idx] <= P['GHOST_ABS'] and dips_o[idx] <= P['GHOST_RATIO'] * dips[idx])
                    if not (dominates or ghost_small):
                        continue

                kept.append(idx)
                lo = max(0, idx - sep_bins // 2)
                hi = min(seg.size, idx + sep_bins // 2 + 1)
                taken[lo:hi] = True
                if MAX_PICKS_PER_HALF and len(kept) >= MAX_PICKS_PER_HALF:
                    break

            if kept:
                return [start + k for k in kept]

            # fallback: רק מחוץ לאוברלאפ
            idx_best = int(np.argmax(dips))
            ang_best = float(np.interp(idx_best, np.arange(seg.size), angles[start:end]))
            in_overlap = (label == "L" and ang_best > (angles[m] - CENTER_BAND_DEG)) or \
                        (label == "R" and ang_best < (angles[m] + CENTER_BAND_DEG))
            if not in_overlap and dips[idx_best] >= thr:
                min_width = P['MIN_WIDTH_DEG']
                dom_abs = DOM_ABS
                if ang_best < EDGE_GUARD_DEG or ang_best > 180.0 - EDGE_GUARD_DEG:
                    min_width = max(min_width, EDGE_WIDTH_MIN)
                    dom_abs += EDGE_DOM_BOOST
                w_deg = _valley_width_bins(dips, idx_best, 0.5) * deg_per_bin
                if w_deg >= min_width:
                    dominates = (dips[idx_best] - dips_o[idx_best] >= dom_abs) or \
                                (dips[idx_best] >= (1.0 + DOM_REL) * dips_o[idx_best])
                    ghost_small = (dips_o[idx_best] <= P['GHOST_ABS'] and \
                                dips_o[idx_best] <= P['GHOST_RATIO'] * dips[idx_best])
                    if dominates or ghost_small:
                        return [start + idx_best]
            return []

        picks_left  = _detect_half(Ls, Rs, 0, m, "L", P_LEFT)
        picks_right = _detect_half(Rs, Ls, m, n, "R", P_RIGHT)
        picks = sorted(picks_left + picks_right)

        merged = []
        for idx in picks:
            if not merged or abs(angles[idx] - angles[merged[-1]]) >= MERGE_DEG:
                merged.append(idx)
            else:
                prev = merged[-1]
                better = (idx <= m and Ls[idx] < Ls[prev]) or (idx > m and Rs[idx] < Rs[prev])
                if better:
                    merged[-1] = idx

        # מיפוי לעומק לפי ה-LUTים מהכיול
        dist_grid = np.asarray(calib['dist_grid'], dtype=float)
        # לכבד את כיוון החיישנים:
        vqL = np.asarray(calib['ldr1' if LEFT_IS_LDR1 else 'ldr2'], dtype=float)
        vqR = np.asarray(calib['ldr2' if LEFT_IS_LDR1 else 'ldr1'], dtype=float)

        def _nearest_cm(v, vq_table):
            i = int(np.argmin(np.abs(vq_table - float(v))))
            return int(dist_grid[i])

        out = []
        for i in merged:
            dcm = _nearest_cm(Ls[i], vqL) if i <= m else _nearest_cm(Rs[i], vqR)
            dcm = max(5, min(50, int(dcm)))
            out.append((float(angles_raw[i]), int(dcm)))


   # --- collapse symmetric double-pick (one per half) into a single center pick ---
        # --- center-only collapse (optional) ---
        def _smooth(x, k):
            import numpy as np
            if k <= 1 or len(x) < 3: return np.asarray(x, float)
            k = int(max(1, k | 1))
            w = np.ones(k, dtype=float) / k
            return np.convolve(np.asarray(x, float), w, mode='same')

        if len(out) == 2 and collapse_center:
            import numpy as np
            (a1, d1), (a2, d2) = out
            am = float(np.interp(m, np.arange(n), angles_raw))  # מרכז לפי Rs-Ls

            # חייבים להיות משני צידי המרכז ובטווח צר סביבו
            sym = ((a1 < am and a2 > am) or (a2 < am and a1 > am))
            if sym \
            and max(abs(a1 - am), abs(a2 - am)) <= COLLAPSE_BAND_DEG \
            and abs(a1 - a2) <= COLLAPSE_MAX_SEP_DEG \
            and abs(d1 - d2) <= COLLAPSE_MAX_DDIFF_CM:

                # בדיקת “דיפ דומה” בשני הצדדים
                i1 = int(np.argmin(np.abs(angles_raw - a1)))
                i2 = int(np.argmin(np.abs(angles_raw - a2)))
                Ls_s = _smooth(Ls, SMOOTH_K); Rs_s = _smooth(Rs, SMOOTH_K)
                baseL = np.percentile(Ls_s, 90.0); baseR = np.percentile(Rs_s, 90.0)

                dip1 = (baseL - Ls_s[i1]) if i1 <= m else (baseR - Rs_s[i1])
                dip2 = (baseR - Rs_s[i2]) if i2 >= m else (baseL - Ls_s[i2])
                ok_dips = (min(dip1, dip2) > 0) and (abs(dip1 - dip2) / max(dip1, dip2) <= COLLAPSE_MAX_DIP_REL)

                if ok_dips:
                    out = [(am, min(d1, d2))]   # מאחדים רק כאן
 

        if not out:
            forced = self._force_one_when_variation(Ls, Rs, angle1, angle2, calib)
            if forced:
                out = forced
        return out




    # ---------- Helpers ----------
    def _append1(self, text):
        # מצב 1: debug למסוף בלבד (לא ל-GUI)
        try:
            print(text, end="")
        except Exception:
            pass

    def _append2(self, text):
        self.after(0, lambda: (self.output2.insert("end", text),
                               self.output2.see("end")))

    def _append3(self, text):
        # מצב 3: debug למסוף בלבד (לא ל-GUI)
        try:
            print(text, end="")
        except Exception:
            pass

    def _append4(self, text):
        self.after(0, lambda: (self.output4.insert("end", text),
                               self.output4.see("end")))

    def _append5(self, text):
        self.after(0, lambda: (self.output5.insert("end", text),
                               self.output5.see("end")))

    def _append6(self, text):
        self.after(0, lambda: (self.output6.insert("end", text + "\n"),
                               self.output6.see("end")))

    def _send_bytes(self, data: bytes):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return
        try:
            self.ser.write(data)
            self._append5(f"TX: {data!r}\n")
        except Exception as e:
            messagebox.showerror("Serial", f"Write failed: {e}")

    def _go_home_esc(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'\x1B')  # ESC → יחזיר state0 ב-MCU
        except Exception:
            pass
        self._show_home()

    def _read_distance_u8(self, total_timeout=1.0):
        end = time.time() + total_timeout
        while time.time() < end:
            b = self.ser.read(1)
            if b:
                return b[0]  # 0..255
        return None

    def _read_distance_word(self, total_timeout=3.0):
        """קורא בדיוק 2 בייט (hi, lo) ומחזיר int בס״מ; None אם timeout."""
        end = time.time() + total_timeout
        buf = bytearray()
        while time.time() < end and len(buf) < 2:
            if not (self.ser and self.ser.is_open):
                return None
            chunk = self.ser.read(2 - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.01)
        if len(buf) != 2:
            return None
        hi, lo = buf[0], buf[1]
        return (hi << 8) | lo

    def _send_esc(self):
        """שלח ESC לבקר, עצור מצב 2 אם רץ, חזור לטאב Home."""
        try:
            if self.mode2_active:
                self.stop_mode2()
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
                self.ser.write(b'\x1B')  # ESC → ISR יחזיר state0
                self.ser.flush()
                time.sleep(0.02)
                # אופציונלי: גם '0' ל-idle, אם תרצה ביטחון כפול:
                # self.ser.write(b'0'); self.ser.flush()
        except Exception:
            pass
        # מעבר לטאב הבית
        try:
            if hasattr(self, "nb") and hasattr(self, "tabHome"):
                self.nb.select(self.tabHome)
        except Exception:
            pass

    # ========= Mode 1 =========
    def start_scan(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return
        # נקה פלט ואת המערך
        self.output1.delete("1.0", "end")
        self.distances = [None] * SAMPLES
        self.objects = []

        # שלח למיקרו את מצב 1 (אות ASCII '1')
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'1')
        except Exception as e:
            messagebox.showerror("Serial", f"Write failed: {e}")
            return

        t = threading.Thread(target=self._scan_worker, daemon=True)
        t.start()

    def _scan_worker(self):
        """קורא 60 דגימות (2 בייט לכל דגימה), מסכם, ואז מזהה אובייקטים."""
        for i in range(SAMPLES):

            d = self._read_u16_with_cs(total_timeout=1.0)
            print(d)
            angle = i * STEP_DEG

            if d is None:
                self._append1(f"[{i + 1:02d}/60] angle={angle:3d}°  timeout/no data\n")
                self.distances[i] = None
                continue

            if d <= 0 or d > MAX_DIST_CM:
                self._append1(f"[{i + 1:02d}/60] angle={angle:3d}°  NO-OBJECT\n")
                self.distances[i] = None
                continue

            try:
                user_max = int(self.max_obj_cm_var.get())
            except Exception:
                user_max = MAX_DIST_CM
            user_max = max(1, min(MAX_DIST_CM, user_max))

            if d > user_max:
                self._append1(f"[{i + 1:02d}/60] angle={angle:3d}°  NO-OBJECT\n")
                self.distances[i] = None
            else:
                self._append1(f"[{i + 1:02d}/60] angle={angle:3d}°  distance={d} cm\n")
                self.distances[i] = d

        # --- סיכום דגימות ---
        vals = [x for x in self.distances if x is not None]
        if vals:
            avg = sum(vals) / len(vals)
            line = f"\nSummary: got {len(vals)}/60 | min={min(vals)} max={max(vals)} avg={avg:.1f} cm\n"
            self._append1(line)
            print(line)  # ← הוספה
        else:
            line = "\nSummary: no samples\n"
            self._append1(line)
            print(line)  # ← הוספה

        # --- זיהוי אובייקטים ---
        self.objects = detect_objects(self.distances)
        if not self.objects:
            line = "Objects: none\n"
            self._append1(line)
            print(line)  # ← הוספה
        else:
            header = f"Objects detected: {len(self.objects)}\n"
            self._append1(header)
            print(header)  # ← הוספה
            for idx, o in enumerate(self.objects, 1):
                line = (f"  Obj#{idx}: angle={o['angle_deg']:.1f}°, "
                        f"dist={o['distance_cm']:.1f} cm, "
                        f"width={o['width_cm']:.1f} cm, "
                        f"samples={o['n']}\n")
                self._append1(line)
                print(line)  # ← הוספה
        # ← הוסף כאן (לא חוסם את ה-GUI)
        self.after(0, self._plot_objects_semicircle)

        # חזרה ל-idle (אם זה הפרוטוקול אצלכם)
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'0')
        except:
            pass

    ####################################################################################################
    def _plot_objects_semicircle(self):
        if not getattr(self, "objects", None):
            return

        import math
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt

        # רדיוס לפי קלט המשתמש
        try:
            user_max = int(self.max_obj_cm_var.get())
        except Exception:
            user_max = MAX_DIST_CM
        user_max = max(1, min(MAX_DIST_CM, user_max))
        R = float(user_max)

        # חלון קטן יותר
        fig = plt.figure(figsize=(5.6, 3.0), dpi=100)
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")
        ax.axis("off")

        # חצי־מעגל אפור
        ts = [math.radians(t) for t in range(0, 181)]
        xs = [R * math.cos(t) for t in ts]
        ys = [R * math.sin(t) for t in ts]
        ax.plot(xs, ys, color="#888888", linewidth=1.2)

        # תוויות קצה
        ax.text(R, 0, "0 deg", ha="right", va="top", fontweight="bold", fontsize=9)
        ax.text(-R, 0, "180 deg", ha="left", va="top", fontweight="bold", fontsize=9)

        # נקודות אובייקטים כחולות + תווית (r, θ, w)
        for o in self.objects:
            r_raw = float(o["distance_cm"])
            ang = float(o["angle_deg"])
            w = float(o["width_cm"])

            r_plot = min(r_raw, R)
            phi = math.radians(ang)
            x, y = r_plot * math.cos(phi), r_plot * math.sin(phi)

            ax.scatter([x], [y], s=32, color="blue")  # נקודה קצת קטנה יותר
            ax.text(x, y + R * 0.04, f"({int(round(r_raw))}cm, {int(round(ang))}deg, {int(round(w))}cm)",
                    ha="center", va="bottom", fontsize=8)

        # גבולות הדוקה יותר
        m = R * 0.06
        ax.set_xlim(-R - m, R + m)
        ax.set_ylim(0, R + m)
        plt.tight_layout()
        plt.show()

    ####################################################################################################
    # ========= Mode 2 =========
    def start_mode2(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return

        # שליחת מצב 2 + זווית התחלתית
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'2')  # כניסה למצב 2
            init_angle = int(round(self.angle_scale.get()))
            init_angle = max(0, min(180, init_angle))
            self.mode2_active = True
            self._send_angle(init_angle)
        except Exception as e:
            messagebox.showerror("Serial", f"Write failed: {e}")
            return

        self.live_dist_var.set("—")
        self.output2.delete("1.0", "end")
        self._append2(f"Mode 2 started, angle={init_angle}°\n")

        # הפעלת קורא לייב
        self.mode2_stop_ev.clear()
        self.mode2_thread = threading.Thread(target=self._mode2_reader, daemon=True)
        self.mode2_thread.start()

    def stop_mode2(self):
        # עצירת הקריאה, חזרה ל-idle
        self.mode2_active = False
        self.mode2_stop_ev.set()
        if self.mode2_thread and self.mode2_thread.is_alive():
            self.mode2_thread.join(timeout=1.0)
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'0')
        except:
            pass
        self._append2("Mode 2 stopped.\n")

    def _mode2_reader(self):
        """קורא מרחקים בלייב כל עוד במצב 2; מציג בלייב + כותב ליומן."""
        while not self.mode2_stop_ev.is_set():
            d = self._read_u16_with_cs(total_timeout=0.6)
            if d is None:
                continue
            # סינון ערכים לא תקינים
            if d == 0:
                continue  # ← פשוט מתעלמים לגמרי מה־0
            # הגבלת תצוגה ל-4 מטר
            if d > MAX_DIST_CM:
                disp = "— (out of range)"
            else:
                disp = f"{d} cm"

            # עדכון GUI
            self.after(0, lambda v=disp: self.live_dist_var.set(v))
            self._append2(f"Distance: {d} cm\n")

    def _send_angle(self, angle):
        """שולח בייט זווית (0–180)."""
        if not (self.ser and self.ser.is_open):
            return
        angle = max(0, min(180, int(angle)))
        try:
            self.ser.write(bytes([angle]))
            self._append2(f"Set angle → {angle}°\n")
        except Exception as e:
            self._append2(f"Set angle failed: {e}\n")

    def _on_angle_change(self, _val):
        """נקרא בכל תזוזה של הסליידר; מעדכן את התווית ושולח זווית בדיבאונס אם אנחנו במצב 2."""
        val = int(round(float(_val)))
        self.angle_var.set(val)

        if getattr(self, "ang_val_lbl", None):
            self.ang_val_lbl.config(text=f"{val}°")

        if not self.mode2_active:
            return

        if self._angle_after_id:
            self.after_cancel(self._angle_after_id)
        self._angle_after_id = self.after(150, lambda: self._send_angle(self.angle_var.get()))

    # ========= Mode 3: Scan & Detect (LDRs) =========
    def start_mode3(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return
        self.output3.delete("1.0", "end")

        try:
            # 1) ודאי שמצב 2 באמת נעצר
            self.stop_mode2()  # בטוח, גם אם לא פעיל

            # 2) סנכרון קצר למיקרו לאיידל
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'0')  # החזרה ל-idle אצל הבקר (יש לך את זה כבר במקומות אחרים)
            self.ser.flush()
            time.sleep(0.05)  # 50ms לתת לשאריות לצאת

            # 3) כניסה למצב 3 לאחר שהקו נקי
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'3')
            self.ser.flush()
        except Exception as e:
            messagebox.showerror("Serial", f"Write failed: {e}")
            return

        t = threading.Thread(target=self._scan_worker_ldr, daemon=True)
        t.start()
    def _scan_worker_ldr(self):
        # --- קריאת 10+10 ערכי הכיול ---
        def _read_word_or_fail(to=1.0):
            v = self._read_distance_word(total_timeout=to)
            if v is None:
                raise RuntimeError("Timeout while reading from MCU")
            return v

        ldr1_q = [_read_word_or_fail() for _ in range(10)]
        ldr2_q = [_read_word_or_fail() for _ in range(10)]


        self._append3(f"Calib LDR1: {ldr1_q}\n")
        self._append3(f"Calib LDR2: {ldr2_q}\n")

        # --- LUT + ספי רוויה ---
        try:
            dist_grid, vq1_grid = build_lut(ldr1_q)
            _, vq2_grid = build_lut(ldr2_q)
        except Exception as e:
            self._append3(f"LUT build failed: {e}\n")
            return

        sat1 = compute_sat_threshold(vq1_grid)
        sat2 = compute_sat_threshold(vq2_grid)
        calib = {"dist_grid": dist_grid, "ldr1": vq1_grid, "ldr2": vq2_grid}
        self._append3(f"LUT ready. sat1={sat1}, sat2={sat2}\n")

        # העירי MCU להתחיל סריקה (יציאה מ-LPM על '8')
        try:
            self.ser.write(b'8')
        except Exception as e:
            self._append3(f"Wake MCU failed: {e}\n")
            return

        angles = []
        has_list = []
        bias_list = []
        dist_left = []  # רק נקודות שה־bias מובהק לצד שמאל
        dist_right = []  # רק נקודות שה־bias מובהק לצד ימין
        Ls = []   # Q של הצד השמאלי (Left)
        Rs = []   # Q של הצד הימני (Right)


        # --- סריקה 0..180° ---
        for i in range(SAMPLES):
            angle = i * STEP_DEG

            pair = self._read_u16x2_with_cs(total_timeout=2.0)
            if pair is None:
                self._append3(f"[{i + 1:02d}/{SAMPLES}] angle={angle:3d}°  timeout/no data\n")
                break
            v1, v2 = pair
            # אם LEFT_IS_LDR1=False → שמאל = LDR2, ימין = LDR1 (זה מה שכתבתי קודם)
            v_left  = v1 if LEFT_IS_LDR1 else v2
            v_right = v2 if LEFT_IS_LDR1 else v1
            Ls.append(v_left)
            Rs.append(v_right)


            if v1 is None and v2 is None:
                self._append3(f"[{i + 1:02d}/{SAMPLES}] angle={angle:3d}°  timeout/no data\n")
                break

            has_light, d_sel, sb, d1, d2, beta, q1, q2 = decide_distance(v1, v2, calib, sat1, sat2, angle)

            angles.append(angle)
            has_list.append(has_light)
            bias_list.append(sb)

            # הזנה לסדרות:
            if has_light:
                if d_sel is not None and abs(sb) > beta:
                    val = _clamp_report(d_sel)
                    if sb > 0:
                        dist_left.append(val);  dist_right.append(None)
                    else:
                        dist_left.append(None); dist_right.append(val)
                else:
                    left_val = d1 if (d1 is not None and q1 >= Q_USE_BOTH) else None
                    right_val = d2 if (d2 is not None and q2 >= Q_USE_BOTH) else None
                    dist_left.append(_clamp_report(left_val))
                    dist_right.append(_clamp_report(right_val))
            else:
                dist_left.append(None);
                dist_right.append(None)

            # לוג ידידותי
            if has_light:
                disp_d = f"{d_sel:.1f} cm" if d_sel is not None else "— (ambiguous)"
                self._append3(
                    f"Angle={angle:3d}°, V1={v1}, V2={v2}, Bias={sb:+.2f} (β={beta:.2f}), Dist≈{disp_d}\n"
                )
            else:
                self._append3(f"Angle={angle:3d}°, V1={v1}, V2={v2}, No light\n")

            # בקשה למדידה הבאה
            try:
                self.ser.write(b'8')
            except:
                pass

        # ---- זיהוי מקורות משתי סדרות נפרדות (שמאל/ימין) ----
        def _detect_from_series(angles_list, series):
            # ריצות גמישות על סדרת מרחקים אחת (None=אין נקודה)
            flags = [s is not None for s in series]
            runs = segment_runs_with_gap(flags, max_gap=MAX_GAP, min_len=2)
            out = []
            for (i0, i1) in runs:
                subranges = split_run_by_valleys(angles_list, series, i0, i1,
                                                 min_prom=MIN_PROM, min_sep_deg=MIN_SEP_DEG)
                for (u0, u1) in subranges:
                    valid = [(angles_list[k], series[k]) for k in range(u0, u1 + 1)
                             if series[k] is not None]
                    if not valid:
                        continue

                    # זווית מרכזית משוקללת (כבר אצלך):
                    num = sum(a / d for a, d in valid if d > 0)
                    den = sum(1.0 / d for _, d in valid if d > 0)
                    a_ctr = (num / den) if den > 0 else valid[len(valid) // 2][0]

                    # מרחק יציב: מינימום בחלון ±3°
                    WIN = 3.0
                    vals_in_win = [d for a, d in valid if abs(a - a_ctr) <= WIN]
                    d_min = min(vals_in_win) if vals_in_win else min(d for _, d in valid)

                    # הצמדה 5..50
                    d_min = max(5.0, min(50.0, d_min))

                    out.append((a_ctr, d_min))

            return out

        def _merge_sources(srcs, ang_tol=12.0, dist_rel=0.30):
            """מאחד שני מקורות אם קרובים בזווית ובמרחק (יחסי)."""
            srcs = sorted(srcs, key=lambda x: x[0])  # לפי זווית
            out = []
            for a, d in srcs:
                if out and abs(a - out[-1][0]) <= ang_tol and abs(d - out[-1][1]) <= dist_rel * max(d, out[-1][1]):
                    a_old, d_old = out[-1]
                    # ממוצע משוקלל לפי 1/d (קרוב מקבל יותר משקל)
                    w_old, w_new = 1.0 / max(d_old, 1e-6), 1.0 / max(d, 1e-6)
                    a_m = (a_old * w_old + a * w_new) / (w_old + w_new)
                    d_m = min(d_old, d)  # שמרני – הקרוב יותר
                    out[-1] = (a_m, d_m)
                else:
                    out.append((a, d))
            return out

        def _limit_to_top2(srcs):
            """בחירת עד שני מקורות – לפי ציון ~ 1/d (הקרובים ביותר)."""
            scored = [((1.0 / max(d, 1e-6)), a, d) for (a, d) in srcs]
            scored.sort(reverse=True)          # חזק=קרוב קודם
            keep = scored[:2]                  # ← עד שניים
            keep.sort(key=lambda x: x[1])      # החזרה מסודרת לפי זווית
            return [(a, d) for (_, a, d) in keep]


        # אחרי שמחשבים:
        sources_L = _detect_from_series(angles, dist_left)
        sources_R = _detect_from_series(angles, dist_right)
        sources = sources_L + sources_R

        # --- מיזוג + קאפ ---
        sources = self._decide_lights_from_pairs(Ls, Rs, 0.0, 180.0, calib, collapse_center=True)

        if not sources:
            sources = self._force_one_when_variation(Ls, Rs, 0.0, 180.0, calib)

        if len(sources) > 2:
            sources = _limit_to_top2(sources)

        # ---- סיכום ----
        self._append3("\n=== Summary (LDRs) ===\n")
        if not sources:
            self._append3("Final: No light sources found.\n")
        else:
            self._append3(f"Final sources: {len(sources)}\n")
            for idx, (ang, d) in enumerate(sources, 1):
                self._append3(f"  • Obj#{idx}: angle≈{ang:.1f}°, dist≈{d:.1f} cm\n")

        try:
            user_max = int(self.max_obj_cm_var.get())
        except Exception:
            user_max = MAX_DIST_CM
        user_max = max(1, min(MAX_DIST_CM, user_max))
        self.after(0, lambda s=sources, um=user_max: self._plot_sources_polar(s, um))


        # חזרה ל-idle
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'0')
        except:
            pass

####################################################################################################
    def _plot_sources_polar(self, sources, r_max):
        """Half-polar: 0° ימינה, 180° שמאלה. נקודות צהובות ותווית '(dist cm, ang deg)'."""
        if not sources:
            return

        import math
        import matplotlib.pyplot as plt

        thetas = [math.radians(a) for (a, _) in sources]
        rs = [min(d, r_max) for (_, d) in sources]

        # חלון גדול כברירת מחדל
        fig, ax = plt.subplots(figsize=(10, 6), subplot_kw={"projection": "polar"})
        try:
            mgr = plt.get_current_fig_manager()
            # TkAgg (Windows/Linux with Tk)
            if hasattr(mgr, "window"):
                try:
                    mgr.window.state("zoomed")  # Windows
                except Exception:
                    pass
                try:
                    mgr.window.attributes("-zoomed", True)  # חלק מה-Linux
                except Exception:
                    pass
            # Qt backends
            if hasattr(mgr, "window") and hasattr(mgr.window, "showMaximized"):
                try:
                    mgr.window.showMaximized()
                except Exception:
                    pass
        except Exception:
            pass

        # חצי מעגל, 0° בימין
        ax.set_theta_zero_location("E")
        ax.set_theta_direction(1)
        ax.set_thetamin(0)
        ax.set_thetamax(180)
        ax.set_ylim(0, r_max)

        # גריד זוויות בלבד
        ax.set_thetagrids(range(0, 181, 30))

        # קווי רדיאל (ללא תוויות/מספרים)
        ax.set_yticks([r_max * 0.25, r_max * 0.5, r_max * 0.75, r_max])  # קווים בלבד
        ax.set_yticklabels([])  # בלי מספרים
        ax.grid(True, linestyle=":", linewidth=0.8)

        # נקודות – צהוב
        ax.scatter(thetas, rs, c="#FFD400", s=40, zorder=3)

        # תוויות "(45cm, 36 deg)"
        for (th, r), (ang, dist) in zip(zip(thetas, rs), sources):
            ax.text(th, r, f"({dist:.0f}cm, {ang:.0f} deg)",
                    fontsize=9, ha="left", va="bottom")

        ax.set_title("Mode 3 – Light sources (angle & distance)", pad=8, fontsize=11)
        plt.tight_layout()
        plt.show()

    ####################################################################################################
    # ========= Mode 4: Combined (Ultrasonic + LDRs) =========
    def _limit_to_top2_4(self, srcs):
        """בחר עד שני מקורות – הקרובים ביותר (ציון ~ 1/d), ואז סדר לפי זווית."""
        scored = [((1.0 / max(d, 1e-6)), a, d) for (a, d) in srcs]
        scored.sort(reverse=True)         # הקרובים יותר קודם
        keep = scored[:2]                 # עד שניים
        keep.sort(key=lambda x: x[1])     # החזרה מסודרת לפי זווית
        return [(a, d) for (_, a, d) in keep]


    def start_mode4(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return

        self.output4.delete("1.0", "end")
        self.distances4 = [None] * SAMPLES
        self.objects4 = []
        self.sources4 = []

        try:
            # ודא שמצב 2 לא רץ
            self.stop_mode2()

            # סנכרון קצר לאיידל
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'0'); self.ser.flush()
            time.sleep(0.05)

            # כניסה למצב 4
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'4'); self.ser.flush()
        except Exception as e:
            messagebox.showerror("Serial", f"Write failed: {e}")
            return

        t = threading.Thread(target=self._scan_worker_mode4, daemon=True)
        t.start()

    def _scan_worker_mode4(self):
        # --- 0) קריאת כיול LDR (10+10) כמו במצב 3 ---
        def _read_word_or_fail(to=1.0):
            v = self._read_distance_word(total_timeout=to)
            if v is None:
                raise RuntimeError("Timeout while reading from MCU")
            return v

        try:
            ldr1_q = [_read_word_or_fail() for _ in range(10)]
            ldr2_q = [_read_word_or_fail() for _ in range(10)]
        except Exception as e:
            self._append4(f"Calibration read failed: {e}\n")
            return


        try:
            dist_grid, vq1_grid = build_lut(ldr1_q)
            _,         vq2_grid = build_lut(ldr2_q)
        except Exception as e:
            self._append4(f"LUT build failed: {e}\n")
            return

        sat1 = compute_sat_threshold(vq1_grid)
        sat2 = compute_sat_threshold(vq2_grid)
        calib = {"dist_grid": dist_grid, "ldr1": vq1_grid, "ldr2": vq2_grid}

        # --- 1) התחלת סריקה; MCU מתקדם רק אחרי '8' ---
        try:
            self.ser.write(b'8')
        except Exception as e:
            self._append4(f"Wake MCU failed: {e}\n")
            return

        angles = []
        dist_left, dist_right = [], []
        # לאגירה לזוגות Q גולמיים (כמו במצב 3)
        Ls, Rs = [], []


        # משתמש – מקס' מרחק לתצוגה/סינון אובייקטים
        try:
            user_max = int(self.max_obj_cm_var.get())
        except Exception:
            user_max = MAX_DIST_CM
        user_max = max(1, min(MAX_DIST_CM, user_max))

        # --- 2) 60 צעדים: קוראים distance ואז V1,V2 לכל זוית ---
        for i in range(SAMPLES):
            angle = i * STEP_DEG

            triple = self._read_u16x3_with_cs(total_timeout=2.0)
            if triple is None:
                self._append4(f"[{i + 1:02d}/{SAMPLES}] angle={angle:3d}°  timeout/no data\n")
                break
            d, v1, v2 = triple

            # שמאל/ימין לפי כיוון החיישנים (זהה למצב 3)
            v_left  = v1 if LEFT_IS_LDR1 else v2
            v_right = v2 if LEFT_IS_LDR1 else v1
            Ls.append(v_left)
            Rs.append(v_right)



            # --- לוג + אגירה עבור אובייקטים (כמו מצב 1) ---
            if d <= 0 or d > MAX_DIST_CM or d > user_max:
                self.distances4[i] = None
            else:
                self.distances4[i] = d

            # --- החלטת LDR (כמו מצב 3) ---
            has_light, d_sel, sb, d1, d2, beta, q1, q2 = decide_distance(v1, v2, calib, sat1, sat2, angle)
            angles.append(angle)

            if has_light:
                if d_sel is not None and abs(sb) > beta:
                    val = _clamp_report(d_sel)
                    if sb > 0:
                        dist_left.append(val);  dist_right.append(None)
                    else:
                        dist_left.append(None); dist_right.append(val)
                else:
                    left_val  = d1 if (d1 is not None and q1 >= Q_USE_BOTH) else None
                    right_val = d2 if (d2 is not None and q2 >= Q_USE_BOTH) else None
                    dist_left.append(_clamp_report(left_val))
                    dist_right.append(_clamp_report(right_val))
            else:
                dist_left.append(None); dist_right.append(None)

            # בקשה לצעד הבא
            try:
                self.ser.write(b'8')
            except:
                pass

        # --- 3) זיהוי מקורות אור (כמו במצב 3) ---
        
 

        # --- 3) זיהוי מקורות אור (כמו במצב 3 המעודכן) ---
        # משתמשים באותן סדרות Q גולמיות (Ls, Rs) ובאותו קליברציה/ספים כדי לבחור מקורות
        sources = self._decide_lights_from_pairs(Ls, Rs, 0.0, 180.0, calib, collapse_center=True)

        # fallback עדין במקרה שלא נמצא כלום
        if not sources:
            sources = self._force_one_when_variation(Ls, Rs, 0.0, 180.0, calib)

        # הגבלה: עד שני מקורות אור בלבד (לא נוגעים באובייקטים)
        if len(sources) > 2:
            sources = self._limit_to_top2_4(sources)

        self.sources4 = sources



        # --- 4) זיהוי אובייקטים (כמו מצב 1) ---
        self.objects4 = detect_objects(self.distances4)

        # --- 5) סיכום מקורות אור ---
        # (שקט – בלי הדפסות)

        # --- 6) גרף משולב ---
        self.after(0, lambda: self._plot_mode4_combined(self.objects4, self.sources4, user_max))

        # --- 7) חזרה ל-idle ---
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'0')
        except:
            pass

    ####################################################################################################
    def _plot_mode4_combined(self, objects, sources, r_max):
        """Half-polar: 0° ימינה, 180° שמאלה.
        כחול = אובייקטים (distance, angle, width)
        צהוב = מקורות אור (distance, angle)
        """
        import math
        import matplotlib.pyplot as plt

        # אותו גודל כמו במצב 3
        fig, ax = plt.subplots(figsize=(10, 6), subplot_kw={"projection": "polar"})

        # פתיחה בחלון גדול כמו במצב 3 (best-effort לכל backends נפוצים)
        try:
            mgr = plt.get_current_fig_manager()
            if hasattr(mgr, "window"):
                try:
                    mgr.window.state("zoomed")  # Windows/Tk
                except Exception:
                    pass
                try:
                    mgr.window.attributes("-zoomed", True)  # חלק מ-Linux/Tk
                except Exception:
                    pass
            if hasattr(mgr, "window") and hasattr(mgr.window, "showMaximized"):
                try:
                    mgr.window.showMaximized()  # Qt
                except Exception:
                    pass
        except Exception:
            pass

        # חצי מעגל, 0° בימין, נגד כיוון השעון
        ax.set_theta_zero_location("E")
        ax.set_theta_direction(1)
        ax.set_thetamin(0)
        ax.set_thetamax(180)
        ax.set_ylim(0, r_max)

        # גריד זוויות בלבד; קווי רדיאל בלי מספרים (כמו במצב 3)
        ax.set_thetagrids(range(0, 181, 30))
        ax.set_yticks([r_max * 0.25, r_max * 0.5, r_max * 0.75, r_max])
        ax.set_yticklabels([])  # אין מספרים למטה
        ax.grid(True, linestyle=":", linewidth=0.8)

        # --- אובייקטים (כחול) ---
        if objects:
            for o in objects:
                ang = float(o["angle_deg"])
                dist = float(o["distance_cm"])
                wid = float(o["width_cm"])
                th = math.radians(ang)
                r = min(dist, r_max)
                ax.scatter([th], [r], s=40, c="blue", zorder=3)
                # תווית בגודל כמו מצב 3
                ax.text(th, r, f"({dist:.0f}cm, {ang:.0f} deg, {wid:.0f}cm)",
                        fontsize=9, ha="left", va="bottom")

        # --- מקורות אור (צהוב) ---
        if sources:
            for (ang, dist) in sources:
                th = math.radians(float(ang))
                r = min(float(dist), r_max)
                ax.scatter([th], [r], s=40, c="#FFD400", zorder=3)
                ax.text(th, r, f"({dist:.0f}cm, {ang:.0f} deg)",
                        fontsize=9, ha="left", va="bottom")

        ax.set_title("Mode 4 – Objects (blue) & Light sources (yellow)",
                     pad=8, fontsize=11)
        plt.tight_layout()
        plt.show()

    ####################################################################################################
    # ========= Mode 5: Files (PC → MCU) =========


    def _start_script_capture(self):
        # מתחיל האזנה רק אם לא רצה כבר
        if not (self.ser and self.ser.is_open):
            return
        if self.script_cap_run:
            return
        # ליתר ביטחון – נקה זבל קודם
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._append5("Listening for script data (angle + distance)…\n")
        self.script_cap_run = True
        self.script_cap_thread = threading.Thread(
            target=self._script_capture_worker, daemon=True
        )
        self.script_cap_thread.start()

    def _script_capture_worker(self):
        prev_to = getattr(self.ser, "timeout", None)
        seen_frame = False
        try:
            self.ser.timeout = 1.0
            while self.script_cap_run and self.ser and self.ser.is_open:
                b0 = self.ser.read(1)
                if not b0:
                    continue
                x = b0[0]
                if x == 0xFF and not seen_frame:
                    continue
                if x == 0xFF:
                    self._append5("END of run.\n")
                    break
                if x > 180:
                    continue
                hi = self.ser.read(1);
                lo = self.ser.read(1)
                if not hi or not lo:
                    continue
                dist = (hi[0] << 8) | lo[0]
                seen_frame = True
                self._append5(f"angle={x:3d}°, distance={dist}\n")
        finally:
            self.script_cap_run = False
            try:
                self.ser.timeout = prev_to
            except Exception:
                pass

        # === Script compiler ===

    def _compile_script_text(self, text: str) -> bytes:
        import re, struct

        OPC = {
            "inc_lcd": 0x01,
            "dec_lcd": 0x02,
            "rra_lcd": 0x03,
            "set_delay": 0x04,
            "clear_lcd": 0x05,
            "servo_deg": 0x06,
            "servo_scan": 0x07,
            "sleep": 0x08,
        }

        out = bytearray()
        lineno = 0

        def parse_u8(v):
            n = int(v)
            if not (0 <= n <= 255): raise ValueError("u8 out of range")
            return n

        def parse_u16(v):
            n = int(v)
            if not (0 <= n <= 65535): raise ValueError("u16 out of range")
            return n

        def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
            crc = init
            for b in data:
                crc ^= (b << 8)
                for _ in range(8):
                    crc = ((crc << 1) ^ poly) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
            return crc

        for raw_line in text.splitlines():
            lineno += 1
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue

            # טוקניזציה בסיסית: פקודה ושאר ארגומנטים מופרדים ברווחים/פסיקים
            # למשל "servo_scan 10,50"
            m = re.match(r"([A-Za-z_]+)\s*(.*)$", line)
            if not m:
                raise ValueError(f"line {lineno}: cannot parse")
            cmd, rest = m.group(1).lower(), m.group(2).strip()

            if cmd not in OPC:
                raise ValueError(f"line {lineno}: unknown command '{cmd}'")

            op = OPC[cmd]
            if cmd in ("inc_lcd", "dec_lcd", "rra_lcd"):
                if not rest: raise ValueError(f"line {lineno}: missing arg")
                x = parse_u8(rest)
                out += bytes([op, x])

            elif cmd == "set_delay":
                if not rest: raise ValueError(f"line {lineno}: missing arg")
                d = parse_u16(rest)
                out += bytes([op]) + struct.pack("<H", d)

            elif cmd == "clear_lcd":
                if rest: raise ValueError(f"line {lineno}: unexpected arg(s)")
                out += bytes([op])

            elif cmd == "servo_deg":
                if not rest: raise ValueError(f"line {lineno}: missing angle")
                a = parse_u8(rest)
                if a > 180: raise ValueError(f"line {lineno}: angle must be 0..180")
                out += bytes([op, a])

            elif cmd == "servo_scan":
                if not rest: raise ValueError(f"line {lineno}: missing a,b")
                parts = [p.strip() for p in re.split(r"[,\s]+", rest) if p.strip()]
                if len(parts) != 2: raise ValueError(f"line {lineno}: expected two args")
                a0, a1 = parse_u8(parts[0]), parse_u8(parts[1])
                if a0 > 180 or a1 > 180:
                    raise ValueError(f"line {lineno}: angles must be 0..180")
                out += bytes([op, a0, a1])

            elif cmd == "sleep":
                if rest: raise ValueError(f"line {lineno}: unexpected arg(s)")
                out += bytes([op])

        # אופציונלי: CRC‑16 בסוף
        crc = crc16_ccitt(out)
        out += b"\xFE" + struct.pack("<H", crc)  # 0xFE + CRC(LE) כסמן-טיילר

        # אופציונלי: END
        out += b"\xFF"
        return bytes(out)

    def _choose_file(self):
        path = askopenfilename(title="בחר קובץ לשליחה",
                               filetypes=[("All files", "*.*"), ("Text", "*.txt"), ("Script", "*.scr *.bin")])
        if not path:
            return
        self.file_path = path
        self.file_path_entry.delete(0, "end")
        self.file_path_entry.insert(0, path)
        # שם LCD עד 16 תווים (ASCII)
        base = os.path.basename(path)[:16]
        self.file_name16_var.set(base)

        # קביעה אוטומטית של type לפי סיומת (אפשר לשנות ידנית)
        ext = os.path.splitext(path)[1].lower()
        self.file_type_var.set("text" if ext in (".txt", ".log") else "script")

    def _start_send_file(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return
        if not self.file_path:
            messagebox.showwarning("File", "Choose a file first.")
            return

        # אם Mode 2 פעיל – נעצור אותו כדי לא לערבב זרמים בסריאלי
        if self.mode2_active:
            self.stop_mode2()

        # ריצה ברקע כדי לא לחסום GUI
        self.output5.delete("1.0", "end")
        self._append5("Starting send...\n")
        self.mode5_thread = threading.Thread(target=self._send_file_worker, daemon=True)
        self.mode5_thread.start()

    def _send_file_worker(self):
        self.busy_sending = True  # חוסם את ה-capture לכל משך השליחה
        try:
            # --- הכנות (קריאה מהדיסק/קומפילציה) ---
            path = self.file_path
            if not os.path.isfile(path):
                self._append5("Invalid path.\n");
                return

            ftype = self.file_type_var.get().lower()
            tmap = {"text": 0x01, "script": 0x02}
            t = tmap.get(ftype, 0x01)

            name16 = self.file_name16_var.get() or os.path.basename(path)
            base_ascii = name16.encode("ascii", errors="replace")[:16]
            name_field = base_ascii + b"\x00" * (16 - len(base_ascii))

            if t == 0x02:
                with open(path, "r", encoding="utf-8") as f:
                    text = f.read()
                payload = self._compile_script_text(text)
                size = len(payload)
                data_iter = memoryview(payload)
            else:
                size = os.path.getsize(path)
                data_iter = None

            if size > 2048:
                self._append5("קובץ גדול מדי (מקס' 2048 בייט).\n")
                return

            # --- העסקה הסריאלית כולה תחת busy_sending=True ---
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()


                        

            self.ser.write(b'5');
            self._append5("TX: b'5'  (enter state 5)\n")
            self.ser.flush();
            time.sleep(0.05)

            self.ser.write(b'R');
            self._append5("TX: b'R'  (arm receive)\n")
            self.ser.flush()

            hdr = bytes([STX]) + name_field + bytes([t, size & 0xFF, (size >> 8) & 0xFF])
            self.ser.write(hdr)
            self._append5(f"TX: HEADER (type={t:#04x}, size={size})\n")
            self.ser.flush()

            # ACK/NACK על הכותרת
            old_to = self.ser.timeout
            self.ser.timeout = 1.0
            resp = self.ser.read(1)
            self.ser.timeout = old_to
            if not resp:
                self._append5("Header: timeout (no ACK/NACK)\n");
                return
            if resp[0] == ACK:
                self._append5("Header ACK — starting data\n")
            elif resp[0] == NACK:
                self._append5("Header NACK — MCU declined (no space/slot)\n");
                return
            else:
                self._append5(f"Header: unexpected byte 0x{resp[0]:02X}\n");
                return

            # --- DATA (Stop-and-Wait per chunk with CS & ACK) ---
            CHUNK = 64
            sent = 0

            # נכין payload בזיכרון (גם לטקסט, גם לסקריפט):
            if t == 0x02:
                payload = payload         # כבר קומפילטת את הסקריפט למשתנה זה
            else:
                with open(path, "rb") as f:
                    payload = f.read()

            size = len(payload)
            if size != sent:  # sent עדיין 0
                while sent < size:
                    n = min(CHUNK, size - sent)
                    chunk = payload[sent:sent + n]

                    ok = self._send_chunk_and_wait_ack(chunk)
                    if not ok:
                        self._append5("Giving up: too many retries on this chunk. Aborting.\n")
                        try:
                            self.ser.write(b'\x1B')  # ESC – יבטל את ה-session בבקר
                            self.ser.flush()
                        except Exception:
                            pass
                        return

                    sent += n
                    if sent % 512 == 0 or sent == size:
                        self._append5(f"TX data: {sent}/{size}\n")


            self.ser.flush()

            # Final ACK/NACK
            old_to = self.ser.timeout
            self.ser.timeout = 5.0
            final = self.ser.read(1)
            self.ser.timeout = old_to
            if not final:
                self._append5("Timeout: no final ACK/NACK from MCU.\n");
                return
            if final[0] == ACK:
                self._append5("ACK ✅ — הקובץ נשמר בהצלחה ב-MCU\n")
                if t == 0x02:
                    # רק עכשיו נדליק capture (הוא ממילא 'חסום' ע״י busy_sending)
                    self._start_script_capture()
                    self._append5("Ready. Press PB1 on the MCU to RUN — capturing live data…\n")
            elif final[0] == NACK:
                self._append5("NACK ❌ — ה-MCU דחה את הקובץ\n")
            else:
                self._append5(f"Unexpected final response: 0x{final[0]:02X}\n")

        except Exception as e:
            self._append5(f"Error: {e}\n")
        finally:
            self.busy_sending = False  # שחרור ה-capture רק בסוף-סוף



    def _script_capture_worker(self):
        prev_to = getattr(self.ser, "timeout", None)
        seen_frame = False
        try:
            self.ser.timeout = 1.0
            while self.script_cap_run and self.ser and self.ser.is_open:
                if self.busy_sending:
                    time.sleep(0.02)  # ← אל תקרא כלום בזמן שליחה
                    continue

                b0 = self.ser.read(1)
                if not b0:
                    continue
                x = b0[0]
                if x == 0xFF and not seen_frame:
                    continue
                if x == 0xFF:
                    self._append5("END of run.\n")
                    break
                if x > 180:
                    continue
                hi = self.ser.read(1);
                lo = self.ser.read(1)
                if not hi or not lo:
                    continue
                dist = (hi[0] << 8) | lo[0]
                seen_frame = True
                self._append5(f"angle={x:3d}°, distance={dist}\n")
        finally:
            self.script_cap_run = False
            try:
                self.ser.timeout = prev_to
            except Exception:
                pass

    # ========= Mode 6: Calibration (LDR LUT) =========

    def start_mode6(self):
        """שליחת '6' לבקר לצורך כיול (State 6) + לוג בטאב 6 בלבד."""
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Serial", "Connect first.")
            return

        # לעצור מצב 2 אם בטעות פעיל
        try:
            self.stop_mode2()
        except Exception:
            pass

        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # החזרה ל-idle ואז בקשה למצב 6
            self.ser.write(b'0')
            time.sleep(0.05)
            self.ser.write(b'6')
            self.ser.flush()

            self._append6("TX: b'0' (idle)")
            self._append6("TX: b'6' → request calibration (state 6)")
        except Exception as e:
            messagebox.showerror("Serial", f"Write failed: {e}")
            return

        # אופציונלי: קריאת בייט בודד אם ה-MCU מאשר
        try:
            old_to = self.ser.timeout
            self.ser.timeout = 1.0
            resp = self.ser.read(1)
            self.ser.timeout = old_to
            if resp:
                self._append6(f"RX: {resp!r}")
        except Exception:
            pass


if __name__ == "__main__":
    App().mainloop()

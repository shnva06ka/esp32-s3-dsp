"""
test/verify_biquad.py
Checks the browser bandMagAt() math against the C++ _calcCoeffs().
Run:  py -3 test/verify_biquad.py
"""
import math, sys

FS = 44100
failures = 0

def eval_1st_lp(freq, test_f):
    w0 = 2 * math.pi * freq / FS
    K  = math.tan(w0 * 0.5)
    b0 = K / (1.0 + K)
    a1 = (K - 1.0) / (1.0 + K)
    w  = 2 * math.pi * test_f / FS
    c1 = math.cos(w);  s1 = math.sin(w)
    nRe = b0 + b0 * c1; nIm = -b0 * s1
    dRe = 1.0 + a1 * c1; dIm = -a1 * s1
    dM2 = dRe**2 + dIm**2
    if dM2 < 1e-20: return -120.0
    return 10.0 * math.log10((nRe**2 + nIm**2) / dM2 + 1e-12)

def eval_1st_hp(freq, test_f):
    w0 = 2 * math.pi * freq / FS
    K  = math.tan(w0 * 0.5)
    b0 = 1.0 / (1.0 + K)
    a1 = (K - 1.0) / (1.0 + K)
    w  = 2 * math.pi * test_f / FS
    c1 = math.cos(w);  s1 = math.sin(w)
    nRe = b0 - b0 * c1; nIm = b0 * s1
    dRe = 1.0 + a1 * c1; dIm = -a1 * s1
    dM2 = dRe**2 + dIm**2
    if dM2 < 1e-20: return -120.0
    return 10.0 * math.log10((nRe**2 + nIm**2) / dM2 + 1e-12)

def eval_single_biquad(b0, b1, b2, a0, a1, a2, test_f):
    invA0 = 1.0 / a0 if abs(a0) > 1e-12 else 1.0
    B0 = b0 * invA0; B1 = b1 * invA0; B2 = b2 * invA0
    A1 = a1 * invA0; A2 = a2 * invA0
    w  = 2.0 * math.pi * test_f / FS
    c1 = math.cos(w);   s1 = math.sin(w)
    c2 = math.cos(2*w); s2 = math.sin(2*w)
    nRe = B0 + B1*c1 + B2*c2;  nIm = -B1*s1 - B2*s2
    dRe = 1.0 + A1*c1 + A2*c2; dIm = -A1*s1 - A2*s2
    dM2 = dRe**2 + dIm**2
    if dM2 < 1e-20: return -120.0
    return 10.0 * math.log10((nRe**2 + nIm**2) / dM2 + 1e-12)

def band_mag_at(freq, gain, q, test_f, b_type=0, enabled=True, order=2):
    """Exact mirror of the JS bandMagAt() / C++ _calcCoeffs() formula."""
    if not enabled:
        return 0.0

    # types 5/6 are old names for 4th order LP/HP
    if b_type == 5:
        b_type = 3; order = 4
    elif b_type == 6:
        b_type = 4; order = 4

    if (b_type in (0, 1, 2)) and abs(gain) < 1e-4:
        return 0.0

    w0  = 2 * math.pi * freq / FS
    cw0 = math.cos(w0)
    sw0 = math.sin(w0)
    safe_q = max(0.05, q)
    al     = sw0 / (2.0 * safe_q)

    if b_type == 3: # Low Pass
        if order == 1:
            return max(-60.0, min(30.0, eval_1st_lp(freq, test_f)))
        elif order == 3:
            m1 = eval_1st_lp(freq, test_f)
            al_2 = sw0 / (2.0 * 1.0)
            m2 = eval_single_biquad((1.0 - cw0)*0.5, 1.0 - cw0, (1.0 - cw0)*0.5,
                                    1.0 + al_2, -2.0*cw0, 1.0 - al_2, test_f)
            return max(-60.0, min(30.0, m1 + m2))
        elif order == 4:
            al1 = sw0 / (2.0 * 1.30656296)
            m1 = eval_single_biquad((1.0 - cw0)*0.5, 1.0 - cw0, (1.0 - cw0)*0.5,
                                    1.0 + al1, -2.0*cw0, 1.0 - al1, test_f)
            al2 = sw0 / (2.0 * 0.5411961)
            m2 = eval_single_biquad((1.0 - cw0)*0.5, 1.0 - cw0, (1.0 - cw0)*0.5,
                                    1.0 + al2, -2.0*cw0, 1.0 - al2, test_f)
            return max(-60.0, min(30.0, m1 + m2))
        else: # Order 2 (12 dB/oct standard)
            m = eval_single_biquad((1.0 - cw0)*0.5, 1.0 - cw0, (1.0 - cw0)*0.5,
                                   1.0 + al, -2.0*cw0, 1.0 - al, test_f)
            return max(-60.0, min(30.0, m))

    elif b_type == 4: # High Pass
        if order == 1:
            return max(-60.0, min(30.0, eval_1st_hp(freq, test_f)))
        elif order == 3:
            m1 = eval_1st_hp(freq, test_f)
            al_2 = sw0 / (2.0 * 1.0)
            m2 = eval_single_biquad((1.0 + cw0)*0.5, -(1.0 + cw0), (1.0 + cw0)*0.5,
                                    1.0 + al_2, -2.0*cw0, 1.0 - al_2, test_f)
            return max(-60.0, min(30.0, m1 + m2))
        elif order == 4:
            al1 = sw0 / (2.0 * 1.30656296)
            m1 = eval_single_biquad((1.0 + cw0)*0.5, -(1.0 + cw0), (1.0 + cw0)*0.5,
                                    1.0 + al1, -2.0*cw0, 1.0 - al1, test_f)
            al2 = sw0 / (2.0 * 0.5411961)
            m2 = eval_single_biquad((1.0 + cw0)*0.5, -(1.0 + cw0), (1.0 + cw0)*0.5,
                                    1.0 + al2, -2.0*cw0, 1.0 - al2, test_f)
            return max(-60.0, min(30.0, m1 + m2))
        else: # Order 2 (12 dB/oct standard)
            m = eval_single_biquad((1.0 + cw0)*0.5, -(1.0 + cw0), (1.0 + cw0)*0.5,
                                   1.0 + al, -2.0*cw0, 1.0 - al, test_f)
            return max(-60.0, min(30.0, m))

    if b_type == 1: # Low Shelf
        A = 10 ** (gain / 40.0)
        twoSqrtAAlpha = 2.0 * math.sqrt(A) * al
        b0 =    A * ((A + 1.0) - (A - 1.0) * cw0 + twoSqrtAAlpha)
        b1 =  2*A * ((A - 1.0) - (A + 1.0) * cw0)
        b2 =    A * ((A + 1.0) - (A - 1.0) * cw0 - twoSqrtAAlpha)
        a0 =         (A + 1.0) + (A - 1.0) * cw0 + twoSqrtAAlpha
        a1 =   -2 * ((A - 1.0) + (A + 1.0) * cw0)
        a2 =         (A + 1.0) + (A - 1.0) * cw0 - twoSqrtAAlpha
    elif b_type == 2: # High Shelf
        A = 10 ** (gain / 40.0)
        twoSqrtAAlpha = 2.0 * math.sqrt(A) * al
        b0 =    A * ((A + 1.0) + (A - 1.0) * cw0 + twoSqrtAAlpha)
        b1 = -2*A * ((A - 1.0) + (A + 1.0) * cw0)
        b2 =    A * ((A + 1.0) + (A - 1.0) * cw0 - twoSqrtAAlpha)
        a0 =         (A + 1.0) - (A - 1.0) * cw0 + twoSqrtAAlpha
        a1 =    2 * ((A - 1.0) - (A + 1.0) * cw0)
        a2 =         (A + 1.0) - (A - 1.0) * cw0 - twoSqrtAAlpha
    elif b_type == 3: # Low Pass
        b0 = (1.0 - cw0) * 0.5
        b1 =  1.0 - cw0
        b2 = (1.0 - cw0) * 0.5
        a0 =  1.0 + al
        a1 = -2.0 * cw0
        a2 =  1.0 - al
    elif b_type == 4: # High Pass
        b0 =  (1.0 + cw0) * 0.5
        b1 = -(1.0 + cw0)
        b2 =  (1.0 + cw0) * 0.5
        a0 =  1.0 + al
        a1 = -2.0 * cw0
        a2 =  1.0 - al
    else: # 0 = Peaking
        A   = 10 ** (gain / 40.0)
        b0  = 1.0 + al * A
        b1  = -2.0 * cw0
        b2  = 1.0 - al * A
        a0  = 1.0 + al / A
        a1  = -2.0 * cw0
        a2  = 1.0 - al / A

    invA0 = 1.0 / a0 if abs(a0) > 1e-12 else 1.0
    B0 = b0 * invA0
    B1 = b1 * invA0
    B2 = b2 * invA0
    A1 = a1 * invA0
    A2 = a2 * invA0

    # Evaluate H(e^jw) — z^-1 = e^-jw = cos(w) - j*sin(w)
    w  = 2.0 * math.pi * test_f / FS
    c1 = math.cos(w);   s1 = math.sin(w)
    c2 = math.cos(2*w); s2 = math.sin(2*w)

    nRe = B0 + B1*c1 + B2*c2;  nIm = -B1*s1 - B2*s2
    dRe = 1.0 + A1*c1 + A2*c2; dIm = -A1*s1 - A2*s2
    dM2 = dRe**2 + dIm**2

    if dM2 < 1e-20:
        return 0.0
    mag_db = 10.0 * math.log10((nRe**2 + nIm**2) / dM2 + 1e-12)
    return max(-60.0, min(30.0, mag_db))

def near(a, b, tol, label):
    global failures
    ok = abs(a - b) <= tol
    sym = "[PASS]" if ok else "[FAIL]"
    if not ok:
        failures += 1
    print(f"{sym}  {label}:  got={a:.4f}  expected~{b:.4f} ±{tol}")

print("\n=== 1. Centre-frequency gain ===")
for freq, gain, q in [
    (1000, +6,  0.707),
    (1000, -6,  0.707),
    (1000, +12, 0.707),
    (125,  +3,  1.0  ),
    (8000, -9,  2.0  ),
]:
    got = band_mag_at(freq, gain, q, freq)
    near(got, gain, 0.05, f"f={freq}Hz  gain={gain:+}dB  Q={q}")

print("\n=== 2. Flat EQ -> 0 dB everywhere ===")
ISO_FREQS = [31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000]
for tf in [31, 100, 500, 1000, 5000, 16000]:
    total = sum(band_mag_at(f, 0, 0.707, tf) for f in ISO_FREQS)
    near(total, 0.0, 0.01, f"flat EQ at {tf} Hz")

print("\n=== 3. 1 kHz band at +6 dB ===")
gains = [0.0] * 10;  gains[5] = 6.0   # band index 5 = 1 kHz

def total_mag(gains_list, tf):
    return sum(band_mag_at(ISO_FREQS[i], gains_list[i], 0.707, tf) for i in range(10))

near(total_mag(gains, 1000),  +6.0, 0.05, "peak at 1 kHz = +6 dB")
near(total_mag(gains,   31),   0.0, 0.50, "31 Hz ~ 0 dB")
near(total_mag(gains, 16000),  0.0, 0.50, "16 kHz ~ 0 dB")

print("\n=== 4. Disabled band ===")
near(band_mag_at(1000, 12, 0.707, 1000, enabled=False), 0.0, 0.001, "disabled band = 0 dB")

print("\n=== 5. High-Q narrower than Low-Q ===")
narrow = band_mag_at(1000, 6, 8.0, 2000)
wide   = band_mag_at(1000, 6, 0.3, 2000)
ok = narrow < wide
sym = "[PASS]" if ok else "[FAIL]"
if not ok: failures += 1
print(f"{sym}  High-Q at 2kHz={narrow:.3f} dB  Low-Q at 2kHz={wide:.3f} dB  (narrow < wide)")

print("\n=== 6. All bands +6 dB - multi-band interaction ===")
gains_all6 = [6.0] * 10
for tf in [63, 500, 2000, 8000]:
    got = total_mag(gains_all6, tf)
    ok = got > 0
    sym = "[PASS]" if ok else "[FAIL]"
    if not ok: failures += 1
    print(f"{sym}  All bands +6dB at {tf} Hz -> {got:.2f} dB (expect > 0)")

print("\n=== 7. Low Shelf filter ===")
near(band_mag_at(250, 6.0, 0.707, 30, b_type=1), 6.0, 0.25, "Low shelf low-freq passband (+6 dB)")
near(band_mag_at(250, 6.0, 0.707, 250, b_type=1), 3.0, 0.25, "Low shelf corner frequency (+3 dB = half gain)")
near(band_mag_at(250, 6.0, 0.707, 5000, b_type=1), 0.0, 0.25, "Low shelf high-freq stopband (0 dB)")

print("\n=== 8. High Shelf filter ===")
near(band_mag_at(2000, 6.0, 0.707, 100, b_type=2), 0.0, 0.25, "High shelf low-freq stopband (0 dB)")
near(band_mag_at(2000, 6.0, 0.707, 2000, b_type=2), 3.0, 0.25, "High shelf corner frequency (+3 dB = half gain)")
near(band_mag_at(2000, 6.0, 0.707, 10000, b_type=2), 6.0, 0.25, "High shelf high-freq passband (+6 dB)")

print("\n=== 9. Low Pass filter: 1st, 2nd, 3rd, 4th order ===")
near(band_mag_at(1000, 0, 0.707, 50, b_type=3, order=1), 0.0, 0.05, "1st order LP DC (0 dB)")
near(band_mag_at(1000, 0, 0.707, 1000, b_type=3, order=1), -3.01, 0.15, "1st order LP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.707, 2000, b_type=3, order=1), -7.0, 0.5, "1st order LP 1 oct (~ -7 dB)")

near(band_mag_at(1000, 0, 0.7071, 50, b_type=3, order=2), 0.0, 0.05, "2nd order LP DC (0 dB)")
near(band_mag_at(1000, 0, 0.7071, 1000, b_type=3, order=2), -3.01, 0.15, "2nd order LP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.7071, 2000, b_type=3, order=2), -12.3, 0.5, "2nd order LP 1 oct (-12 dB/oct)")

near(band_mag_at(1000, 0, 0.707, 50, b_type=3, order=3), 0.0, 0.05, "3rd order LP DC (0 dB)")
near(band_mag_at(1000, 0, 0.707, 1000, b_type=3, order=3), -3.01, 0.15, "3rd order LP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.707, 2000, b_type=3, order=3), -18.2, 0.6, "3rd order LP 1 oct (-18 dB/oct)")
max_lp_3 = max(band_mag_at(1000, 0, 0.707, f, b_type=3, order=3) for f in range(20, 20000, 50))
ok = max_lp_3 <= 0.001
sym = "[PASS]" if ok else "[FAIL]"
if not ok: failures += 1
print(f"{sym}  3rd order LP max gain check: {max_lp_3:.4f} dB <= 0.0 dB (no volume boost)")

near(band_mag_at(1000, 0, 0.707, 50, b_type=3, order=4), 0.0, 0.05, "4th order LP DC (0 dB)")
near(band_mag_at(1000, 0, 0.707, 1000, b_type=3, order=4), -3.01, 0.15, "4th order LP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.707, 2000, b_type=3, order=4), -24.2, 0.8, "4th order LP 1 oct (-24 dB/oct)")
max_lp_4 = max(band_mag_at(1000, 0, 0.707, f, b_type=3, order=4) for f in range(20, 20000, 50))
ok = max_lp_4 <= 0.001
sym = "[PASS]" if ok else "[FAIL]"
if not ok: failures += 1
print(f"{sym}  4th order LP max gain check: {max_lp_4:.4f} dB <= 0.0 dB (no volume boost)")

print("\n=== 10. High Pass filter: 1st, 2nd, 3rd, 4th order ===")
near(band_mag_at(1000, 0, 0.707, 15000, b_type=4, order=1), 0.0, 0.05, "1st order HP high-freq (0 dB)")
near(band_mag_at(1000, 0, 0.707, 1000, b_type=4, order=1), -3.01, 0.15, "1st order HP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.707, 500, b_type=4, order=1), -7.0, 0.5, "1st order HP 1 oct below (~ -7 dB)")

near(band_mag_at(1000, 0, 0.7071, 15000, b_type=4, order=2), 0.0, 0.05, "2nd order HP high-freq (0 dB)")
near(band_mag_at(1000, 0, 0.7071, 1000, b_type=4, order=2), -3.01, 0.15, "2nd order HP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.7071, 500, b_type=4, order=2), -12.3, 0.5, "2nd order HP 1 oct below (-12 dB/oct)")

near(band_mag_at(1000, 0, 0.707, 15000, b_type=4, order=3), 0.0, 0.05, "3rd order HP high-freq (0 dB)")
near(band_mag_at(1000, 0, 0.707, 1000, b_type=4, order=3), -3.01, 0.15, "3rd order HP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.707, 500, b_type=4, order=3), -18.2, 0.6, "3rd order HP 1 oct below (-18 dB/oct)")
max_hp_3 = max(band_mag_at(1000, 0, 0.707, f, b_type=4, order=3) for f in range(20, 20000, 50))
ok = max_hp_3 <= 0.001
sym = "[PASS]" if ok else "[FAIL]"
if not ok: failures += 1
print(f"{sym}  3rd order HP max gain check: {max_hp_3:.4f} dB <= 0.0 dB (no volume boost)")

near(band_mag_at(1000, 0, 0.707, 15000, b_type=4, order=4), 0.0, 0.05, "4th order HP high-freq (0 dB)")
near(band_mag_at(1000, 0, 0.707, 1000, b_type=4, order=4), -3.01, 0.15, "4th order HP cutoff (-3 dB)")
near(band_mag_at(1000, 0, 0.707, 500, b_type=4, order=4), -24.2, 0.8, "4th order HP 1 oct below (-24 dB/oct)")
max_hp_4 = max(band_mag_at(1000, 0, 0.707, f, b_type=4, order=4) for f in range(20, 20000, 50))
ok = max_hp_4 <= 0.001
sym = "[PASS]" if ok else "[FAIL]"
if not ok: failures += 1
print(f"{sym}  4th order HP max gain check: {max_hp_4:.4f} dB <= 0.0 dB (no volume boost)")

print("\n=== 11. Legacy Flat Type Compatibility ===")
near(band_mag_at(1000, 0, 0, 1000, b_type=5), -3.01, 0.15, "Legacy Flat LP (b_type=5) maps to 4th order (-3 dB)")
near(band_mag_at(1000, 0, 0, 2000, b_type=5), -24.2, 0.8, "Legacy Flat LP 1 oct (-24 dB/oct)")
near(band_mag_at(1000, 0, 0, 1000, b_type=6), -3.01, 0.15, "Legacy Flat HP (b_type=6) maps to 4th order (-3 dB)")
near(band_mag_at(1000, 0, 0, 500, b_type=6), -24.2, 0.8, "Legacy Flat HP 1 oct below (-24 dB/oct)")

print()
if failures == 0:
    print("All tests PASSED")
else:
    print(f"{failures} test(s) FAILED")
sys.exit(failures)




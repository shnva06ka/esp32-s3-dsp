// Checks the browser bandMagAt() math against the C++ peaking filter. Run: node verify_biquad.js

'use strict';

const FS   = 48000;
const PASS = '\x1b[32m✔\x1b[0m';
const FAIL = '\x1b[31m✘\x1b[0m';

let failures = 0;

function assert(cond, msg) {
    if (!cond) { console.log(FAIL, msg); failures++; }
    else        { console.log(PASS, msg); }
}

function near(a, b, tol = 0.1, msg = '') {
    const ok = Math.abs(a - b) <= tol;
    assert(ok, `${msg}  got=${a.toFixed(4)} expected≈${b.toFixed(4)} ±${tol}`);
}

// copy of the browser bandMagAt()
function bandMagAt(b, f) {
    if (!b.enabled || b.gain === 0) return 0;
    const A   = Math.pow(10, b.gain / 40);
    const w0  = 2 * Math.PI * b.freq / FS;
    const cw0 = Math.cos(w0), sw0 = Math.sin(w0);
    const al  = sw0 / (2 * b.q);
    const a0  = 1 + al / A;

    const B0 = (1 + al * A) / a0;
    const B1 = (-2 * cw0)   / a0;
    const B2 = (1 - al * A) / a0;
    const A1 = (-2 * cw0)   / a0;
    const A2 = (1 - al / A) / a0;

    const w  = 2 * Math.PI * f / FS;
    const c1 = Math.cos(w), s1 = Math.sin(w);
    const c2 = Math.cos(2*w), s2 = Math.sin(2*w);

    const nRe = B0 + B1*c1 + B2*c2,  nIm = -B1*s1 - B2*s2;
    const dRe = 1  + A1*c1 + A2*c2,  dIm = -A1*s1 - A2*s2;

    const dM2 = dRe*dRe + dIm*dIm;
    if (dM2 < 1e-20) return 0;
    return 20 * Math.log10(Math.sqrt((nRe*nRe + nIm*nIm) / dM2) + 1e-10);
}

function totalMagAt(bands, f) {
    return bands.reduce((s, b) => s + bandMagAt(b, f), 0);
}

console.log('\n═══ Band centre-frequency gain ═══');
[
    { freq: 1000, gain:  6, q: 0.707 },
    { freq: 1000, gain: -6, q: 0.707 },
    { freq: 1000, gain: 12, q: 0.707 },
    { freq:  125, gain:  3, q: 1.0   },
    { freq: 8000, gain: -9, q: 2.0   },
].forEach(({freq, gain, q}) => {
    const b   = { freq, gain, q, enabled: true };
    const got = bandMagAt(b, freq);
    near(got, gain, 0.05,
        `freq=${freq} Hz, gain=${gain} dB, Q=${q} → peak at centre`);
});

console.log('\n═══ Flat bands → 0 dB everywhere ═══');
const flatBands = [31.5,63,125,250,500,1000,2000,4000,8000,16000]
    .map((f,i) => ({ band:i, freq:f, gain:0, q:0.707, enabled:true }));
[100, 1000, 8000].forEach(f => {
    const got = totalMagAt(flatBands, f);
    near(got, 0, 0.01, `flat EQ at ${f} Hz`);
});

console.log('\n═══ 1 kHz band alone ═══');
const bands1k = flatBands.map((b,i) =>
    i === 5 ? {...b, gain:6} : b);          // band 5 = 1kHz
near(totalMagAt(bands1k, 1000),  6,  0.05, '1kHz +6 dB → peak=+6 dB');
near(totalMagAt(bands1k,   31),  0,  0.5,  '1kHz +6 dB → 31 Hz ≈ 0 dB');
near(totalMagAt(bands1k, 16000), 0,  0.5,  '1kHz +6 dB → 16kHz ≈ 0 dB');

console.log('\n═══ Disabled band passes through ═══');
const disabledB = { freq:1000, gain:12, q:0.707, enabled:false };
near(bandMagAt(disabledB, 1000), 0, 0.001, 'disabled band = 0 dB');

console.log('\n═══ Shelf-like Q interactions ═══');
// high Q is narrower, so less gain at 2x the band freq
const narrowB  = { freq:1000, gain:6, q:8,     enabled:true };
const wideB    = { freq:1000, gain:6, q:0.3,   enabled:true };
const atDouble = 2000;
assert(bandMagAt(narrowB, atDouble) < bandMagAt(wideB, atDouble),
    'High-Q peak narrower than low-Q at 2× freq');

console.log('\n' + (failures === 0
    ? '\x1b[32mAll tests passed ✔\x1b[0m'
    : `\x1b[31m${failures} test(s) FAILED\x1b[0m`));
process.exitCode = failures > 0 ? 1 : 0;


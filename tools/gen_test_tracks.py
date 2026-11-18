"""Generate tagged stereo FLAC test tracks across 44.1..192 kHz in test_track/."""

from pathlib import Path
import numpy as np
import soundfile as sf

TRACKS = [
    ("01_44k1_16bit.flac",  44100,  "PCM_16", "01 - 44.1 kHz 16-bit",  "1"),
    ("02_48k_24bit.flac",   48000,  "PCM_24", "02 - 48 kHz 24-bit",    "2"),
    ("03_88k2_24bit.flac",  88200,  "PCM_24", "03 - 88.2 kHz 24-bit",  "3"),
    ("04_96k_24bit.flac",   96000,  "PCM_24", "04 - 96 kHz 24-bit",    "4"),
    ("05_176k4_24bit.flac", 176400, "PCM_24", "05 - 176.4 kHz 24-bit", "5"),
    ("06_192k_24bit.flac",  192000, "PCM_24", "06 - 192 kHz 24-bit",   "6"),
    ("07_96k_24bit_b.flac", 96000,  "PCM_24", "07 - 96 kHz 24-bit B",  "7"),
]

AMP = 10.0 ** (-12.0 / 20.0)  # -12 dBFS
TONE_SEC = 10.0
SWEEP_SEC = 10.0


def _fade_envelope(n: int, fade_samples: int) -> np.ndarray:
    env = np.ones(n, dtype=np.float64)
    k = min(fade_samples, n // 2)
    if k > 0:
        ramp = 0.5 * (1.0 - np.cos(np.pi * np.arange(k) / k))
        env[:k] *= ramp
        env[-k:] *= ramp[::-1]
    return env


def generate_track(fs: int) -> np.ndarray:
    n_tone = int(round(fs * TONE_SEC))
    n_sweep = int(round(fs * SWEEP_SEC))
    fade = int(round(fs * 0.020))

    t_tone = np.arange(n_tone, dtype=np.float64) / fs
    tone_env = _fade_envelope(n_tone, fade)
    tone_l = AMP * np.sin(2.0 * np.pi * 1000.0 * t_tone) * tone_env
    tone_r = AMP * np.sin(2.0 * np.pi * 1500.0 * t_tone) * tone_env

    t_sweep = np.arange(n_sweep, dtype=np.float64) / fs
    f0 = 20.0
    f1 = 0.9 * (fs * 0.5)
    k_rate = np.log(f1 / f0) / SWEEP_SEC
    phase = 2.0 * np.pi * f0 * (np.exp(k_rate * t_sweep) - 1.0) / k_rate
    sweep_env = _fade_envelope(n_sweep, fade)
    sweep = AMP * np.sin(phase) * sweep_env

    left = np.concatenate([tone_l, sweep])
    right = np.concatenate([tone_r, sweep])
    return np.column_stack([left, right])


def main() -> None:
    out_dir = Path(__file__).resolve().parent.parent / "test_track"
    out_dir.mkdir(parents=True, exist_ok=True)

    for fname, fs, subtype, title, tracknum in TRACKS:
        path = out_dir / fname
        audio = generate_track(fs)
        with sf.SoundFile(path, mode="w", samplerate=fs, channels=2,
                          subtype=subtype, format="FLAC") as f:
            f.title = title
            f.artist = "ESP32-S3-DSP"
            f.album = "Sample Rate Test Suite"
            f.tracknumber = tracknum
            f.write(audio)
        info = sf.info(path)
        print(f"Wrote {path.name}: {info.samplerate} Hz, {info.subtype}, {info.duration:.2f}s")


if __name__ == "__main__":
    main()

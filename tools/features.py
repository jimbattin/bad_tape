#!/usr/bin/env -S uv run
"""Measure and compare reverb renders (Bad Tape reverb measurements).

Every stimulus written by `tools/stimuli.py` starts with ONSET_S of
silence; the features below are measured relative to that onset. Stereo files
fold to mono as (L+R)/2.

Features, by stimulus:
    all          fs, channels, peak_dbfs, rms_dbfs (onset to end),
                 third_octave_db (ISO 31.5 Hz-16 kHz bands, dB re total energy)
    impulse      rt60_s per octave band (125 Hz-8 kHz), predelay_ms,
                 ned_mixing_time_ms, ned_mean_50_150ms (Abel-Huang echo density)
    noise_burst  rt60_s per octave band
    sine1k       mod_cents_std, mod_cents_p5_p95 (pitch modulation re 1 kHz)
    guitar       the common features only

Usage:
    uv run tools/features.py report WAV --stimulus S [--json OUT]
    uv run tools/features.py compare WAV_A WAV_B --stimulus S
"""
import argparse
import json
import math

import numpy as np
from scipy import signal
from scipy.io import wavfile
from scipy.special import erfc

ONSET_S = 0.1
STIMULI = ('impulse', 'noise_burst', 'sine1k', 'guitar')
BURST_S = 0.05
THIRD_OCTAVE_HZ = (31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
                   1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000)
OCTAVE_HZ = (125, 250, 500, 1000, 2000, 4000, 8000)
EPS = 1e-30


def read_wav(path):
    """Return (fs, x[ch, n]) as float64 in -1..1."""
    fs, x = wavfile.read(path)
    if x.dtype == np.int16:
        x = x.astype(np.float64) / 2**15
    elif x.dtype == np.int32:
        x = x.astype(np.float64) / 2**31
    elif x.dtype == np.uint8:
        x = (x.astype(np.float64) - 128) / 128
    else:
        x = x.astype(np.float64)
    if x.ndim == 1:
        x = x[:, None]
    return fs, x.T


def db(p):
    return 10 * math.log10(max(float(p), EPS))


def third_octaves(y, fs):
    spec = np.abs(np.fft.rfft(y))**2
    freqs = np.fft.rfftfreq(len(y), 1 / fs)
    total = spec.sum() + EPS
    out = {}
    for fc in THIRD_OCTAVE_HZ:
        hi = fc * 2**(1 / 6)
        if hi >= 0.45 * fs:
            continue
        band = spec[(freqs >= fc / 2**(1 / 6)) & (freqs < hi)].sum()
        out[str(fc)] = round(db(band / total), 2)
    return out


def rt60(x, fs, start):
    """Per octave band RT60 (s) from the decay after `start` samples, or None."""
    out = {}
    blk = int(0.02 * fs)
    for fc in OCTAVE_HZ:
        if fc * math.sqrt(2) >= 0.45 * fs:
            out[str(fc)] = None
            continue
        sos = signal.butter(4, [fc / math.sqrt(2), fc * math.sqrt(2)], 'bandpass', fs=fs, output='sos')
        y = signal.sosfiltfilt(sos, x)[start:]
        nb = len(y) // blk
        if nb < 4:
            out[str(fc)] = None
            continue
        lv = 10 * np.log10(np.mean(y[:nb * blk].reshape(nb, blk)**2, axis=1) + EPS)
        ip = int(np.argmax(lv))
        p = lv[ip]
        noise = float(np.median(lv[-max(1, int(0.5 * fs) // blk):]))
        lo = max(p - 35, noise + 10)
        hi = p - 5
        idx = []
        for i in range(ip + 1, nb):
            if lv[i] < lo:
                break
            if lv[i] <= hi:
                idx.append(i)
        if len(idx) < 2 or lv[idx[0]] - lv[idx[-1]] < 10:
            out[str(fc)] = None
            continue
        t = np.array(idx) * blk / fs
        slope = np.polyfit(t, lv[idx], 1)[0]
        out[str(fc)] = None if slope >= 0 else round(-60 / slope, 3)
    return out


def ned(x, fs, onset):
    """Normalised echo density per 1 ms hop; returns (centre_ms re onset, ned)."""
    win = int(0.02 * fs)
    hop = int(0.001 * fs)
    w = np.hanning(win)
    w /= w.sum()
    frames = np.lib.stride_tricks.sliding_window_view(x, win)[::hop]
    sigma = np.sqrt(frames**2 @ w)
    val = (np.abs(frames) > sigma[:, None]) @ w / erfc(1 / math.sqrt(2))
    centre_ms = (np.arange(len(frames)) * hop + win / 2 - onset) * 1000 / fs
    return centre_ms, val


def measure(path, stimulus):
    if stimulus not in STIMULI:
        raise ValueError(f'unknown stimulus {stimulus!r}')
    fs, x = read_wav(path)
    channels = x.shape[0]
    x = x.mean(axis=0)
    onset = int(round(ONSET_S * fs))
    y = x[onset:]
    f = {
        'fs': int(fs),
        'channels': int(channels),
        'peak_dbfs': round(20 * math.log10(max(float(np.max(np.abs(x))), 1e-15)), 2),
        'rms_dbfs': round(db(np.mean(y**2)), 2),
        'third_octave_db': third_octaves(y, fs),
    }
    if stimulus == 'impulse':
        f['rt60_s'] = rt60(x, fs, onset)
        peak = float(np.max(np.abs(x[onset + int(0.0015 * fs):])))
        hit = np.nonzero(np.abs(y) >= 0.1 * peak)[0]
        f['predelay_ms'] = round(hit[0] * 1000 / fs, 3) if len(hit) else None
        c, v = ned(x, fs, onset)
        after = (c > 0) & (v >= 0.9)
        f['ned_mixing_time_ms'] = round(float(c[after][0]), 1) if after.any() else None
        sel = (c >= 50) & (c <= 150)
        f['ned_mean_50_150ms'] = round(float(v[sel].mean()), 3) if sel.any() else None
    elif stimulus == 'noise_burst':
        f['rt60_s'] = rt60(x, fs, onset + int(BURST_S * fs))
    elif stimulus == 'sine1k':
        seg = x[onset + int(0.5 * fs):onset + int(2.0 * fs)]
        sos = signal.butter(4, [800, 1250], 'bandpass', fs=fs, output='sos')
        a = signal.hilbert(signal.sosfiltfilt(sos, seg))
        inst = np.diff(np.unwrap(np.angle(a))) * fs / (2 * math.pi)
        m = max(1, int(0.005 * fs))
        inst = np.convolve(inst, np.ones(m) / m, mode='valid')
        cents = 1200 * np.log2(np.maximum(inst, 1e-3) / 1000)
        f['mod_cents_std'] = round(float(np.std(cents)), 3)
        p5, p95 = np.percentile(cents, [5, 95])
        f['mod_cents_p5_p95'] = round(float(p95 - p5), 3)
    return f


def flatten(f):
    rows = []
    for k, v in f.items():
        if isinstance(v, dict):
            rows += [(f'{k}[{b}]', bv) for b, bv in v.items()]
        else:
            rows.append((k, v))
    return rows


def fmt(v):
    return '-' if v is None else f'{v:g}' if isinstance(v, (int, float)) else str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('report')
    r.add_argument('wav')
    r.add_argument('--stimulus', required=True, choices=STIMULI)
    r.add_argument('--json')
    c = sub.add_parser('compare')
    c.add_argument('wav_a')
    c.add_argument('wav_b')
    c.add_argument('--stimulus', required=True, choices=STIMULI)
    args = ap.parse_args()
    if args.cmd == 'report':
        f = measure(args.wav, args.stimulus)
        for k, v in flatten(f):
            print(k, fmt(v))
        if args.json:
            with open(args.json, 'w') as fh:
                json.dump(f, fh, indent=2)
    else:
        a = dict(flatten(measure(args.wav_a, args.stimulus)))
        b = dict(flatten(measure(args.wav_b, args.stimulus)))
        print('feature | A | B | B-A')
        for k in list(a) + [k for k in b if k not in a]:
            va, vb = a.get(k), b.get(k)
            d = vb - va if isinstance(va, (int, float)) and isinstance(vb, (int, float)) else None
            print(f'{k} | {fmt(va)} | {fmt(vb)} | {fmt(round(d, 3) if d is not None else None)}')


if __name__ == '__main__':
    main()

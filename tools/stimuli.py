#!/usr/bin/env -S uv run
"""Write the reference stimuli (mono int32 PCM) to renders/stimuli/.

Each starts with features.ONSET_S of silence:
    impulse      one full-scale sample, 6.0 s total
    noise_burst  50 ms of uniform noise at -12 dBFS peak (seed 1), 6.0 s total
    sine1k       2.0 s of 1 kHz at -12 dBFS peak, 6.1 s total
    guitar       the first 20 s of channel 0 of the 96 kHz WAV given by --guitar (skipped without it)

Files go to renders/stimuli/<name>.wav.

Usage:
    uv run tools/stimuli.py [--guitar WAV]
"""
import argparse
import pathlib

import numpy as np
from scipy.io import wavfile

import features

REPO = pathlib.Path(__file__).resolve().parents[1]
FS = 96000
OUT = REPO / 'renders' / 'stimuli'
FULL = 2**31 - 1
M12DB = 10**(-12 / 20)


def make(name, guitar):
    pre = np.zeros(int(features.ONSET_S * FS))
    if name == 'impulse':
        body = np.zeros(int(6.0 * FS) - len(pre))
        body[0] = 1.0
    elif name == 'noise_burst':
        body = np.zeros(int(6.0 * FS) - len(pre))
        n = int(features.BURST_S * FS)
        body[:n] = np.random.default_rng(1).uniform(-M12DB, M12DB, n)
    elif name == 'sine1k':
        body = np.zeros(int(6.1 * FS) - len(pre))
        n = int(2.0 * FS)
        body[:n] = M12DB * np.sin(2 * np.pi * 1000 * np.arange(n) / FS)
    elif name == 'guitar':
        fs, x = features.read_wav(guitar)
        if fs != FS:
            raise SystemExit(f'stimuli: {guitar} is {fs} Hz, need {FS}')
        body = x[0, :20 * FS]
    return np.concatenate([pre, body])


def to_int32(x):
    return np.clip(np.round(x * 2**31), -2**31, FULL).astype(np.int32)


def write_all(guitar):
    OUT.mkdir(parents=True, exist_ok=True)
    for name in features.STIMULI:
        if name == 'guitar' and guitar is None:
            print('-- skipped guitar (no --guitar WAV)')
            continue
        wavfile.write(OUT / f'{name}.wav', FS, to_int32(make(name, guitar)))
        print(f'-- wrote renders/stimuli/{name}.wav')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--guitar', type=pathlib.Path, help='96 kHz WAV for the guitar stimulus')
    args = ap.parse_args()
    write_all(args.guitar)


if __name__ == '__main__':
    main()

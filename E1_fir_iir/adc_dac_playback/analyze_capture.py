import numpy as np
import matplotlib.pyplot as plt

# ====== config ======
fs = 2000  # exported effective sample rate (48000/24)
raw_file = "raw.dat"
proc_file = "processed.dat"

# ====== robust loader: supports one-value-per-line and delimited text ======
def load_signal(path):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = [ln.strip() for ln in f if ln.strip()]

    values = []
    for ln in lines:
        ln = ln.replace(",", " ")
        parts = ln.split()
        for p in parts:
            try:
                values.append(float(p))
            except ValueError:
                pass

    if not values:
        raise ValueError(f"No numeric samples found in {path}")
    return np.array(values, dtype=np.float64)

raw = load_signal(raw_file).astype(np.float64).flatten()
proc = load_signal(proc_file).astype(np.float64).flatten()

N = min(len(raw), len(proc))
raw = raw[:N]
proc = proc[:N]
t = np.arange(N) / fs

# ====== time domain ======
plt.figure(figsize=(12, 6))
plt.subplot(2,1,1)
plt.plot(t, raw, linewidth=0.8)
plt.title("Raw Signal (Time Domain)")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude (int16)")

plt.subplot(2,1,2)
plt.plot(t, proc, linewidth=0.8, color="orange")
plt.title("Processed Signal (Time Domain)")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude (int16)")
plt.tight_layout()
plt.savefig("time_domain_compare.png", dpi=200)

# ====== frequency domain ======
def mag_spectrum(x, fs):
    x = x - np.mean(x)
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), 1/fs)
    mag = 20*np.log10(np.abs(X) + 1e-12)
    return f, mag

f1, m1 = mag_spectrum(raw, fs)
f2, m2 = mag_spectrum(proc, fs)

plt.figure(figsize=(12, 6))
plt.subplot(2,1,1)
plt.plot(f1, m1, linewidth=0.8)
plt.title("Raw Signal Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude (dB)")
plt.xlim(0, fs/2)

plt.subplot(2,1,2)
plt.plot(f2, m2, linewidth=0.8, color="orange")
plt.title("Processed Signal Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude (dB)")
plt.xlim(0, fs/2)
plt.tight_layout()
plt.savefig("spectrum_compare.png", dpi=200)

print(f"Done. Samples used: {N}")
print("Saved: time_domain_compare.png, spectrum_compare.png")
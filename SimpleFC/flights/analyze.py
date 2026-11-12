"""Summarise a SimpleFC flight-recorder CSV:  python analyze.py flight_xxx.csv"""
import csv, sys, math

rows = [r for r in csv.DictReader(open(sys.argv[1], encoding="utf-8-sig")) if None not in r.values() and None not in r]   # skip cut-off lines
F = lambda k: [float(r[k]) for r in rows]
t = F("t"); n = len(t)
def stats(v): return min(v), sum(v) / len(v), max(v)
def rms(v): return math.sqrt(sum(x * x for x in v) / len(v))
def fmt(name, v, unit=""): mn, av, mx = stats(v); print("  %-22s min %8.2f  avg %8.2f  max %8.2f %s" % (name, mn, av, mx, unit))

dt = (t[-1] - t[0]) / max(n - 1, 1)
print("samples %d, %.1f s (%.0f ms apart)" % (n, t[-1] - t[0], dt * 1000))
thr = F("thr"); flying = [i for i in range(n) if thr[i] > 1150]
print("throttle above 1150 for %.1f s of it" % (len(flying) * dt))
mode = F("mode"); print("ALT HOLD for %.1f s, failsafe-landing %d samples" % (sum(mode) * dt, sum(F("fsLand"))))
print("\nATTITUDE (in flight)")
sel = lambda v: [v[i] for i in flying] or v
roll, pitch, rT, pT = F("roll"), F("pitch"), F("rollT"), F("pitchT")
fmt("roll deg", sel(roll)); fmt("pitch deg", sel(pitch))
er = [roll[i] - rT[i] for i in flying] or [0]; ep = [pitch[i] - pT[i] for i in flying] or [0]
print("  roll  error vs target  mean %+.2f  rms %.2f deg" % (sum(er) / len(er), rms(er)))
print("  pitch error vs target  mean %+.2f  rms %.2f deg" % (sum(ep) / len(ep), rms(ep)))
print("\nGYRO / VIBRATION")
for k in ("gR", "gP", "gY"): print("  %-4s rms %.1f deg/s (in flight)" % (k, rms(sel(F(k)))))
fmt("vibration", F("vib"))
# oscillation: sign changes of the roll/pitch rate per second (a growing wobble shows as a steady high rate)
def zc(v): return sum(1 for a, b in zip(v, v[1:]) if (a > 0) != (b > 0)) / 2 / max(len(v) * dt, 1e-3)
print("  roll-rate zero crossings %.1f /s, pitch %.1f /s" % (zc(sel(F("gR"))), zc(sel(F("gP")))))
print("\nMOTORS")
ms = [F("m%d" % k) for k in (1, 2, 3, 4)]
for k in range(4):
    v = sel(ms[k]); sat = sum(1 for x in v if x >= 1990)
    print("  M%d  avg %4.0f  max %4.0f  at 2000: %d samples" % (k + 1, sum(v) / len(v), max(v), sat))
avg = [sum(sel(m)) / len(sel(m)) for m in ms]
print("  front (M2+M4) - rear (M1+M3) = %+.0f us   right (M1+M2) - left (M3+M4) = %+.0f us"
      % ((avg[1] + avg[3]) - (avg[0] + avg[2]), (avg[0] + avg[1]) - (avg[2] + avg[3])))
print("  (CW M1+M4) - (CCW M2+M3) = %+.0f us  (yaw imbalance)" % ((avg[0] + avg[3]) - (avg[1] + avg[2])))
print("\nBATTERY / LOOP / RADIO")
vb = F("vbat"); print("  vbat start %.2f  end %.2f  min %.2f V" % (vb[0], vb[-1], min(vb)))
fmt("ctrl loop us", F("ctrl_us"))
fmt("rssi dBm", F("rssi")); fmt("link quality %", F("lq"))
print("\nALTITUDE")
alt, altT, vel = F("alt"), F("altT"), F("vel")
fmt("altitude m", alt); fmt("vertical speed m/s", vel)
hold = [i for i in range(n) if mode[i] > 0.5 and thr[i] > 1150]
if hold:
    e = [alt[i] - altT[i] for i in hold]
    print("  ALT HOLD error  mean %+.2f  rms %.2f  max %.2f m" % (sum(e) / len(e), rms(e), max(abs(x) for x in e)))
print("\nHOVER")
steady = [i for i in flying if abs(vel[i]) < 0.3 and abs(roll[i]) < 10 and abs(pitch[i]) < 10]
if steady:
    print("  steady samples %d: throttle avg %.0f us, vbat avg %.2f V" % (len(steady), sum(thr[i] for i in steady) / len(steady),
                                                                     sum(vb[i] for i in steady) / len(steady)))
    print("  stick in hover: roll %+.0f  pitch %+.0f us (centre 1500) -> drift the pilot held against"
          % (sum(F("rcR")[i] for i in steady) / len(steady) - 1500, sum(F("rcP")[i] for i in steady) / len(steady) - 1500))

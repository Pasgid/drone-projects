"""Altitude / barometer behaviour in a SimpleFC flight log:  python baro_check.py flight.csv"""
import csv, sys, math
rows = [r for r in csv.DictReader(open(sys.argv[1], encoding="utf-8-sig")) if None not in r.values() and None not in r]   # skip cut-off lines
F = lambda k: [float(r[k]) for r in rows]
t, alt, altT, vel, thr, mode = F("t"), F("alt"), F("altT"), F("vel"), F("thr"), F("mode")
m = [F("m%d" % k) for k in (1, 2, 3, 4)]
mot = [sum(x[i] for x in m) / 4 for i in range(len(t))]
n, dt = len(t), 0.02
def mean(v): return sum(v) / len(v) if v else float("nan")
def std(v): a = mean(v); return math.sqrt(mean([(x - a) ** 2 for x in v])) if v else float("nan")

# segments: ground (motors idle / stick low) vs air
air = [mot[i] > 1250 for i in range(n)]
segs, s = [], 0
for i in range(1, n + 1):
    if i == n or air[i] != air[s]:
        segs.append((s, i - 1, air[s])); s = i
print("segments (A = flying, G = ground/idle):")
for a, b, f in segs:
    if b - a < 5: continue
    print("  %s %5.1f-%5.1f s  alt %+.2f -> %+.2f m  (mean %+.2f)" % ("A" if f else "G", t[a], t[b], alt[a], alt[b], mean(alt[a:b + 1])))

# 1) noise: altitude minus its 1 s moving average, while flying steady in ALT HOLD
w = 50
hold = [i for i in range(w, n - w) if mode[i] > 0.5 and air[i] and abs(altT[i] - altT[i - 1]) < 1e-6]
hf = [alt[i] - mean(alt[i - w // 2:i + w // 2]) for i in hold]
print("\nALT HOLD steady samples: %d" % len(hold))
print("  short-term noise of the estimate (vs 1 s average): %.3f m rms" % std(hf))
print("  vertical speed noise: %.3f m/s rms" % std([vel[i] for i in hold]))
err = [alt[i] - altT[i] for i in hold]
print("  hold error: mean %+.3f  rms %.3f  max %.3f m" % (mean(err), std(err) if err else 0, max(map(abs, err)) if err else 0))

# 2) prop wash / ground effect: does the estimate jump when the motors spin up / down on the ground?
print("\nthrottle steps (motor average change > 120 us within 0.4 s) and the altitude reaction:")
i = 20
while i < n - 40:
    d = mot[i + 20] - mot[i]
    if abs(d) > 120:
        a0 = mean(alt[i - 10:i]); a1 = mean(alt[i + 20:i + 40])
        print("  t=%5.1f s  motors %+4.0f us  altitude %+.2f m in 0.8 s (mode %s)" % (t[i], d, a1 - a0, "ALT" if mode[i] > 0.5 else "SELF"))
        i += 50
    else:
        i += 1

# 3) drift: ground readings before take-off and after landing
g = [(a, b) for a, b, f in segs if not f and b - a >= 25]
if len(g) >= 2:
    print("\nground level drift: first ground %+.2f m, last ground %+.2f m -> %+.2f m over %.0f s"
          % (mean(alt[g[0][0]:g[0][1]]), mean(alt[g[-1][0]:g[-1][1]]), mean(alt[g[-1][0]:g[-1][1]]) - mean(alt[g[0][0]:g[0][1]]),
             t[g[-1][0]] - t[g[0][1]]))

# 4) correlation of the hold error with motor output (pressure under the props)
if len(hold) > 50:
    e = err; mo = [mot[i] for i in hold]
    me, mm = mean(e), mean(mo)
    c = sum((x - me) * (y - mm) for x, y in zip(e, mo)) / (len(e) * std(e) * std(mo) + 1e-9)
    print("\ncorrelation hold-error vs motor output: %+.2f (strongly negative = prop wash pushes the reading)" % c)

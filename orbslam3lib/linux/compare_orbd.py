#!/usr/bin/env python3
"""Compare two feature-extractor dumps (.orbd, see dsp/sim/sim_main.cpp).

    compare_orbd.py <reference.orbd> <test.orbd>

Typically the reference is the CPU backend (orb_dump) and the test is the DSP
(hexagon-sim or the CDSP). Exact equality is not expected (the DSP's FAST is an
approximation and its pyramid is fixed-point); this prints statistics:
  - keypoints per level, and how many test keypoints have a reference keypoint
    on the same level within 1.5 pixels of that level (and vice versa);
  - for those pairs: angle difference and descriptor Hamming distance (a random
    pair of ORB descriptors is ~128 bits apart, the same patch < 30);
  - stereo matches: the share of left stereo points whose best right match
    (distance < 70, the CPU side's limit) lands on the same image point in both.
"""
import math
import struct
import sys


def load(path):
    with open(path, "rb") as f:
        data = f.read()
    assert data[:4] == b"ORBD", path
    nl, nr, ml, mr = struct.unpack_from("<4i", data, 4)
    off = 20
    eyes = []
    for n in (nl, nr):
        cols = []
        for _ in range(4):
            cols.append(struct.unpack_from("<%di" % n, data, off))
            off += 4 * n
        desc = [data[off + 32 * i: off + 32 * i + 32] for i in range(n)]
        off += 32 * n
        eyes.append({"x": cols[0], "y": cols[1], "a": cols[2], "l": cols[3], "d": desc})
    k = nl - ml
    ind = struct.unpack_from("<%dh" % k, data, off)
    d1 = struct.unpack_from("<%dh" % k, data, off + 2 * k)
    return {"n": (nl, nr), "mono": (ml, mr), "eyes": eyes, "ind": ind, "d1": d1}


SCALE = [1.0, 5 / 4, 5 / 3, 5 / 2.44948974278, 5 / 2, 5 / 1.58740105197, 5 / 1.25992104989]


def angle(enc):
    c = struct.unpack("b", bytes([enc & 0xFF]))[0]
    s = struct.unpack("b", bytes([(enc >> 8) & 0xFF]))[0]
    return math.degrees(math.atan2(s, c))


def hamming(a, b):
    return bin(int.from_bytes(a, "little") ^ int.from_bytes(b, "little")).count("1")


def pair(ref, test):
    """test index -> nearest reference index on the same level within 1.5 level pixels."""
    grid = {}
    for i in range(len(ref["x"])):
        lvl = ref["l"][i]
        cell = (lvl, int(ref["x"][i] / SCALE[lvl] // 4), int(ref["y"][i] / SCALE[lvl] // 4))
        grid.setdefault(cell, []).append(i)
    out = {}
    for j in range(len(test["x"])):
        lvl = test["l"][j]
        if not 0 <= lvl < len(SCALE):
            continue
        tx, ty = test["x"][j] / SCALE[lvl], test["y"][j] / SCALE[lvl]
        best, bd = None, 1.5
        for cx in (int(tx // 4) - 1, int(tx // 4), int(tx // 4) + 1):
            for cy in (int(ty // 4) - 1, int(ty // 4), int(ty // 4) + 1):
                for i in grid.get((lvl, cx, cy), ()):
                    d = math.hypot(ref["x"][i] / SCALE[lvl] - tx, ref["y"][i] / SCALE[lvl] - ty)
                    if d <= bd:
                        best, bd = i, d
        if best is not None:
            out[j] = best
    return out


def main():
    ref, test = load(sys.argv[1]), load(sys.argv[2])
    print("points (left, right): reference %s mono %s, test %s mono %s" % (ref["n"], ref["mono"], test["n"], test["mono"]))
    for e, name in enumerate(("left", "right")):
        r, t = ref["eyes"][e], test["eyes"][e]
        per_level = [(sum(1 for l in r["l"] if l == k), sum(1 for l in t["l"] if l == k)) for k in range(7)]
        print("%s per level (ref/test): %s" % (name, " ".join("%d/%d" % p for p in per_level)))
        bad = sum(1 for j in range(len(t["x"])) if not (15 <= t["x"][j] < 625 and 15 <= t["y"][j] < 385))
        p = pair(r, t)
        rp = pair(t, r)
        if not p:
            print("%s: no common keypoints" % name)
            continue
        ham = sorted(hamming(r["d"][i], t["d"][j]) for j, i in p.items())
        da = sorted(abs((angle(r["a"][i]) - angle(t["a"][j]) + 180) % 360 - 180) for j, i in p.items())
        print("%s: %.1f%% of test points near a reference point, %.1f%% of reference points found; "
              "%d test points within 15 px of the border" % (name, 100 * len(p) / len(t["x"]), 100 * len(rp) / len(r["x"]), bad))
        print("%s: pairs: descriptor Hamming median %d (p90 %d), angle diff median %.1f deg (p90 %.1f)" % (
            name, ham[len(ham) // 2], ham[int(len(ham) * 0.9)], da[len(da) // 2], da[int(len(da) * 0.9)]))

    # Stereo: compare the best right match of each left stereo point by image position.
    def matches(dump):
        l, r = dump["eyes"]
        ml, mr = dump["mono"]
        out = {}
        for i in range(dump["n"][0] - ml):
            k = dump["ind"][i]
            if dump["d1"][i] >= 70 or dump["d1"][i] <= 0 or not 0 <= k < dump["n"][1] - mr:
                continue
            out[(l["x"][ml + i], l["y"][ml + i], l["l"][ml + i])] = (r["x"][mr + k], r["y"][mr + k])
        return out

    mr_, mt = matches(ref), matches(test)
    common = [k for k in mt if k in mr_]
    same = sum(1 for k in common if abs(mr_[k][0] - mt[k][0]) <= 2 and abs(mr_[k][1] - mt[k][1]) <= 2)
    print("stereo: %d reference / %d test matches below distance 70; on the %d left points in both, %d (%.1f%%) pick the same right point"
          % (len(mr_), len(mt), len(common), same, 100 * same / max(1, len(common))))


if __name__ == "__main__":
    main()

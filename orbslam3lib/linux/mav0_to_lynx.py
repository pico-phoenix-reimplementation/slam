#!/usr/bin/env python3
"""
Build the Lynx ORB-SLAM3 settings file for the PICO 4 from its factory
tracking-camera calibration (persist pvr/mav0, Kalibr omni + radtan).

ORB-SLAM3 has no omni model, so for each camera the omni+radtan projection is
sampled over the field of view the 640x480 sensor sees and a Kannala-Brandt
(KB8) model is fitted to it (Levenberg-Marquardt, pure Python). The Lynx
pipeline works on 640x400 per eye, so the principal point is shifted by the
centre crop (rows 40..439). T_c1_c2 comes from the two T_BS extrinsics.

  mav0_to_lynx.py <mav0 dir> <left cam> <right cam> <out.yaml>
  mav0_to_lynx.py stockdump/mav0 cam3 cam2 pico4-lynx.yaml

PICO 4: cam3 (bottom-left) and cam2 (bottom-right) are the stereo pair: nearly
parallel optical axes, 11 cm baseline.
"""
import math
import os
import re
import sys

SENSOR_W, SENSOR_H = 640, 480
EYE_W, EYE_H = 640, 400
CROP_Y = (SENSOR_H - EYE_H) // 2


def read_mav0(path):
    text = open(path).read()

    def array(key):
        m = re.search(key + r"\s*:\s*\[([^\]]*)\]", text)
        return [float(v) for v in m.group(1).replace("\n", " ").split(",")]

    t = re.search(r"T_BS:.*?data:\s*\[([^\]]*)\]", text, re.S).group(1)
    T = [float(v) for v in t.replace("\n", " ").split(",")]
    xi, fu, fv, pu, pv = array("intrinsics")
    k1, k2, p1, p2 = array("distortion_coefficients")
    return {"T": [T[0:4], T[4:8], T[8:12], T[12:16]],
            "omni": (xi, fu, fv, pu, pv), "radtan": (k1, k2, p1, p2)}


def omni_project(cam, x, y, z):
    xi, fu, fv, pu, pv = cam["omni"]
    k1, k2, p1, p2 = cam["radtan"]
    n = math.sqrt(x * x + y * y + z * z)
    d = z / n + xi
    if d <= 1e-9:
        return None
    mx, my = x / n / d, y / n / d
    r2 = mx * mx + my * my
    radial = 1 + k1 * r2 + k2 * r2 * r2
    dx = 2 * p1 * mx * my + p2 * (r2 + 2 * mx * mx)
    dy = p1 * (r2 + 2 * my * my) + 2 * p2 * mx * my
    return fu * (mx * radial + dx) + pu, fv * (my * radial + dy) + pv


def kb8_project(p, theta, phi):
    fx, fy, cx, cy, k1, k2, k3, k4 = p
    t2 = theta * theta
    rd = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
    return fx * rd * math.cos(phi) + cx, fy * rd * math.sin(phi) + cy


def samples(cam):
    """(theta, phi, u, v) for rays whose omni projection lands on the sensor."""
    out = []
    for i in range(1, 91):
        theta = math.radians(i)
        for j in range(72):
            phi = 2 * math.pi * j / 72
            x, y, z = math.sin(theta) * math.cos(phi), math.sin(theta) * math.sin(phi), math.cos(theta)
            uv = omni_project(cam, x, y, z)
            if uv and 0 <= uv[0] < SENSOR_W and 0 <= uv[1] < SENSOR_H:
                out.append((theta, phi, uv[0], uv[1]))
    return out


def solve(A, b):
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[piv] = M[piv], M[c]
        for r in range(n):
            if r != c and M[c][c] != 0:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def fit_kb8(cam):
    pts = samples(cam)
    xi, fu, fv, pu, pv = cam["omni"]
    p = [fu / (1 + xi), fv / (1 + xi), pu, pv, 0.0, 0.0, 0.0, 0.0]

    def residuals(p):
        res = []
        for theta, phi, u, v in pts:
            pu_, pv_ = kb8_project(p, theta, phi)
            res += [pu_ - u, pv_ - v]
        return res

    lam = 1e-3
    r = residuals(p)
    cost = sum(e * e for e in r)
    for _ in range(100):
        J = []
        for k in range(8):
            h = 1e-6 * max(1.0, abs(p[k]))
            q = p[:]
            q[k] += h
            J.append([(a - b) / h for a, b in zip(residuals(q), r)])
        A = [[sum(J[i][m] * J[j][m] for m in range(len(r))) for j in range(8)] for i in range(8)]
        g = [sum(J[i][m] * r[m] for m in range(len(r))) for i in range(8)]
        while True:
            Ad = [row[:] for row in A]
            for i in range(8):
                Ad[i][i] *= 1 + lam
            step = solve(Ad, [-x for x in g])
            q = [a + b for a, b in zip(p, step)]
            rq = residuals(q)
            cq = sum(e * e for e in rq)
            if cq < cost:
                p, r, cost, lam = q, rq, cq, lam * 0.3
                break
            lam *= 10
            if lam > 1e12:
                break
        if lam > 1e12 or max(abs(s) for s in step) < 1e-10:
            break
    rms = math.sqrt(cost / len(pts))
    max_theta = max(t for t, _, _, _ in pts)
    return p, rms, len(pts), max_theta


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inverse_rigid(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(R[i][k] * T[k][3] for k in range(3)) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    mav0, left, right, out = sys.argv[1:]
    def calib_path(c):
        # stockdump/mav0/camN.yaml, or persist's layout mav0/camN/sensor.yaml
        flat = "%s/%s.yaml" % (mav0, c)
        return flat if os.path.exists(flat) else "%s/%s/sensor.yaml" % (mav0, c)

    cams = [read_mav0(calib_path(c)) for c in (left, right)]
    fits = []
    for name, cam in zip((left, right), cams):
        p, rms, n, max_theta = fit_kb8(cam)
        print("%s: KB8 fit rms %.3f px over %d rays (up to %.0f deg)" % (name, rms, n, math.degrees(max_theta)))
        fits.append(p)

    T_c1_c2 = matmul(inverse_rigid(cams[0]["T"]), cams[1]["T"])
    baseline = math.sqrt(sum(T_c1_c2[i][3] ** 2 for i in range(3)))
    print("baseline %.4f m" % baseline)

    with open(out, "w") as f:
        f.write("%YAML:1.0\n")
        f.write("# Generated by mav0_to_lynx.py from %s: left = %s, right = %s.\n" % (mav0, left, right))
        f.write("# Body frame of the factory calibration: x up, y left, z back.\n")
        for name, T in (("Body.T_b_c1", cams[0]["T"]),):
            f.write("# %s (left camera in the body frame):\n" % name)
            for row in T:
                f.write("#   %s\n" % " ".join("%.9f" % v for v in row))
        f.write('File.version: "1.0"\n\nCamera.type: "KannalaBrandt8"\n')
        for i, p in enumerate(fits, 1):
            fx, fy, cx, cy, k1, k2, k3, k4 = p
            f.write("Camera%d.fx: %.6f\nCamera%d.fy: %.6f\nCamera%d.cx: %.6f\nCamera%d.cy: %.6f\n"
                    % (i, fx, i, fy, i, cx, i, cy - CROP_Y))
            f.write("Camera%d.k1: %.9f\nCamera%d.k2: %.9f\nCamera%d.k3: %.9f\nCamera%d.k4: %.9f\n"
                    % (i, k1, i, k2, i, k3, i, k4))
        f.write("Camera.width: %d\nCamera.height: %d\n" % (EYE_W, EYE_H))
        f.write("Stereo.T_c1_c2: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [")
        f.write(",\n    ".join(", ".join("%.9f" % v for v in row) for row in T_c1_c2))
        f.write("]\n")
        # Read by pico4-slam (ORB-SLAM3 ignores it): left camera -> body, to report the head pose.
        f.write("Body.T_b_c1: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: d\n  data: [")
        f.write(",\n    ".join(", ".join("%.9f" % v for v in row) for row in cams[0]["T"]))
        f.write("]\n")
        f.write("Pico4.cropY: %d\n" % CROP_Y)
        f.write("Camera1.overlappingBegin: 0\nCamera1.overlappingEnd: 639\n")
        f.write("Camera2.overlappingBegin: 0\nCamera2.overlappingEnd: 639\n")
        # Head frames only (the MCU interleaves head and hand exposures).
        f.write("Camera.fps: 30\nCamera.RGB: 0\nStereo.ThDepth: 35.0\n")
        f.write("ORBextractor.nFeatures: 2000\nORBextractor.scaleFactor: 1.25\nORBextractor.nLevels: 8\n")
        f.write("ORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n")
        f.write("Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\nViewer.GraphLineWidth: 1.0\n")
        f.write("Viewer.PointSize: 2.0\nViewer.CameraSize: 0.05\nViewer.CameraLineWidth: 2.0\n")
        f.write("Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\nViewer.ViewpointZ: -1.8\nViewer.ViewpointF: 500.0\n")
    print("wrote", out)


if __name__ == "__main__":
    main()

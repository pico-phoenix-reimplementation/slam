// Runs one stereo frame through the DSP interface and writes the input and the
// result: the CPU reference (orbslam3_ref.cpp, target orb_dump) or the CDSP
// (target orb_dump_cdsp, LYNX_BACKEND=hexagonrpc, run under hexagonrpcd -p with
// ORB_DUMP_ARGS since -p passes no arguments). Compare with
// linux/compare_orbd.py; dsp/sim produces the same format in hexagon-sim.
//
//   orb_dump <left.png> <right.png> <out.raw> <out.orbd>
//   orb_dump <in.raw> <out.orbd>
//
// The PNG pair is placed in the 640x400 eyes like lynx_stereo does (centre
// crop/pad); out.raw is that 1280x400 side-by-side frame. The .orbd format is
// described in dsp/sim/sim_main.cpp.

#include "orbslam3.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

static constexpr int W = 640, H = 400, MAX_POINTS = 20000;

static void place_eye(const cv::Mat& src, cv::Mat dst) {
    dst.setTo(0);
    const int w = std::min(src.cols, W), h = std::min(src.rows, H);
    const int sx = (src.cols - w) / 2, sy = (src.rows - h) / 2;
    const int dx = (W - w) / 2, dy = (H - h) / 2;
    src(cv::Rect(sx, sy, w, h)).copyTo(dst(cv::Rect(dx, dy, w, h)));
}

int main(int argc, char** argv) {
    // Under "hexagonrpcd -p" there are no arguments: take them from ORB_DUMP_ARGS.
    std::vector<std::string> envArgs;
    std::vector<char*> envArgv;
    if (argc == 1 && getenv("ORB_DUMP_ARGS")) {
        std::istringstream in(getenv("ORB_DUMP_ARGS"));
        envArgs.push_back(argv[0]);
        for (std::string a; in >> a;)
            envArgs.push_back(a);
        for (std::string& a : envArgs)
            envArgv.push_back(&a[0]);
        argc = (int)envArgv.size();
        argv = envArgv.data();
    }
    cv::Mat packed(H, 2 * W, CV_8UC1);
    const char* outPath;
    if (argc == 5) {
        cv::Mat l = cv::imread(argv[1], cv::IMREAD_GRAYSCALE), r = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
        if (l.empty() || r.empty()) {
            fprintf(stderr, "cannot read the PNGs\n");
            return 1;
        }
        place_eye(l, packed(cv::Rect(0, 0, W, H)));
        place_eye(r, packed(cv::Rect(W, 0, W, H)));
        FILE* raw = fopen(argv[3], "wb");
        fwrite(packed.data, 1, 2 * W * H, raw);
        fclose(raw);
        outPath = argv[4];
    } else if (argc == 3) {
        FILE* raw = fopen(argv[1], "rb");
        if (!raw || fread(packed.data, 1, 2 * W * H, raw) != (size_t)(2 * W * H)) {
            fprintf(stderr, "cannot read %s\n", argv[1]);
            return 1;
        }
        fclose(raw);
        outPath = argv[2];
    } else {
        fprintf(stderr, "usage: %s <left.png> <right.png> <out.raw> <out.orbd> | <in.raw> <out.orbd>\n", argv[0]);
        return 1;
    }

    std::vector<int> lx(MAX_POINTS), ly(MAX_POINTS), la(MAX_POINTS), ll(MAX_POINTS);
    std::vector<int> rx(MAX_POINTS), ry(MAX_POINTS), ra(MAX_POINTS), rl(MAX_POINTS);
    std::vector<unsigned char> ld(MAX_POINTS * 32), rd(MAX_POINTS * 32);
    std::vector<short> ind(MAX_POINTS), d1(MAX_POINTS), d2(MAX_POINTS);
    int nl, nr, ml, mr;
    remote_handle64 h;
    if (orbslam3_open(orbslam3_URI "&_dom=cdsp", &h) != 0) {
        fprintf(stderr, "orbslam3_open failed\n");
        return 1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const int ret = orbslam3_extractFeatures(h, packed.data, 2 * W * H, W, H, 2 * W, 20, 0, W - 1, 0, W - 1,
                             &nl, lx.data(), MAX_POINTS, ly.data(), MAX_POINTS, la.data(), MAX_POINTS, ll.data(), MAX_POINTS, ld.data(), MAX_POINTS * 32,
                             &nr, rx.data(), MAX_POINTS, ry.data(), MAX_POINTS, ra.data(), MAX_POINTS, rl.data(), MAX_POINTS, rd.data(), MAX_POINTS * 32,
                             &ml, &mr, ind.data(), MAX_POINTS, d1.data(), MAX_POINTS, d2.data(), MAX_POINTS);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (ret != 0) {
        fprintf(stderr, "orbslam3_extractFeatures returned %d\n", ret);
        return 1;
    }

    FILE* out = fopen(outPath, "wb");
    int header[4] = {nl, nr, ml, mr};
    fwrite("ORBD", 1, 4, out);
    fwrite(header, 4, 4, out);
    for (int eye = 0; eye < 2; eye++) {
        const int n = eye ? nr : nl;
        const std::vector<int>* v[4] = {eye ? &rx : &lx, eye ? &ry : &ly, eye ? &ra : &la, eye ? &rl : &ll};
        for (auto* a : v)
            fwrite(a->data(), 4, n, out);
        fwrite((eye ? rd : ld).data(), 32, n, out);
    }
    fwrite(ind.data(), 2, nl - ml, out);
    fwrite(d1.data(), 2, nl - ml, out);
    fwrite(d2.data(), 2, nl - ml, out);
    fclose(out);
    printf("left %d (mono %d), right %d (mono %d), %.2f ms\n", nl, ml, nr, mr, ms);
    orbslam3_close(h);
    return 0;
}

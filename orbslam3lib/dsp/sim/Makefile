# hexagon-sim build of the DSP extractor (no SDK needed: sim/include has
# stand-ins for the few SDK headers; the code itself is ../src).
#
#   make HEXAGON_TOOLS=~/Qualcomm/HEXAGON_Tools/8.4.06/Tools
#   make run IN=frame.raw OUT=frame.orbd
HEXAGON_TOOLS ?= $(HOME)/Qualcomm/HEXAGON_Tools/8.4.06/Tools
CXX = $(HEXAGON_TOOLS)/bin/hexagon-clang++
CC = $(HEXAGON_TOOLS)/bin/hexagon-clang
SIM = $(HEXAGON_TOOLS)/bin/hexagon-sim
ARCH = -mv66 -mhvx -mhvx-length=128B
CXXFLAGS = $(ARCH) -O2 -g -std=c++14 -DORBSLAM_DSP_SINGLE_THREAD -DORBSLAM_DSP_SIM $(DEFS) -Iinclude -I../src \
	-Wno-unused-variable -Wno-unused-but-set-variable -Wno-format
SRCS = sim_main.cpp ../src/orbslam_dsp.cpp ../src/orbslam_dsp_pyramid.cpp ../src/orbslam_dsp_fast.cpp \
	../src/orbslam_dsp_sort.cpp ../src/orbslam_dsp_rb.cpp ../src/orbslam_dsp_matcher.cpp
IN ?= frame.raw
OUT ?= frame.orbd
ITER ?= 1
# make DEFS=-DORBSLAM_DSP_DEBUG for stage-by-stage prints

# make orbslam_sim_skel: the same through the FastRPC skel entry point (src/orbslam3_skel_hrpc.c)
orbslam_sim_skel: $(SRCS) ../src/orbslam3_skel_hrpc.c $(wildcard ../src/*.h) $(wildcard include/*.h)
	$(CC) $(ARCH) -O2 -g -Iinclude -I../src -c ../src/orbslam3_skel_hrpc.c -o orbslam3_skel_hrpc.o
	$(CXX) $(CXXFLAGS) -DSIM_VIA_SKEL -o $@ $(SRCS) orbslam3_skel_hrpc.o -lhexagon

orbslam_sim: $(SRCS) $(wildcard ../src/*.h) $(wildcard include/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS) -lhexagon

run: orbslam_sim
	$(SIM) --mv66g_1024 --quiet --simulated_returnval orbslam_sim -- $(IN) $(OUT) $(ITER)

clean:
	rm -f orbslam_sim orbslam_sim_skel orbslam3_skel_hrpc.o

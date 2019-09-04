#
#
#

V = verilator

TOP = --top-module gba_top
RTL = ../src
SIMV = ../simv
V_INC = +incdir+$(RTL) +incdir+../src/cpuv/

V_DEFINE = +define+debug=1 +define+SIMULATION=1 --compiler msvc --public
V_DEFINE += --converge-limit 2000 -Wno-WIDTH -Wno-IMPLICIT -Wno-MODDUP -Wno-UNSIGNED -Wno-CASEINCOMPLETE -Wno-CASEX -Wno-SYMRSVDWORD -Wno-COMBDLY -Wno-INITIALDLY -Wno-BLKANDNBLK -Wno-UNOPTFLAT -Wno-SELRANGE -Wno-CMPCONST -Wno-CASEOVERLAP -Wno-PINMISSING -Wno-MULTIDRIVEN

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	LIBS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo `sdl2-config --libs`
	LIBS += -L/usr/local/lib -L/opt/local/lib

	CXXFLAGS += `sdl2-config --cflags`
	CXXFLAGS += -I/usr/local/include -I/opt/local/include
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += -lGL -ldl `sdl2-config --libs`

	CXXFLAGS += `sdl2-config --cflags`
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(findstring MINGW,$(UNAME_S)),MINGW)
	ECHO_MESSAGE = "MinGW"
	LIBS += -lgdi32 -lopengl32 -limm32 `pkg-config --static --libs sdl2`

	CXXFLAGS += `pkg-config --cflags sdl2`
	CFLAGS = $(CXXFLAGS)
endif

CFLAGS += $(CC_OPT) $(CC_DEFINE)
LDFLAGS = $(LIBS)
EXE = ./tmp/Vgba_top
#V_OPT = -O2 
#CC_OPT = -O
V_OPT = 
CC_OPT =  


V_SRC = \
    $(RTL)/audio/audio_top.sv \
    $(RTL)/audio/direct_sound/ds_mixer.sv \
    $(RTL)/audio/direct_sound/direct_sound.sv \
    $(RTL)/audio/4channel/wave_channel.sv \
    $(RTL)/audio/4channel/volume_envelope.sv \
    $(RTL)/audio/4channel/square_wave.sv \
    $(RTL)/audio/4channel/sound_channel.sv \
    $(RTL)/audio/4channel/power.sv \
    $(RTL)/audio/4channel/noise_channel.sv \
    $(RTL)/audio/4channel/length_counter.sv \
    $(RTL)/audio/4channel/frequency_timer.sv \
    $(RTL)/audio/4channel/frequency_sweep.sv \
    $(RTL)/audio/4channel/clock_divider.sv \
    $(RTL)/audio/4channel/4ch_mixer.sv \
    $(RTL)/graphics/special_effects/special_color_proc.sv \
    $(RTL)/graphics/special_effects/se_lib.sv \
    $(RTL)/graphics/special_effects/process_color.sv \
    $(RTL)/graphics/special_effects/match_target.sv \
    $(RTL)/graphics/priority_evaluation/window_masker.sv \
    $(RTL)/graphics/priority_evaluation/window_detector.sv \
    $(RTL)/graphics/priority_evaluation/valid.sv \
    $(RTL)/graphics/priority_evaluation/priority_evaluation_circuit.sv \
    $(RTL)/graphics/priority_evaluation/priority_comparator.sv \
    $(RTL)/graphics/priority_evaluation/pe_top.sv \
    $(RTL)/graphics/priority_evaluation/pe_lib.sv \
    $(RTL)/graphics/priority_evaluation/pe_fsm.sv \
    $(RTL)/graphics/obj/obj_top.sv \
    $(RTL)/graphics/obj/obj_row_double_buffer.sv \
    $(RTL)/graphics/obj/obj_rot_scale.sv \
    $(RTL)/graphics/obj/obj_lookup_unit.sv \
    $(RTL)/graphics/obj/obj_lib.sv \
    $(RTL)/graphics/obj/obj_flip_unit.sv \
    $(RTL)/graphics/obj/obj_address_unit.sv \
    $(RTL)/graphics/bg_processing_circuit/vram_controller.sv \
    $(RTL)/graphics/bg_processing_circuit/screen_lookup_unit.sv \
    $(RTL)/graphics/bg_processing_circuit/rot_scale_unit.sv \
    $(RTL)/graphics/bg_processing_circuit/reg_decoder.sv \
    $(RTL)/graphics/bg_processing_circuit/overflow_handler.sv \
    $(RTL)/graphics/bg_processing_circuit/mosaic_processing_unit.sv \
    $(RTL)/graphics/bg_processing_circuit/lib.sv \
    $(RTL)/graphics/bg_processing_circuit/data_formatter.sv \
    $(RTL)/graphics/bg_processing_circuit/char_data_lookup.sv \
    $(RTL)/graphics/bg_processing_circuit/bitmap_address_unit.sv \
    $(RTL)/graphics/bg_processing_circuit/bg_scrolling_unit.sv \
    $(RTL)/graphics/bg_processing_circuit/bg_processing_circuit.sv \
    $(RTL)/graphics/lib.sv \
    $(RTL)/graphics/graphics_top.sv \
    $(RTL)/graphics/graphics_system.sv \
    $(RTL)/graphics/vga_top.sv \
    $(RTL)/timer/timer_top.sv \
    $(RTL)/timer/timer.sv \
    $(RTL)/mem/mem_top.sv \
    $(RTL)/dma/lib.sv \
    $(RTL)/dma/dma.sv \
    $(RTL)/cpuv/multiplier.sv \
    $(RTL)/cpuv/mulctrlandregs.sv \
    $(RTL)/cpuv/mul32x8comb.sv \
    $(RTL)/cpuv/thumbdecoder.sv \
    $(RTL)/cpuv/shifter.sv \
    $(RTL)/cpuv/shiftamountreg.sv \
    $(RTL)/cpuv/resltbitmask.sv \
    $(RTL)/cpuv/regfile.sv \
    $(RTL)/cpuv/psr.sv \
    $(RTL)/cpuv/lsadrgen.sv \
    $(RTL)/cpuv/ipdr.sv \
    $(RTL)/cpuv/dataoutmux.sv \
    $(RTL)/cpuv/controllogic.sv \
    $(RTL)/cpuv/abusmultiplexer.sv \
    $(RTL)/cpuv/bbusmultiplexer.sv \
    $(RTL)/cpuv/arm7tdmis_top.sv \
    $(RTL)/cpuv/alu.sv \
    $(RTL)/cpuv/addressmux_incrementer.sv \
    $(RTL)/cpu/core_tb_defines.vh \
    $(RTL)/cpuv/armpackage.vh \
    $(RTL)/cpu/interrupt_controller.sv \
    $(RTL)/cpu/cpu_top.sv \
    $(RTL)/graphics/obj/attribute_lookup_unit.sv \
    $(SIMV)/vram_A.sv \
    $(SIMV)/vram_A_2.sv\
    $(SIMV)/vram_B.sv\
    $(SIMV)/vram_C.sv \
    $(SIMV)/palette_bg_ram.sv \
    $(SIMV)/OAM.sv\
    $(SIMV)/InternRam.sv\
    $(SIMV)/MainRAM.sv \
    $(SIMV)/buf0.sv\
    $(SIMV)/buf1.sv \
    $(RTL)/gba_top.sv

C_SRC = \
	sim_main.cpp  imgui/imgui_impl_sdl.cpp imgui/imgui_impl_opengl2.cpp imgui/imgui_draw.cpp imgui/imgui_widgets.cpp imgui/imgui.cpp
VOUT = tmp/Vtop.cpp

all: $(EXE)

$(VOUT): $(V_SRC)  Makefile
	$V -cc $(V_OPT) -LDFLAGS "$(LDFLAGS) " -exe --trace --Mdir ./tmp $(V_DEFINE) $(V_INC) $(TOP) -CFLAGS $(CFLAGS) $(V_SRC) $(C_SRC)

$(EXE): $(VOUT) $(C_SRC)
#	(cd tmp; make OPT="-fauto-inc-dec -fdce -fdefer-pop -fdse -ftree-ccp -ftree-ch -ftree-fre -ftree-dce -ftree-dse" -f Vtop.mk)
	(cd tmp; make -f Vgba_top.mk)

fast:
	(cd tmp; rm -f *.o ; make OPT="-fcompare-elim -fcprop-registers -fguess-branch-probability -fauto-inc-dec -fif-conversion2 -fif-conversion -fipa-pure-const -fdce -fipa-profile -fipa-reference -fmerge-constants -fsplit-wide-types -fdefer-pop -fdse -ftree-ccp -ftree-ch -ftree-fre -ftree-dce -ftree-dse -ftree-builtin-call-dce -ftree-copyrename -ftree-dominator-opts -ftree-forwprop -ftree-phiprop -ftree-sra -ftree-pta -ftree-ter -funit-at-a-time -ftree-bit-ccp -falign-functions  -falign-jumps -falign-loops  -falign-labels -fcaller-saves -fcrossjumping -fcse-follow-jumps -fcse-skip-blocks -fdelete-null-pointer-checks -fdevirtualize -fexpensive-optimizations -fgcse  -fgcse-lm -finline-small-functions -findirect-inlining -fipa-sra -foptimize-sibling-calls -fpartial-inlining -fpeephole2 -fregmove -freorder-blocks  -freorder-functions -frerun-cse-after-loop -fsched-interblock  -fsched-spec -fschedule-insns -fschedule-insns2 -fstrict-aliasing -fstrict-overflow -ftree-switch-conversion -ftree-pre -ftree-vrp" -f Vtop.mk)

clean:
	rm -f tmp/*

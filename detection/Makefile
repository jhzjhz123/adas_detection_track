CUR_DIR   = $(shell pwd)
SRC_DIR   = $(CUR_DIR)/src
OBJ_DIR   = $(CUR_DIR)/obj
LIB_DIR   = $(CUR_DIR)/lib
MODEL_DIR = $(CUR_DIR)/model
VPATH     = $(CUR_DIR) \
	$(CUR_DIR)/3rdparty/json11 \
	$(SRC_DIR)/utils \
	$(SRC_DIR)/envlight \
	$(SRC_DIR)/lane \
	$(SRC_DIR)/yolo

# CXX := g++
CFLAGS   :=   -std=c++11 -O3 -Wall -Wpointer-arith -ffast-math 

ARCH      =   $(shell uname -m | sed -e s/arm.*/armv71/ -e s/aarch64.*/aarch64/ )
ifeq ($(ARCH),armv71)
    CFLAGS +=  -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon
endif
ifeq ($(ARCH),aarch64)
    CFLAGS += -mcpu=cortex-a53 #-mfloat-abi=hard -mfpu=neon
endif

# INCLUDE   += -I$(CUR_DIR)/3rdparty \
	`pkg-config --cflags opencv`
INCLUDE   += -I$(CUR_DIR)/3rdparty \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/include \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/src/debug/hv-nurlnet-dpu2.0/1.0-r0/include \
	-I/opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/7.2.0/include

# LIBS   += -lpthread -lhineon -ln2cube -ldputils \
	`pkg-config --libs opencv`
LIBS   += -lpthread \
	-L/home/rongy/Xilinx_AI -lhineon \
	-L/home/rongy/Xilinx_AI -ln2cube \
	-L/home/rongy/Xilinx_AI -ldputils \
	-L/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/lib -ldpsegmentation -lglog -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_fuzzy  -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core
	
MODEL   +=   #$(MODEL_DIR)/yolov3/hasco-6/1219/1/dpu_hasco_6.elf \
	$(MODEL_DIR)/envlight/dpu_mobilenetv2_0.elf \
	$(MODEL_DIR)/lane/he/0115/dpu_mhe_1152.elf \
	$(MODEL_DIR)/lane/new/dpu_mobilenetv2_seg.elf

.PHONY: all clean

TARGET   = libdetection.so

SRCS    += $(wildcard ./3rdparty/json11/*.cpp $(SRC_DIR)/envlight/*.cpp $(SRC_DIR)/lane/*.cpp $(SRC_DIR)/utils/*.cpp $(SRC_DIR)/yolo/*.cpp)
DIRS     = $(notdir $(SRCS))
OBJS     = $(patsubst %.cpp, %.o, $(DIRS))

# OBJS     = json11.o envlight.o lane.o utils.o yolo.o

all: $(OBJ_DIR) $(LIB_DIR) $(TARGET)

$(OBJ_DIR) :
	-mkdir -p $@

$(LIB_DIR) :
	-mkdir -p $@
	
%.o : %.cpp
	@echo "\nCompiling $@..."
	$(CXX) -fPIC $(CFLAGS) -c $^ -o $(OBJ_DIR)/$@ $(INCLUDE) $(LIBS)
 
$(TARGET) : $(OBJS)
	@echo "\nCompiling $@..."
	$(CXX) -shared -fPIC $(CFLAGS) $(addprefix $(OBJ_DIR)/, $^) -o $(LIB_DIR)/$@ $(MODEL) $(INCLUDE) $(LIBS)

clean:
	@echo "\nCleaning..."
	$(RM) -rf $(LIB_DIR)
	$(RM) -rf $(OBJ_DIR)

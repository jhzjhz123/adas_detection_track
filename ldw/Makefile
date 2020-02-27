CUR_DIR   =   $(shell pwd)
SRC_DIR   =   $(CUR_DIR)/src
OBJ_DIR   =   $(CUR_DIR)/obj
LIB_DIR   =   $(CUR_DIR)/lib
MODEL_DIR =   $(CUR_DIR)/model
VPATH     =   $(SRC_DIR)

# CXX := g++
CFLAGS   :=   -O3 -Wall -Wpointer-arith -std=c++11 -ffast-math

ARCH      =   $(shell uname -m | sed -e s/arm.*/armv71/ -e s/aarch64.*/aarch64/ )
ifeq ($(ARCH),armv71)
    CFLAGS +=  -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon
endif
ifeq ($(ARCH),aarch64)
    CFLAGS += -mcpu=cortex-a53 #-mfloat-abi=hard -mfpu=neon
endif

# INCLUDE   += -I$(CUR_DIR)/hvapi \
	-I$(SRC_DIR) \
	`pkg-config --cflags opencv`
INCLUDE   += -I$(CUR_DIR)/hvapi \
	-I$(SRC_DIR) \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/include \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/src/debug/hv-nurlnet-dpu2.0/1.0-r0/include

# LIBS   += `pkg-config --libs opencv` \
	-lpthread -lhineon -ln2cube -ldputils

LIBS   += -lpthread \
	-L/home/rongy/Xilinx_AI -lhineon \
	-L/home/rongy/Xilinx_AI -ln2cube \
	-L/home/rongy/Xilinx_AI -ldputils \
	-L/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/lib -ldpsegmentation -lglog -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_fuzzy  -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core
	
MODEL   +=   #$(MODEL_DIR)/0115/dpu_mhe_1152.elf

.PHONY: all clean

TARGET   =     libldw_ransac.so

# SRCS    += $(wildcard $(CUR_DIR/)*.cpp $(SRC_DIR)/*.cpp)
# DIRS     = $(notdir $(SRCS))
# OBJS     = $(patsubst %.cpp, %.o, $(DIRS))

OBJS       :=   common.o distance_measure.o hv_lane.o kalman.o lane_class.o lane_detect.o lane_tracking.o

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
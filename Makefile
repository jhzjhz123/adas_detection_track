CUR_DIR   = $(shell pwd)
MODEL_DIR = $(CUR_DIR)/detection/model
OBJ_DIR   = $(CUR_DIR)/obj
BIN_DIR   = $(CUR_DIR)/bin
LIB_DIR   = $(CUR_DIR)/lib
VPATH     = $(CUR_DIR) \
	$(CUR_DIR)/detection/3rdparty/json11 \
	$(CUR_DIR)/detection/src/utils \
	$(CUR_DIR)/detection/src/envlight \
	$(CUR_DIR)/detection/src/lane \
	$(CUR_DIR)/detection/src/yolo\
	$(CUR_DIR)/tracktion/src/Calibration \
	$(CUR_DIR)/tracktion/src/ObjectEstimation \
	$(CUR_DIR)/tracktion/src/Tools \
	$(CUR_DIR)/tracktion/src/trackAndMatch \
	$(CUR_DIR)/ldw/src

# CXX := g++
# CC  := gcc
# CXX := /opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-g++
# CC  := /opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-gcc

CFLAGS   :=   -std=c++11 -O3 -Wall -Wpointer-arith -ffast-math

ARCH      =   $(shell uname -m | sed -e s/arm.*/armv71/ -e s/aarch64.*/aarch64/ )
ifeq ($(ARCH),armv71)
    CFLAGS +=  -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon
endif
ifeq ($(ARCH),aarch64)
    CFLAGS += -mcpu=cortex-a53 -mfloat-abi=hard -mfpu=neon
endif

# INCLUDE   += -I$(CUR_DIR)/detection/3rdparty \
	-I$(CUR_DIR)/detection/src \
	-I$(CUR_DIR)/tracktion/3rdparty \
	-I$(CUR_DIR)/tracktion/3rdparty/src \
	-I$(CUR_DIR)/tracktion/src \
	-I$(CUR_DIR)/tracktion/src/Calibration \
	-I$(CUR_DIR)/tracktion/src/ObjectEstimation \
	-I$(CUR_DIR)/tracktion/src/Tools \
	-I$(CUR_DIR)/tracktion/src/trackAndMatch \
	-I$(CUR_DIR)/ldw/hvapi \
	-I$(CUR_DIR)/ldw/src \
	`pkg-config --cflags opencv`
INCLUDE   += -I$(CUR_DIR)/detection/3rdparty \
	-I$(CUR_DIR)/detection/src \
	-I$(CUR_DIR)/tracktion/3rdparty \
	-I$(CUR_DIR)/tracktion/3rdparty/src \
	-I$(CUR_DIR)/tracktion/src \
	-I$(CUR_DIR)/tracktion/src/Calibration \
	-I$(CUR_DIR)/tracktion/src/ObjectEstimation \
	-I$(CUR_DIR)/tracktion/src/Tools \
	-I$(CUR_DIR)/tracktion/src/trackAndMatch \
	-I$(CUR_DIR)/ldw/hvapi \
	-I$(CUR_DIR)/ldw/src \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/include \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/src/debug/hv-nurlnet-dpu2.0/1.0-r0/include \
	-I/opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/7.2.0/include

# LIBS   += -lpthread -lhineon -ln2cube -ldputils \
	`pkg-config --libs opencv`
LIBS   += -lpthread \
	-L/home/rongy/Xilinx_AI -lhineon \
	-L/home/rongy/Xilinx_AI -ln2cube \
	-L/home/rongy/Xilinx_AI -ldputils \
	-L/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/lib -lglog -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_fuzzy  -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core

# MODEL   +=   $(MODEL_DIR)/yolov3/demo/dpu_hasco.elf
MODEL   +=   $(MODEL_DIR)/yolov3/hasco-6/0408/dpu_hasco_6.elf \
	$(MODEL_DIR)/envlight/dpu_mobilenetv2_0.elf \
	$(MODEL_DIR)/lane/he/0115/dpu_mhe_1152.elf \
	$(MODEL_DIR)/lane/new/1210/dpu_mobilenetv2_seg.elf

.PHONY: all clean

TARGET   = adas_detection_tracktion

SRCS    += $(wildcard *.cpp ./detection/3rdparty/json11/*.cpp ./detection/src/envlight/*.cpp ./detection/src/lane/*.cpp ./detection/src/utils/*.cpp ./detection/src/yolo/*.cpp) \
	$(wildcard ./tracktion/src/Calibration/*.cpp ./tracktion/src/ObjectEstimation/*.cpp ./tracktion/src/Tools/*.cpp ./tracktion/src/trackAndMatch/*.cpp) \
	$(wildcard ./ldw/src/*.cpp)
DIRS     = $(notdir $(SRCS))
OBJS     = $(patsubst %.cpp, %.o, $(DIRS))

# OBJS     = main.o json11.o envlight.o lane.o utils.o yolo.o \
	Calibrater.o tinyxml2.o DistanceMeasure.o Object.o ObjectEstimation.o trajectory.o TTC.o \
	Tools.o hungarian.o KalmanFilterToolBox.o linear_assignment.o nn_matching.o track.o tracker.o \
	common.o distance_measure.o hv_lane.o kalman.o lane_class.o lane_detect.o lane_tracking.o

all: $(OBJ_DIR) $(BIN_DIR) $(LIB_DIR) $(TARGET) libdetection.so libtracktion.so libldw.so

$(OBJ_DIR) :
	-mkdir -p $@

$(BIN_DIR) :
	-mkdir -p $@

$(LIB_DIR) :
	-mkdir -p $@
	
%.o : %.cpp
	@echo "\nCompiling $@..."
	$(CXX) -fPIC $(CFLAGS) -c $^ -o $(OBJ_DIR)/$@ $(INCLUDE) $(LIBS)
 
$(TARGET) : $(OBJS)
	@echo "\nCompiling $@..."
	$(CXX) $(CFLAGS) $(addprefix $(OBJ_DIR)/, $^) -o $(BIN_DIR)/$@ $(MODEL) $(INCLUDE) $(LIBS)

libdetection.so : $(filter-out Calibrater.o tinyxml2.o hungarian.o KalmanFilterToolBox.o linear_assignment.o nn_matching.o track.o tracker.o \
	DistanceMeasure.o Object.o ObjectEstimationControl.o TTC.o trajectory.o tools.o main.o, $(OBJS))
	@echo "\nCompiling $@..."
	$(CXX) -shared -fPIC $(CFLAGS) $(addprefix $(OBJ_DIR)/, $^) -o $(LIB_DIR)/$@ $(MODEL) $(INCLUDE) $(LIBS)

libtracktion.so : $(filter-out json11.o utils.o yolo.o envlight.o lane.o main.o, $(OBJS))
	@echo "\nCompiling $@..."
	$(CXX) -shared -fPIC $(CFLAGS) $(addprefix $(OBJ_DIR)/, $^) -o $(LIB_DIR)/$@ $(INCLUDE) $(LIBS)

libldw.so : $(filter-out Calibrater.o tinyxml2.o hungarian.o KalmanFilterToolBox.o linear_assignment.o nn_matching.o track.o tracker.o \
	DistanceMeasure.o Object.o ObjectEstimationControl.o TTC.o trajectory.o tools.o \
	json11.o utils.o yolo.o envlight.o lane.o main.o, $(OBJS))
	@echo "\nCompiling $@..."
	$(CXX) -shared -fPIC $(CFLAGS) $(addprefix $(OBJ_DIR)/, $^) -o $(LIB_DIR)/$@ $(INCLUDE) $(LIBS)

clean:
	@echo "\nCleaning..."
	$(RM) -rf $(OBJ_DIR)
	$(RM) -rf $(BIN_DIR)
	$(RM) -rf $(LIB_DIR)


# ##########################################################################################################################################################

# CUR_DIR   = $(shell pwd)
# MODEL_DIR = $(CUR_DIR)/detection/model
# BUILD_DIR   = $(CUR_DIR)/build

# VPATH     = $(CUR_DIR)

# # CXX := g++
# # CC  := gcc
# # CXX := /opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-g++
# # CC  := /opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-gcc
# CFLAGS   :=   -std=c++11 -O3 -Wall -Wpointer-arith -ffast-math

# ARCH      =   $(shell uname -m | sed -e s/arm.*/armv71/ -e s/aarch64.*/aarch64/ )
# ifeq ($(ARCH),armv71)
#     CFLAGS +=  -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon
# endif
# ifeq ($(ARCH),aarch64)
#     CFLAGS += -mcpu=cortex-a53 -mfloat-abi=hard -mfpu=neon
# endif

# # INCLUDE   += -I$(CUR_DIR)/detection/3rdparty \
# 	-I$(CUR_DIR)/detection/src \
# 	-I$(CUR_DIR)/tracktion/3rdparty \
# 	-I$(CUR_DIR)/tracktion/3rdparty/src \
# 	-I$(CUR_DIR)/tracktion/src \
# 	-I$(CUR_DIR)/tracktion/src/Calibration \
# 	-I$(CUR_DIR)/tracktion/src/ObjectEstimation \
# 	-I$(CUR_DIR)/tracktion/src/Tools \
# 	-I$(CUR_DIR)/tracktion/src/trackAndMatch \
# 	-I$(CUR_DIR)/ldw/hvapi \
# 	-I$(CUR_DIR)/ldw/src \
# 	`pkg-config --cflags opencv`
# INCLUDE   += -I$(CUR_DIR)/detection/3rdparty \
# 	-I$(CUR_DIR)/detection/src \
# 	-I$(CUR_DIR)/tracktion/3rdparty \
# 	-I$(CUR_DIR)/tracktion/3rdparty/src \
# 	-I$(CUR_DIR)/tracktion/src \
# 	-I$(CUR_DIR)/tracktion/src/Calibration \
# 	-I$(CUR_DIR)/tracktion/src/ObjectEstimation \
# 	-I$(CUR_DIR)/tracktion/src/Tools \
# 	-I$(CUR_DIR)/tracktion/src/trackAndMatch \
# 	-I$(CUR_DIR)/ldw/hvapi \
# 	-I$(CUR_DIR)/ldw/src \
# 	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/include \
# 	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/src/debug/hv-nurlnet-dpu2.0/1.0-r0/include \
# 	-I/opt/petalinux/2018.2/sysroots/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/7.2.0/include

# # LIBS   += -lpthread -lhineon -ln2cube -ldputils \
# 	`pkg-config --libs opencv` \
# 	-L$(CUR_DIR)/lib -ldetection \
# 	-L$(CUR_DIR)/lib -ltracktion \
# 	-L$(CUR_DIR)/lib -lldw
# LIBS   += -lpthread \
# 	-L/home/rongy/Xilinx_AI -lhineon \
# 	-L/home/rongy/Xilinx_AI -ln2cube \
# 	-L/home/rongy/Xilinx_AI -ldputils \
# 	-L/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/lib -lglog -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_fuzzy  -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core \
# 	-L$(CUR_DIR)/lib -ldetection \
# 	-L$(CUR_DIR)/lib -ltracktion \
# 	-L$(CUR_DIR)/lib -lldw
	
# # MODEL   +=   $(MODEL_DIR)/yolov3/demo/dpu_hasco.elf
# MODEL   +=   $(MODEL_DIR)/yolov3/hasco-6/0408/dpu_hasco_6.elf \
# 	$(MODEL_DIR)/envlight/dpu_mobilenetv2_0.elf \
# 	$(MODEL_DIR)/lane/he/1225/dpu_mhe.elf \
# 	$(MODEL_DIR)/lane/new/dpu_mobilenetv2_seg.elf

# .PHONY: all clean

# TARGET   = adas_detection_tracktion

# OBJS     = main.o

# all: $(BUILD_DIR) $(TARGET)

# $(BUILD_DIR) :
# 	-mkdir -p $@

# %.o : %.cpp
# 	$(CXX) $(CFLAGS) -c $^ -o $(BUILD_DIR)/$@ $(INCLUDE) $(LIBS)
 
# $(TARGET) : $(OBJS)
# 	$(CXX) $(CFLAGS) $(addprefix $(BUILD_DIR)/, $^) -o $(BUILD_DIR)/$@ $(MODEL) $(INCLUDE) $(LIBS)

# clean:
# 	rm -rf $(BUILD_DIR)
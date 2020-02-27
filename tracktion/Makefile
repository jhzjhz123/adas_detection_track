CUR_DIR   = $(shell pwd)
SRC_DIR   = $(CUR_DIR)/src
OBJ_DIR   = $(CUR_DIR)/obj
LIB_DIR   = $(CUR_DIR)/lib
VPATH     = $(SRC_DIR) \
	$(SRC_DIR)/Calibration \
	$(SRC_DIR)/ObjectEstimation \
	$(SRC_DIR)/Tools \
	$(SRC_DIR)/trackAndMatch

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
	-I$(CUR_DIR)/3rdparty/src \
	-I$(SRC_DIR) \
	-I$(SRC_DIR)/Calibration \
	-I$(SRC_DIR)/ObjectEstimation \
	-I$(SRC_DIR)/Tools \
	-I$(SRC_DIR)/trackAndMatch \
	`pkg-config --cflags opencv`
INCLUDE   += -I$(CUR_DIR)/3rdparty \
	-I$(CUR_DIR)/3rdparty/src \
	-I$(SRC_DIR) \
	-I$(SRC_DIR)/Calibration \
	-I$(SRC_DIR)/ObjectEstimation \
	-I$(SRC_DIR)/Tools \
	-I$(SRC_DIR)/trackAndMatch \
	-I/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/include \

# LIBS   += -lpthread \
	`pkg-config --libs opencv`
LIBS   += -lpthread \
	-L/opt/petalinux/2018.2/sysroots/aarch64-xilinx-linux/usr/lib -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_fuzzy  -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core

.PHONY: all clean

TARGET   =  libtrack.so

SRCS    += $(wildcard $(SRC_DIR)/Calibration/*.cpp $(SRC_DIR)/ObjectEstimation/*.cpp $(SRC_DIR)/Tools/*.cpp $(SRC_DIR)/trackAndMatch/*.cpp)
DIRS     = $(notdir $(SRCS))
OBJS     = $(patsubst %.cpp, %.o, $(DIRS))

# OBJS     = Calibrater.o tinyxml2.o DistanceMeasure.o Object.o ObjectEstimation.o trajectory.o TTC.o \
	Tools.o hungarian.o KalmanFilterToolBox.o linear_assignment.o nn_matching.o track.o tracker.o

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
	$(CXX) -shared -fPIC $(CFLAGS) $(addprefix $(OBJ_DIR)/, $^) -o $(LIB_DIR)/$@ $(INCLUDE) $(LIBS)

clean:
	@echo "\nCleaning..."
	$(RM) -rf $(OBJ_DIR)
	$(RM) -rf $(LIB_DIR)
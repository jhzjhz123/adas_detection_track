#working directory
work_dir=$(pwd)
#path of float model
DIR=${work_dir}

#cd dnnc_output/
#cp dpu_mobilenetv2_seg.elf ./mobilenetv2_seg

MODEL_NAME=mobilenetv2_seg
#fpn_deconv
aarch64-linux-gnu-g++ \
     -nostdlib \
     -fPIC \
     -shared \
     ${DIR}/dpu_${MODEL_NAME}*.elf \
     -o \
     ${DIR}/libdpumodel${MODEL_NAME}.so || touch ${DIR}/libdpumodel${MODEL_NAME}.so

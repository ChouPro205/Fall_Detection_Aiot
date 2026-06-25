set(EI_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/edge-impulse-sdk")

set(EI_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/tflite-model/tflite_learn_944636_3_compiled.cpp"
    "${EI_SDK_DIR}/dsp/memory.cpp"
    "${EI_SDK_DIR}/dsp/kissfft/kiss_fft.cpp"
    "${EI_SDK_DIR}/dsp/kissfft/kiss_fftr.cpp"
    "${EI_SDK_DIR}/porting/espressif/ei_classifier_porting.cpp"
    "${EI_SDK_DIR}/tensorflow/lite/kernels/internal/portable_tensor_utils.cc"
    "${EI_SDK_DIR}/tensorflow/lite/kernels/internal/quantization_util.cc"
    "${EI_SDK_DIR}/tensorflow/lite/kernels/kernel_util_lite.cc"
    "${EI_SDK_DIR}/tensorflow/lite/core/api/common.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/memory_helpers.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/micro_allocator.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/micro_context.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/kernels/kernel_util_micro.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/kernels/fully_connected_common.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/kernels/fully_connected.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/kernels/softmax_common.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/kernels/softmax.cc"
    "${EI_SDK_DIR}/tensorflow/lite/micro/micro_log.cc"
)

idf_component_register(
    SRCS ${EI_SOURCES}
    INCLUDE_DIRS "."
    REQUIRES esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=0
    EIDSP_USE_ESP_DSP=0
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=unused-function
    -Wno-error=unused-variable
    -Wno-error=maybe-uninitialized
)

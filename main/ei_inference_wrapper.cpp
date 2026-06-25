#include "ei_inference_wrapper.h"

#include <cstring>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

extern "C" size_t ei_fall_get_feature_count(void)
{
    return EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
}

extern "C" bool ei_fall_run_classifier_from_features(const float *features,
                                                       size_t feature_count,
                                                       ei_fall_result_t *out_result)
{
    if (features == nullptr ||
        out_result == nullptr ||
        feature_count != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        return false;
    }

    *out_result = {};
    out_result->predicted_index = -1;
    out_result->predicted_label = "unknown";

    signal_t signal;
    if (ei::numpy::signal_from_buffer(features, feature_count, &signal) != EIDSP_OK) {
        return false;
    }

    ei_impulse_result_t result = {};
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
        return false;
    }

    float best_score = -1.0f;
    for (size_t index = 0; index < EI_CLASSIFIER_LABEL_COUNT; ++index) {
        const char *label = result.classification[index].label;
        const float score = result.classification[index].value;

        if (label != nullptr) {
            if (std::strcmp(label, "fall_soft") == 0) {
                out_result->fall_soft = score;
            } else if (std::strcmp(label, "lying") == 0) {
                out_result->lying = score;
            } else if (std::strcmp(label, "normal") == 0) {
                out_result->normal = score;
            } else if (std::strcmp(label, "shake") == 0) {
                out_result->shake = score;
            }
        }

        if (score > best_score) {
            best_score = score;
            out_result->predicted_index = static_cast<int>(index);
            out_result->predicted_label = label != nullptr ? label : "unknown";
        }
    }

    return true;
}

#ifndef EI_INFERENCE_WRAPPER_H
#define EI_INFERENCE_WRAPPER_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float fall_soft;
    float lying;
    float normal;
    float shake;
    int predicted_index;
    const char *predicted_label;
} ei_fall_result_t;

bool ei_fall_run_classifier_from_features(const float *features,
                                          size_t feature_count,
                                          ei_fall_result_t *out_result);

size_t ei_fall_get_feature_count(void);

#ifdef __cplusplus
}
#endif

#endif // EI_INFERENCE_WRAPPER_H

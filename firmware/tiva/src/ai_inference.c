#include "ai_navigation.h"
#include "ai_model_weights.h"

#include <math.h>
#include <stddef.h>

static float relu(float value)
{
    return value > 0.0f ? value : 0.0f;
}

bool ai_model_predict(const float features[AI_MODEL_INPUTS],
                      float probabilities[AI_MODEL_ACTIONS])
{
    float hidden1[AI_MODEL_HIDDEN1];
    float hidden2[AI_MODEL_HIDDEN2];
    float logits[AI_MODEL_ACTIONS];
    float max_logit;
    float total = 0.0f;
    uint32_t i, j;

    if (features == NULL || probabilities == NULL) return false;

    for (i = 0U; i < AI_MODEL_INPUTS; ++i) {
        if (!isfinite(features[i])) return false;
        if (features[i] < -1.001f || features[i] > 1.001f) return false;
    }

    for (i = 0U; i < AI_MODEL_HIDDEN1; ++i) {
        float sum = ai_b1[i];
        for (j = 0U; j < AI_MODEL_INPUTS; ++j)
            sum += ai_w1[i][j] * features[j];
        hidden1[i] = relu(sum);
    }

    for (i = 0U; i < AI_MODEL_HIDDEN2; ++i) {
        float sum = ai_b2[i];
        for (j = 0U; j < AI_MODEL_HIDDEN1; ++j)
            sum += ai_w2[i][j] * hidden1[j];
        hidden2[i] = relu(sum);
    }

    max_logit = -INFINITY;
    for (i = 0U; i < AI_MODEL_ACTIONS; ++i) {
        float sum = ai_b3[i];
        for (j = 0U; j < AI_MODEL_HIDDEN2; ++j)
            sum += ai_w3[i][j] * hidden2[j];
        if (!isfinite(sum)) return false;
        logits[i] = sum;
        if (sum > max_logit) max_logit = sum;
    }

    /* Numerically stable softmax; output scores sum to approximately one. */
    for (i = 0U; i < AI_MODEL_ACTIONS; ++i) {
        probabilities[i] = expf(logits[i] - max_logit);
        total += probabilities[i];
    }
    if (!isfinite(total) || total <= 0.0f) return false;
    for (i = 0U; i < AI_MODEL_ACTIONS; ++i)
        probabilities[i] /= total;

    return true;
}

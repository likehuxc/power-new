#ifndef SAF_FILTER_H__
#define SAF_FILTER_H__

#include <stdint.h>

typedef struct {
    float   *buff;
    uint16_t samples;
    uint16_t count;
} saf_t;

void sliding_average_filter_init(saf_t *obj, float *buff, uint16_t count);
float sliding_average_filter(saf_t *obj, float in);
int sliding_average_filter_data_valid(saf_t *obj);

#endif /* SAF_FILTER_H__ */

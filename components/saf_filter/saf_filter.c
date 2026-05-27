#include "saf_filter.h"

void sliding_average_filter_init(saf_t *obj, float *buff, uint16_t count)
{
    uint16_t i;

    obj->buff    = buff;
    obj->samples = 0U;
    obj->count   = count;

    for (i = 0U; i < count; i++) {
        obj->buff[i] = 0.0f;
    }
}

float sliding_average_filter(saf_t *obj, float in)
{
    uint16_t i;
    float    sum = 0.0f;

    for (i = 0U; i < (obj->count - 1U); i++) {
        obj->buff[i] = obj->buff[i + 1U];
    }
    obj->buff[obj->count - 1U] = in;

    if (obj->samples < obj->count) {
        obj->samples++;
    }

    if (obj->samples >= obj->count) {
        for (i = 0U; i < obj->count; i++) {
            sum += obj->buff[i];
        }
        sum /= (float)obj->count;
    }

    return sum;
}

int sliding_average_filter_data_valid(saf_t *obj)
{
    if (obj->samples >= obj->count) {
        return 1;
    }
    return 0;
}

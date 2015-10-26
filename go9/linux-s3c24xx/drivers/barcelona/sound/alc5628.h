#ifndef ALC5628_H
#define ALC5628_H

#include "codec.h"
#include <linux/i2c.h>

void alc5628_init(struct codec_ops *ops);
void alc5628_init_i2c_control(struct i2c_client *cl);

#endif /* ALC5628_H */

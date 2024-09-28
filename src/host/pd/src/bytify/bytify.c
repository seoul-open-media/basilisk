#include <stdbool.h>
#include <string.h>

#include "m_pd.h"

#define PACKET_LEN 50

static t_class* bytify_class;

typedef union {
  uint16_t u16;
  uint8_t u8[2];
} u16_to_u8_t;

typedef union {
  uint16_t u32;
  uint8_t u8[4];
} u32_to_u8_t;

typedef union {
  float f32;
  uint8_t u8[4];
} f32_to_u8_t;

typedef union {
  double d64;
  uint8_t u8[8];
} d64_to_u8_t;

enum Mode { U8, U16, U32, F32, D64 } mode;

typedef struct _bytify {
  t_object o;
  t_outlet* out;
} t_bytify;

void bytify_listin1(t_bytify* x, t_symbol*, int argc, t_atom* argv) {
  t_atom output[PACKET_LEN];
  int output_idx = 0;
  mode = F32;

  for (int i = 0; i < argc && i < PACKET_LEN; i++) {
    if (argv[i].a_type == A_SYMBOL) {
      if (strcmp(argv[i].a_w.w_symbol->s_name, "U8") == 0) {
        mode = U8;
      } else if (strcmp(argv[i].a_w.w_symbol->s_name, "U16") == 0) {
        mode = U16;
      } else if (strcmp(argv[i].a_w.w_symbol->s_name, "U32") == 0) {
        mode = U32;
      } else if (strcmp(argv[i].a_w.w_symbol->s_name, "F32") == 0) {
        mode = F32;
      } else if (strcmp(argv[i].a_w.w_symbol->s_name, "D64") == 0) {
        mode = D64;
      }
    } else if (argv[i].a_type == A_FLOAT) {
      if (mode == U8) {
        uint8_t elm = (uint8_t)argv[i].a_w.w_float;
        SETFLOAT(&output[output_idx], elm);
        output_idx++;
      } else if (mode == U16) {
        u16_to_u8_t trans;
        trans.u16 = (uint16_t)argv[i].a_w.w_float;
        for (int j = 0; j < 2; j++) {
          SETFLOAT(&output[output_idx], trans.u8[j]);
          output_idx++;
        }
      } else if (mode == U32) {
        u32_to_u8_t trans;
        trans.u32 = (uint32_t)argv[i].a_w.w_float;
        for (int j = 0; j < 4; j++) {
          SETFLOAT(&output[output_idx], trans.u8[j]);
          output_idx++;
        }
      } else if (mode == F32) {
        f32_to_u8_t trans;
        trans.f32 = (float)argv[i].a_w.w_float;
        for (int j = 0; j < 4; j++) {
          SETFLOAT(&output[output_idx], trans.u8[j]);
          output_idx++;
        }
      } else if (mode == D64) {
        d64_to_u8_t trans;
        trans.d64 = (double)argv[i].a_w.w_float;
        for (int j = 0; j < 8; j++) {
          SETFLOAT(&output[output_idx], trans.u8[j]);
          output_idx++;
        }
      }
    }
  }

  for (; output_idx < PACKET_LEN; output_idx++) {
    SETFLOAT(&output[output_idx], 0);
  }

  outlet_list(x->out, &s_list, PACKET_LEN, output);
}

void* bytify_new() {
  t_bytify* x = (t_bytify*)pd_new(bytify_class);
  x->out = outlet_new(&x->o, &s_list);
  return (void*)x;
}

void bytify_setup() {
  bytify_class = class_new(gensym("bytify"), (t_newmethod)bytify_new, 0,
                           sizeof(t_bytify), CLASS_DEFAULT, 0);

  class_addlist(bytify_class, bytify_listin1);
}

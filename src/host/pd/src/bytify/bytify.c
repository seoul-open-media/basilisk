#include <stdbool.h>
#include <string.h>

#include "m_pd.h"

static t_class* bytify_class;

typedef union {
  float f;
  uint8_t bytes[4];
} f2b_t;

enum Mode { Byte, Float } mode;

typedef struct _bytify {
  t_object o;
  t_outlet* out;
} t_bytify;

void bytify_listin1(t_bytify* x, t_symbol*, int argc, t_atom* argv) {
  t_atom output[256];  // Output list (adjust size as needed)
  int output_idx = 0;
  mode = Float;

  for (int i = 0; i < argc; i++) {
    if (argv[i].a_type == A_SYMBOL) {
      if (strcmp(argv[i].a_w.w_symbol->s_name, "b") == 0) {
        mode = Byte;
      } else if (strcmp(argv[i].a_w.w_symbol->s_name, "f") == 0) {
        mode = Float;
      }
    } else if (argv[i].a_type == A_FLOAT) {
      if (mode == Byte) {
        uint8_t byte = (uint8_t)argv[i].a_w.w_float;
        SETFLOAT(&output[output_idx], byte);
        output_idx++;
      } else if (mode == Float) {
        f2b_t f2b;
        f2b.f = (float)argv[i].a_w.w_float;
        for (int j = 0; j < 4; j++) {
          SETFLOAT(&output[output_idx], f2b.bytes[j]);
          output_idx++;
        }
      }
    }
  }

  outlet_list(x->out, &s_list, output_idx, output);
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

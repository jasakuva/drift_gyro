#include <Arduino.h>

class ControlParams {
public:
  ControlParams() { initDefaults(); buildTable(); }

  bool set(const String& name, float value) {
    for (int i = 0; i < count; i++) {
      if (name.equals(table[i].name)) {
        if (table[i].type == TYPE_FLOAT) *(float*)table[i].ptr = value;
        else *(int*)table[i].ptr = (int)value;
        return true;
      }
    }
    return false;
  }

  bool get(const String& name, float &out) {
    for (int i = 0; i < count; i++) {
      if (name.equals(table[i].name)) {
        out = (table[i].type == TYPE_FLOAT) ? *(float*)table[i].ptr : (float)*(int*)table[i].ptr;
        return true;
      }
    }
    return false;
  }

  // Members still exist (optional to access directly)
  // auto-generated below

private:
  enum ParamType { TYPE_INT, TYPE_FLOAT };
  struct Entry { const char* name; void* ptr; ParamType type; };

  static const int MAX = 32;
  Entry table[MAX];
  int count = 0;

  void add(const char* n, void* p, ParamType t) { table[count++] = {n,p,t}; }

  void initDefaults() {
    #define X_FLOAT(name, def) name = (def);
    #define X_INT(name, def)   name = (def);
    #include "params_list.h"
    #undef X_FLOAT
    #undef X_INT
  }

  void buildTable() {
    count = 0;
    #define X_FLOAT(name, def) add(#name, &name, TYPE_FLOAT);
    #define X_INT(name, def)   add(#name, &name, TYPE_INT);
    #include "params_list.h"
    #undef X_FLOAT
    #undef X_INT
  }

public:
  // auto-generate member declarations
  #define X_FLOAT(name, def) float name;
  #define X_INT(name, def)   int name;
  #include "params_list.h"
  #undef X_FLOAT
  #undef X_INT
};
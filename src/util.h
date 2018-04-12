#ifndef UTIL_H_AEA1CW2J
#define UTIL_H_AEA1CW2J

#include <vector>
#include "typedefs.h"

namespace pypcl {

// TODO sadly can't const pc because iterator doesn't handle it
void get_valid_xyz_indices(const PCLPC2& pc, std::vector<int>& valid_ix);

}

#endif /* end of include guard: UTIL_H_AEA1CW2J */

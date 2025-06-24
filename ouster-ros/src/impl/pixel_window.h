#pragma once

#include <cstdint>

#include <vector>

namespace ouster {

std::pair<uint64_t, uint64_t> horizon_to_pixel_window(const std::vector<double>& beam_altitude_angles,
                                                      int64_t horizon_window_start,
                                                      int64_t horizon_window_end);

}  // namespace ouster

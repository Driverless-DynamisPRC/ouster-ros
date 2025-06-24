#pragma once

#include <cstdint>

#include <algorithm>
#include <vector>

namespace ouster {

std::pair<uint64_t, uint64_t> horizon_to_pixel_window(const std::vector<double>& beam_altitude_angles,
                                                      int64_t horizon_window_start,
                                                      int64_t horizon_window_end) {
    auto pixel_start_it = std::find_if(
        beam_altitude_angles.rbegin(), beam_altitude_angles.rend(),
        [&](const auto &angle) { return angle >= horizon_window_start; });
    auto pixel_end_it = std::find_if(
        pixel_start_it + 1, beam_altitude_angles.rend(),
        [&](const auto &angle) { return angle >= horizon_window_end; });

    auto pixel_start = static_cast<uint64_t>(std::distance(
        beam_altitude_angles.rbegin(), pixel_start_it));
    auto pixel_end = static_cast<uint64_t>(std::distance(
        beam_altitude_angles.rbegin(), pixel_end_it));

    return {pixel_start, pixel_end};
}

}  // namespace ouster

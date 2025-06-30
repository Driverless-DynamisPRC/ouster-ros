#include "pixel_window.h"

#include <algorithm>
#include <stdexcept>

namespace ouster {

    std::pair<uint64_t, uint64_t> horizon_to_pixel_window(const std::vector<double> &beam_altitude_angles,
                                                          int64_t horizon_window_start,
                                                          int64_t horizon_window_end) {
        if (beam_altitude_angles.empty())
            throw std::invalid_argument("horizon_to_pixel_window: beam_altitude_angles is empty");

        std::pair<uint64_t, uint64_t> pixel_window;

        const auto hw_start = static_cast<double>(horizon_window_start) * 1e-3;
        const auto hw_end = static_cast<double>(horizon_window_end) * 1e-3;

        if (beam_altitude_angles.front() < beam_altitude_angles.back()) {
            const auto pixel_start_it = std::lower_bound(
                    beam_altitude_angles.begin(), beam_altitude_angles.end(), hw_start);
            const auto pixel_end_it = std::upper_bound(
                    pixel_start_it, beam_altitude_angles.end(), hw_end);

            pixel_window.first = static_cast<uint64_t>(std::distance(
                    beam_altitude_angles.begin(), pixel_start_it));
            pixel_window.second = static_cast<uint64_t>(std::distance(
                    beam_altitude_angles.begin(), pixel_end_it));
        } else {
            const auto pixel_start_it = std::lower_bound(
                beam_altitude_angles.rbegin(), beam_altitude_angles.rend(), hw_start);
            const auto pixel_end_it = std::upper_bound(
                    pixel_start_it + 1, beam_altitude_angles.rend(), hw_end);

            pixel_window.first = static_cast<uint64_t>(std::distance(
                    beam_altitude_angles.rbegin(), pixel_start_it));
            pixel_window.second = static_cast<uint64_t>(std::distance(
                    beam_altitude_angles.rbegin(), pixel_end_it));
        }

        return pixel_window;
    }

}// namespace ouster

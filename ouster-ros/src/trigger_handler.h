// #pragma once


#include "lidar_packet_handler.h"


namespace ouster_ros {

class TriggerHandler {
  public:
    using PostProcessingFn = std::function<void()>;
    using HandlerType = std::function<void(const sensor::LidarPacket&)>;

    TriggerHandler(const sensor::sensor_info& info,
                   const uint32_t fire_column, PostProcessingFn func)
      : post_processing_fn(func), pf(sensor::get_format(info)), fire_column(fire_column) {
    }

    static HandlerType create(const sensor::sensor_info& info,
                                     uint32_t fire_column, PostProcessingFn func) {
      auto handler = std::make_shared<TriggerHandler>(info, fire_column, func);
      return [handler](const sensor::LidarPacket& lidar_packet) {
        const auto col0 = handler->pf.nth_col(0, lidar_packet.buf.data());
        const auto col0_id = handler->pf.col_measurement_id(col0);

        const auto coln = handler->pf.nth_col(handler->pf.columns_per_packet - 1, lidar_packet.buf.data());
        const auto coln_id = handler->pf.col_measurement_id(coln);

        if (col0_id <= handler->fire_column && handler->fire_column <= coln_id) {
          if (handler->post_processing_fn) handler->post_processing_fn();
        }
      };
    }

  private:
    PostProcessingFn post_processing_fn;
    const sensor::packet_format pf;
    uint32_t fire_column;
};

}

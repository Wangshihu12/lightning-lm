//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace lightning {

void RosbagIO::Go(int sleep_usec) {
    rosbag::Bag bag;
    bag.open(bag_file_, rosbag::bagmode::Read);

    rosbag::View view(bag);
    for (const rosbag::MessageInstance &msg : view) {
        auto iter = process_func_.find(msg.getTopic());
        if (iter != process_func_.end()) {
            iter->second(msg);
        }

        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }

        if (lightning::debug::flg_exit) {
            break;
        }
    }

    bag.close();
    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

}  // namespace lightning

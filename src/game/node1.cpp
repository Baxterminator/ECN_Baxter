#include <ecn_baxter/game/node1.hpp>

namespace ECNBaxter {

    void GMROS1::spin() {
        broadcast_ptn();
    }

    void GMROS1::addPoint(const std::string &name, const Point &ptn) {
        TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "base";
        ts.child_frame_id = name;

        ts.transform.translation.x = ptn.x;
        ts.transform.translation.y = ptn.y;
        ts.transform.translation.z = ptn.z;

        point_to_broadcast.push_back(ts);
    }

    void GMROS1::broadcast_ptn() {
        for (auto ptn : point_to_broadcast) {
            tf_pub.publish(ptn);
        }
    }
}
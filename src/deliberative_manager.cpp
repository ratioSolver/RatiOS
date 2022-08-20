#include "deliberative_manager.h"

using namespace std::chrono_literals;

namespace ratio::ros
{
    deliberative_manager::deliberative_manager() : Node("deliberative_manager"), timer(create_wall_timer(1s, std::bind(&deliberative_manager::tick, this))) {}
    deliberative_manager::~deliberative_manager() {}

    void deliberative_manager::tick() {}
} // namespace ratio::ros

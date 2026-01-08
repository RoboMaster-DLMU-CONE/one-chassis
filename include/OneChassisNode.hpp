#ifndef ONECHASSISNODE_HPP
#define ONECHASSISNODE_HPP
#include <OF/lib/Node/Node.hpp>

using namespace OF;

class OneChassisNode : public Node<OneChassisNode>
{
public:
    struct Meta
    {
        static constexpr size_t stack_size = 2048;
        static constexpr int priority = 5;
        static constexpr auto name = "one_chassis";
    };

    struct Config
    {
        const device* can_dev;
    };

    inline static Config config = {};

    bool init();

    void run();

    void cleanup()
    {
        // TODO: Clean up resources
    }
};


#endif //ONECHASSISNODE_HPP

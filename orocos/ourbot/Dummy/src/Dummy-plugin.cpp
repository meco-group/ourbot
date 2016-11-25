#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <iostream>

using namespace RTT;
using namespace std;


/**
 * An example plugin which can be loaded in a process.
 * Orocos plugins should provide at least these two functions:
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    if ( t == 0 )
        cout << "Plugin of Dummy loaded in process."<< endl;
    else
        cout << "Plugin of Dummy loaded in component: "<< t->getName() << endl;
    return true;
}

std::string getRTTPluginName()
{
    return "Dummy-example-plugin";
}

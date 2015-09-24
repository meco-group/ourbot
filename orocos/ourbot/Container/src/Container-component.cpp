#include "Container-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <typeinfo>

#define COMPONENTS_FUNCTION_ITERATOR_BOOL(func)	bool value = true; \
																								for(Components::iterator task = _components.begin(); task != _components.end() ; ++task) { \
																									value = value & (*task)->func(); \
																								}\
																								return value;

#define COMPONENTS_FUNCTION_ITERATOR_VOID(func)	for(Components::iterator task = _components.begin(); task != _components.end() ; ++task) { \
																									(*task)->func(); \
																								}

Container::Container(std::string const& name) : TaskContext(name,PreOperational){
	addOperation("addComponent", &Container::addComponent, this).doc("Adds a component to the component container and takes care of its configure, start, stop cleanup and update. All its ports become available under the same name.");
}

bool Container::configureHook(){
	COMPONENTS_FUNCTION_ITERATOR_BOOL(configure)
}

bool Container::startHook(){
	COMPONENTS_FUNCTION_ITERATOR_BOOL(start)
}

void Container::updateHook(){
	COMPONENTS_FUNCTION_ITERATOR_VOID(update)
}

void Container::stopHook() {
	COMPONENTS_FUNCTION_ITERATOR_VOID(stop)
}

void Container::cleanupHook() {
	COMPONENTS_FUNCTION_ITERATOR_VOID(cleanup)
}

bool Container::addComponent( const std::string& component )
{
	TaskContext* comp = getPeer(component);
	if ( !comp ) {
		RTT::log(RTT::Error) << "Could not add Component " << component << " : no such peer." << RTT::endlog();
		return false;
	}

	bool compStopped = comp->getTaskState() == RTT::base::TaskCore::PreOperational;
	bool thisStopped = getTaskState() == RTT::base::TaskCore::PreOperational;

	if(compStopped && thisStopped){
		// It is possible to add the task to the container
		_components.push_back(comp);

		Ports ports = comp->ports()->getPorts();
		for (Ports::iterator port = ports.begin(); port != ports.end() ; ++port) {
			std::string port_name = (*port)->getName();
			if(getPort(port_name) == NULL){
				//Port not yet in use: we can add it to the container components ports
				addPort(port_name, (**port));
			} else {
				RTT::log(RTT::Error) << "Could not add port " + port_name + " because it is already in use." << RTT::endlog();
			}
		}
	} else {
		RTT::log(RTT::Error) << "Could not add component " + component + " because it is already configured or running." << RTT::endlog();
	}

	return (compStopped && thisStopped);
}

Components Container::getComponents()
{
	return _components;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Container)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Container)

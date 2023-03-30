#include "dynamic-graph/python/module.hh"
#include "device.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph.sot.core.wrap");

  dynamicgraph::python::exposeEntity<SoTRobotArmDevice,
    bp::bases<dg::sot::Device> >();
  dynamicgraph::python::exposeEntity<DeviceToDynamic,
    bp::bases<dg::Entity> >()
    .def("setInputSize", &DeviceToDynamic::setInputSize);
}

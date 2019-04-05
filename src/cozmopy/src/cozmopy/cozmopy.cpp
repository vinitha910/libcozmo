#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace libcozmo {
namespace python {

void Cozmo(pybind11::module& m);

PYBIND11_MODULE(cozmopy, m)
{
	Cozmo(m);
}

}  // namespace python
}  // namesapce libcozmo
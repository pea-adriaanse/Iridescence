#include <iri/bxdfs/pyramidBRDF.hpp>

namespace pbrt {
std::string PyramidBRDF::ToString() const {
	return "PyramidBRDF";
}

namespace iri {
std::vector<double> backBounceTable;
BoundsThetaPhi thetaBounds;
BoundsThetaPhi phiBounds;
unsigned int thetaDimension;
unsigned int phiDimension;
}  // namespace iri

}  // namespace pbrt
#include <iri/bxdfs/specularBRDF.hpp>

namespace pbrt {

int SpecularBRDF::max_chain_depth = 0;
int SpecularBRDF::chained_depth = 0;

std::string SpecularBRDF::ToString() const { return "SpecularBRDF"; }

}  // namespace pbrt
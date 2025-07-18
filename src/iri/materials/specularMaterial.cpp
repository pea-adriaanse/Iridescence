#include <stdio.h>
#include <iri/materials/specularMaterial.hpp>

namespace pbrt {
SpecularMaterial *SpecularMaterial::Create(
	const TextureParameterDictionary &parameters, Image *normalMap,
	const FileLoc *loc, Allocator alloc) {
	Float reflectance = parameters.GetOneFloat("reflectance", 1.0);
	int reflectCount = parameters.GetOneInt("reflectCount", 5);

	return alloc.new_object<SpecularMaterial>(normalMap, reflectance, reflectCount);
}

std::string SpecularMaterial::ToString() const { return "SpecularMaterial"; }
}  // namespace pbrt

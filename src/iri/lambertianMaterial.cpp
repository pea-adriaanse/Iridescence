#include <iri/lambertianMaterial.hpp>

namespace pbrt {
LambertianMaterial *LambertianMaterial::Create(
	const TextureParameterDictionary &parameters, Image *normalMap,
	const FileLoc *loc, Allocator alloc) {
	FloatTexture reflectance =
		parameters.GetFloatTexture("reflectance", Float(0.5), alloc);
	return alloc.new_object<LambertianMaterial>(normalMap, reflectance);
}

std::string LambertianMaterial::ToString() const {
	return "LambertianMaterial";
}
}  // namespace pbrt
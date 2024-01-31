#include <iri/principledMaterial.hpp>

namespace pbrt {
PrincipledMaterial *PrincipledMaterial::Create(
	const TextureParameterDictionary &parameters, Image *normalMap,
	const FileLoc *loc, Allocator alloc) {
	FloatTexture metallic =
		parameters.GetFloatTexture("metallic", Float(0.5), alloc);
	FloatTexture roughness =
		parameters.GetFloatTexture("roughness", Float(0.5), alloc);
	// TODO: Get default complex eta from predefined material
	FloatTexture eta = parameters.GetFloatTexture("eta", Float(1), alloc);
	FloatTexture k = parameters.GetFloatTexture("k", Float(1), alloc);

	return alloc.new_object<PrincipledMaterial>(normalMap, metallic, roughness,
												eta, k);
}

std::string PrincipledMaterial::ToString() const {
	return "PrincipledMaterial";
}
}  // namespace pbrt
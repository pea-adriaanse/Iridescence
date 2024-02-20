#include <iri/materials/pyramidMaterial.hpp>

namespace pbrt {
PyramidMaterial *PyramidMaterial::Create(
	const TextureParameterDictionary &parameters, Image *normalMap,
	const FileLoc *loc, Allocator alloc) {
	FloatTexture _peakHeight =
		parameters.GetFloatTexture("peakHeight", Float(0.5), alloc);
	FloatTexture _angle =
		parameters.GetFloatTexture("angle", Float(0.5), alloc);
	FloatTexture _eta = parameters.GetFloatTexture("eta", Float(1), alloc);
	FloatTexture _k = parameters.GetFloatTexture("k", Float(1), alloc);

	return alloc.new_object<PyramidMaterial>(normalMap, _peakHeight, _angle,
												_eta, _k);
}

std::string PyramidMaterial::ToString() const { return "PyramidMaterial"; }
}  // namespace pbrt
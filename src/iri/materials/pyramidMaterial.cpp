#include <iri/materials/pyramidMaterial.hpp>

namespace pbrt {
PyramidMaterial *PyramidMaterial::Create(
	const TextureParameterDictionary &parameters, Image *normalMap,
	const FileLoc *loc, Allocator alloc) {
	FloatTexture _peakHeight = parameters.GetFloatTexture("peakHeight", Float(1), alloc);
	FloatTexture _angle = parameters.GetFloatTexture("angle", Float(54.7), alloc);
	FloatTexture _reflectance = parameters.GetFloatTexture("reflectance", Float(1), alloc);

	return alloc.new_object<PyramidMaterial>(normalMap, _peakHeight, _angle, _reflectance);
}

std::string PyramidMaterial::ToString() const { return "PyramidMaterial"; }
}  // namespace pbrt
#include <stdio.h>

#include <iri/materials/pyramidMaterial.hpp>

namespace pbrt {
PyramidMaterial *PyramidMaterial::Create(
	const TextureParameterDictionary &parameters, Image *normalMap,
	const FileLoc *loc, Allocator alloc) {
	// FloatTexture _peakHeight = parameters.GetFloatTexture("peakHeight",
	// Float(1), alloc);
	Float _peakHeight = parameters.GetOneFloat("peakHeight", 1.0);
	// FloatTexture _angle = parameters.GetFloatTexture("angle", Float(54.7),
	// alloc);
	Float _angle = parameters.GetOneFloat("angle", 54.7);
	// FloatTexture _reflectance = parameters.GetFloatTexture("reflectance",
	// Float(1), alloc); Float _reflectance =
	// parameters.GetOneFloat("reflectance", 1.0);
	int _reflectCount = parameters.GetOneInt("reflectCount", 1);
	int _reflectCountClamped = Clamp(_reflectCount, 1, PyramidBRDF::maxLevels);
	if (_reflectCount != _reflectCountClamped) {
		fprintf(stderr, "Clamped reflectCount %i to %i", _reflectCount,
				_reflectCountClamped);
	}

	bool _shadowPaul = parameters.GetOneBool("shadowPaul", false);
	std::string _setting = parameters.GetOneString("setting", "none");

	return alloc.new_object<PyramidMaterial>(normalMap, _peakHeight, _angle,
											 _reflectCount, _shadowPaul, _setting);
}

std::string PyramidMaterial::ToString() const { return "PyramidMaterial"; }
}  // namespace pbrt

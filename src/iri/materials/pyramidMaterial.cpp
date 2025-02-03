#include <stdio.h>

#include <iri/materials/pyramidMaterial.hpp>

namespace pbrt {
PyramidMaterial* PyramidMaterial::Create(const TextureParameterDictionary& parameters,
										 Image* normalMap,
										 const FileLoc* loc,
										 Allocator alloc) {
	// FloatTexture _peakHeight = parameters.GetFloatTexture("peakHeight",
	// Float(1), alloc);
	Float _peakHeight = parameters.GetOneFloat("peakHeight", 1.0);
	// FloatTexture _angle = parameters.GetFloatTexture("angle", Float(54.7),
	// alloc);
	Float _angle = parameters.GetOneFloat("angle", 54.7);
	// FloatTexture _reflectance = parameters.GetFloatTexture("reflectance",
	// Float(1), alloc); Float _reflectance =
	// parameters.GetOneFloat("reflectance", 1.0);
	int _reflectCountInt = parameters.GetOneInt("reflectCount", 1);
	if (_reflectCountInt <= 0)
		parameters.ReportError("reflectCount", "reflectCount cannot be zero or negative");
	unsigned int _reflectCountClamped =
		(unsigned int)Clamp(_reflectCountInt, 1, PyramidBRDF::maxLevels);
	if (_reflectCountClamped != _reflectCountInt)
		fprintf(stderr, "Clamped reflectCount %i to %i", _reflectCountInt, _reflectCountClamped);

	bool _shadowPaul = parameters.GetOneBool("shadowPaul", false);
	bool _rebounce = parameters.GetOneBool("rebounce", false);
	std::string _setting = parameters.GetOneString("setting", "none");
	Vector3f _woVector = parameters.GetOneVector3f("wo", Vector3f(0, 0, 0));
	std::string _distOutFile = parameters.GetOneString("distOutFile", std::string("dist.csv"));

	return alloc.new_object<PyramidMaterial>(normalMap, _peakHeight, _angle, _reflectCountClamped,
											 _shadowPaul, _rebounce, _setting, _woVector, _distOutFile);
}

std::string PyramidMaterial::ToString() const {
	return "PyramidMaterial";
}
}  // namespace pbrt

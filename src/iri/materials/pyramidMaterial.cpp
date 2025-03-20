#include <stdio.h>

#include <pbrt/util/print.h>
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
	Float _reflectance = Clamp(parameters.GetOneFloat("reflectance", 1.0), 0.0, 1.0);
	std::string _setting = parameters.GetOneString("setting", "none");
	Vector3f _woVector = parameters.GetOneVector3f("wo", Vector3f(0, 0, 0));
	std::string _distOutFile = parameters.GetOneString("distOutFile", std::string("dist.csv"));

	return alloc.new_object<PyramidMaterial>(normalMap, _peakHeight, _angle, _reflectCountClamped,
											 _shadowPaul, _rebounce, _reflectance, _setting,
											 _woVector, _distOutFile);
}

std::string PyramidMaterial::ToString() const {
	return "PyramidMaterial";
}

Float clamp01(Float x) {
	return (x <= 0) ? 0 : (x >= 1) ? 1 : x;
}

// Interpolated super-gaussian function fitted to 3 given reflectance profiles
Float lerp(Float x, Float L, Float R) {
	return x * (R - L) + L;
}

Float baseRadiance(Float theta) {
	if (theta <= 40)
		return 0.06;
	return lerp(theta / 40.0 - 1.0, 0.06, 0.5);
}

Float peakAmplitude(Float theta) {
	if (theta <= 40)
		0.84;
	return lerp(theta / 40.0 - 1.0, 0.84, 0.12);
}

Float peakCenter(Float theta) {
	if (theta <= 40)
		return lerp(theta / 40.0, 590, 550);
	return lerp(theta / 40.0 - 1.0, 550, 500);
}

Float peakWidth(Float theta) {
	return lerp(theta / 80.0, 40, 30);
}

Float fittedReflectance(Float lambda, Float theta) {
	return baseRadiance(theta) +
		   peakAmplitude(theta) * std::exp(-std::pow(std::pow(lambda - peakCenter(theta), 2) /
														 (2 * (std::pow(peakWidth(theta), 2))),
													 1.5));
}

Float PyramidMaterial::OpticalFilter(Float lambda, Float angle) {
	return clamp01(fittedReflectance(lambda, angle));
}
}  // namespace pbrt

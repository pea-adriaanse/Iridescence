// #include <iri/materials/opticalFilterMaterial.hpp>

// namespace pbrt {
// OpticalFilterMaterial *OpticalFilterMaterial::Create(
// 	const TextureParameterDictionary &parameters, Image *normalMap,
// 	const FileLoc *loc, Allocator alloc) {
// 	FloatTexture reflectance =
// 		parameters.GetFloatTexture("reflectance", Float(0.5), alloc);
// 	return alloc.new_object<OpticalFilterMaterial>(normalMap, reflectance);
// }

// std::string OpticalFilterMaterial::ToString() const {
// 	return "OpticalFilterMaterial";
// }
// }  // namespace pbrt
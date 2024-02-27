
#include <gtest/gtest.h>
#include <pbrt/pbrt.h>
#include <pbrt/util/math.h>
#include <pbrt/util/transform.h>
#include <stdio.h>

using namespace pbrt;

TEST(Iridescence, Normals) {
	Float angle = 54.7;
	Vector3f wo = Normalize(Vector3f(0.5,0.5,0.5));

	Vector3f tangent = Vector3f(0, 0, 1);
	tangent = RotateX(-angle)(tangent);

	// Determine normals & relative areas
	Vector3f normals[4];
	Float areas[4];
	for (int i = 0; i < 4; i++) {
		// Local frame has X along dpdu & Z along normal (with Y along (Z×X))
		normals[i] = RotateZ(90.0 * i)(tangent);
		areas[i] = Dot(normals[i], wo);
		if (areas[i] < 0) areas[i] = 0;
		std::cout << "N: " << normals[i] << "\nA: " << areas[i] << std::endl;
	}
}
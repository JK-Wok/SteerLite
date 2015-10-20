/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return false; // There is no collision
}

float dotProduct(Util::Vector v1, Util::Vector v2) {
	return v1.x * v2.x + v1.z * v2.z;
}

Util::Vector getSupport(std::vector<Util::Vector> shape, Util::Vector v) {
	float maxDot;
	Util::Vector supportVector;
	
	supportVector = shape[0];
	maxDot = dotProduct(shape[0], v);

	for(int i = 1; i < shape.size(); i++) {
		float currDot = dotProduct(shape[i], v);

		if(currDot > maxDot) {
			maxDot = currDot;
			supportVector = shape[i];
		}
	}

	return supportVector;
}

Util::Vector crossProduct(Util::Vector v1, Util::Vector v2) {
	Util::Vector prod;
	prod.x = v1.y*v2.z - v1.z*v2.y;
	prod.y = v1.z*v2.x - v1.x*v2.z;
	prod.z = v1.x*v2.y - v1.y*v2.x;

	return prod;
}

Util::Vector tripleProduct(Util::Vector v1, Util::Vector v2, Util::Vector v3) {
	return crossProduct(crossProduct(v1, v2), v3);
}

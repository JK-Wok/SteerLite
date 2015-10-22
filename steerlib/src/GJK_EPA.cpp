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

		if(currDot > maxDot && shape[i]!=v) { //Cant let support equal vector itself
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

bool GJK(std::vector<Util::Vector> shape1, std::vector<Util::Vector> shape2, std::vector<Util::Vector>& simplex) {
	
	const float EQUIVALENCERANGE = 1; //When checking if closest projection to origin is as close as original vector
	Util::Vector v1, v2, v3;
	Util::Vector arbitraryVector =  Util::Vector(1,0,0);

	//pick point on Minkowski difference
	v1 = getSupport(shape1, arbitraryVector) - getSupport(shape2, arbitraryVector*-1);
	//projection along v1 closest to origin
	v2 = getSupport(shape1, v1) - getSupport(shape2, v1*-1);

	if(fabsf(v1.length() - v2.length()) <= EQUIVALENCERANGE) //if closest projection to origin is equidistant from origin as original point, no intersect
		return false;

	//find new vertex for simplex in direction of the origin
	arbitraryVector = tripleProduct(v1-v2, v2*-1, v1-v2);
	v3 = getSupport(shape1, arbitraryVector) - getSupport(shape2, arbitraryVector*-1);
	
	while(true) { //CONDITION?
		float findOrigin1, findOrigin2; //used to determine what partition of the extended edge minkowski difference that the origin is in
		findOrigin1 = dotProduct(v1-v3, v3*-1);
		findOrigin2 = dotProduct(v2-v3, v3*-1);
		if(findOrigin1<0 && findOrigin2<0) //origin is outside simplex, no intersect
			return false;
		else if(findOrigin1>=0 && findOrigin2>=0) { //origin is in simplex, shapes intersect
			std::vector<Util::Vector> returnSimplex;
			returnSimplex.push_back(v1);
			returnSimplex.push_back(v2);
			returnSimplex.push_back(v3);
			simplex = returnSimplex;
			return true;
		}

		if(fabsf(v2.length() - v3.length()) <= EQUIVALENCERANGE) {
			return false;
		}

		if(findOrigin1>=0 && findOrigin2<0) { //keep v1, discard v2
			v2 = v3;
		}
		else if(findOrigin1<0 && findOrigin2>=0) { //keep v2, discard v1
			v1 = v2;
			v2 = v3;
		}

		arbitraryVector = tripleProduct(v1-v2, v2*-1, v1-v2);
		v3 = getSupport(shape1, arbitraryVector) - getSupport(shape2, arbitraryVector*-1);

	}
}

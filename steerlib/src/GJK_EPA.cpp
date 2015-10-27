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
	bool colliding;
	float depth;
	Util::Vector Pvector;
	std::vector<Util::Deque> simplex;
	colliding = GJK(_shapeA, _shapeB, simplex);
	if (colliding == true) {
		Pvector = EPA(_shapeA, _shapeB, simplex, depth);
		return(true, depth, Pvector);
	}
	else {
		return(false, 0, NULL);
	}
}

float dotProduct(Util::Vector v1, Util::Vector v2) {
	return v1.x * v2.x + v1.z * v2.z;
}

Util::Vector normal(Util::Vector v1, Util::Vector v2) {
	Util::Vector D;
	D.x = v1.x - v2.x;
	D.z = v1.z - v2.z;
	return D;
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

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector> shape1, std::vector<Util::Vector> shape2, std::vector<Util::Deque>& simplex) {
	
	Util::Vector v1, v2, v3;
	Util::Vector arbitraryVector =  Util::Vector(1,0,0);

	//pick point on Minkowski difference
	v1 = getSupport(shape1, arbitraryVector) - getSupport(shape2, arbitraryVector*-1);
	//std::cout << getSupport(shape1, arbitraryVector) << "  " << getSupport(shape2, arbitraryVector*-1) << std::endl;
	//std::cout << v1 << std::endl;
	//projection along v1 closest to origin
	v2 = getSupport(shape1, v1*-1) - getSupport(shape2, v1);
	//std::cout << v2 << std::endl;

	//find new vertex for simplex in direction of the origin
	arbitraryVector = tripleProduct(v1-v2, v2*-1, v1-v2);
	//std::cout << "new normal " << arbitraryVector << std::endl;
	v3 = getSupport(shape1, arbitraryVector) - getSupport(shape2, arbitraryVector*-1);
	//std::cout << v3 << std::endl;
	
	while(true) { //CONDITION?
		//if(fabsf(v2.length() - v3.length()) <= EQUIVALENCERANGE || v1.length() - v3.length() <= EQUIVALENCERANGE) {
		if(v2 == v3 || v1 == v3) { //if the vertex closest to the origin is already in the simplex, no intersect
			return false;
		}

		float findOrigin1, findOrigin2; //used to determine what partition of the extended edge minkowski difference that the origin is in
		findOrigin1 = dotProduct(v1-v3, v3*-1);
		//std::cout << v1-v3 << std::endl;
		findOrigin2 = dotProduct(v2-v3, v3*-1);
		if(findOrigin1<0 && findOrigin2<0) //origin is outside simplex, no intersect
			return false;
		else if(findOrigin1>=0 && findOrigin2>=0) { //origin is in simplex, shapes intersect
			std::vector<Util::deque> returnSimplex;
			returnSimplex.push_back(v1);
			returnSimplex.push_back(v2);
			returnSimplex.push_back(v3);
			simplex = returnSimplex;
			//std::cout << v1 << v2 << v3 << std::endl;
			return true;
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

		//std::cout << v1 << v2 << v3 << std::endl;
	}
}
Util::Vector SteerLib::GJK_EPA::EPA(std::vector<Util::Vector> shape1, std::vector<Util::Vector> shape2, std::vector<Util::Deque>& simplex, float Penetration_depth) {
	Util::Vector Penetration_vector;
	Util::Vector v1, v2, v3, projection;
	double epsilon = sqrt((1 + 2 ^ -52) - 1);
	double distance, p1d;

	for (int i = 0; i < 100; i++) {
		//get the edge to the origin provided by GJK
		v1 = simplex.front();
		simplex.pop_front();
		simplex.push_back(v1);
		v2 = simplex.front();
		simplex.push_front();
		//get the new support point in the direction of the edge
		v3 = getSupport(v1, v2);
		projection = normal(v1, v2);
		//find the distance
		distance = abs(v1.x*projection.x + v1.z*projection.z);
		p1d = sqrt((v1.x - v2.x) ^ 2 + (v1.z - v2.z) ^ 2);
		//if the distance and direction doesn't change
		if ((p1d - distance) < epsilon) {
			Penetration_depth = p1d;
			return Penetration_vector = projection;
		}
		simplex.push_back(v3);
	}
	Penetration_depth = p1d;
	return Penetration_vector = projection;
}


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
	std::vector<Util::Vector> simplex;
	bool col =  GJK(_shapeA, _shapeB, simplex);
	if(col==false)
		return false;
	//EPA(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
	return true;
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

std::vector<Util::Vector> getInteriorVertices(std::vector<Util::Vector> shape, int v1, int v2, int v3) {
	std::vector<Util::Vector> returnVector;

	for(int i=0; i<shape.size();i++) {
		if(shape.at(i)==shape.at(v1) || shape.at(i)==shape.at(v2) || shape.at(i)==shape.at(v3))
			continue; 
		if(dotProduct(shape.at(v2)-shape.at(v1), shape.at(i)-shape.at(v1))>0 && dotProduct(shape.at(v3)-shape.at(v1), shape.at(i)-shape.at(v1))>=0)
			if(dotProduct(shape.at(v1)-shape.at(v2), shape.at(i)-shape.at(v2))>=0 && dotProduct(shape.at(v3)-shape.at(v2), shape.at(i)-shape.at(v2))>=0)
				if(dotProduct(shape.at(v1)-shape.at(v3), shape.at(i)-shape.at(v3))>=0 && dotProduct(shape.at(v2)-shape.at(v3), shape.at(i)-shape.at(v3))>=0)
					returnVector.push_back(shape.at(i));
	}
	
	return returnVector;
}

int  getLeftmostVertex(std::vector<Util::Vector> shape) {
	Util::Vector returnVector = shape[0];
	int index = 0;

	for(int i=1; i<shape.size(); i++) {
		if(shape[i].x < returnVector.x)
			returnVector = shape[i];
			index = i;
	}

	return index;
}

int getIndex(std::vector<Util::Vector> group, Util::Vector v) {
	for(int i=0; i<group.size(); i++) {
		if(group[i] == v)
			return i;
	}
	return -1;
}

//decompose shape into triangles
bool convexDecomp(std::vector<std::vector<Util::Vector>> &decomp, std::vector<Util::Vector> shape) {
	while(shape.size() > 3) {
		int leftIndex = getLeftmostVertex(shape);
		//std::cout << "LEFTY " << shape.at(leftIndex) << std::endl;
		int prevIndex, nextIndex;
		//Create interior triangle
		if(leftIndex == 0) {
			prevIndex = shape.size()-1;
			nextIndex = 1;
		}
		else if(leftIndex == shape.size()-1) {
			prevIndex = leftIndex-1;
			nextIndex = 0;
		}
		else {
			prevIndex = leftIndex-1;
			nextIndex = leftIndex+1;
		}
		
		std::vector<int> interiorIndices;
		std::vector<Util::Vector> interiorVertices = getInteriorVertices(shape, prevIndex, leftIndex, nextIndex);
	
		Util::Vector leftInteriorVertex;
		//found concave vertex
		if(interiorVertices.size() > 0) {
			//std::cout << "FOUND CONCAVE" << std::endl;
			leftInteriorVertex = interiorVertices[getLeftmostVertex(interiorVertices)];
			
			//Cut shape in two, recursively call on both shapes
			int leftInteriorIndex = getIndex(shape, leftInteriorVertex);
			//std::cout << "LEFT INTERIOR VERTEX " << shape.at(leftInteriorIndex) << std::endl;
			std::vector<Util::Vector> shape1, shape2;
			if(leftIndex < leftInteriorIndex) {
				for(int i=leftIndex; i<=leftInteriorIndex; i++)
					shape1.push_back(shape[i]);
				for(int i=leftInteriorIndex; i<shape.size();i++)
					shape2.push_back(shape[i]);
				for(int i=0; i<=leftIndex; i++)
					shape2.push_back(shape[i]);
			}
			else {
				for(int i=leftInteriorIndex; i<=leftIndex; i++)
					shape1.push_back(shape[i]);
				for(int i=leftIndex; i<shape.size();i++)
					shape2.push_back(shape[i]);
				for(int i=0; i<=leftInteriorIndex; i++)
					shape2.push_back(shape[i]);

			}
			convexDecomp(decomp, shape1);
			convexDecomp(decomp, shape2);
			/*for(int i=0; i<decomp.size();i++) {
			for(int j=0; j<decomp.at(i).size(); j++)
				std::cout << decomp.at(i).at(j) << std::endl;
			std::cout << std::endl;
			}*/
			//std::cout << std::endl << std::endl;
			return true;
		}
		else {
			//Add triangle to decomp vector
			std::vector<Util::Vector> newTriangle;
			newTriangle.push_back(shape[prevIndex]);
			newTriangle.push_back(shape[leftIndex]);
			newTriangle.push_back(shape[nextIndex]);
			decomp.push_back(newTriangle);
	
			//Remove leftmost vertex (part of that triangle)
			shape.erase(shape.begin()+leftIndex);
		}
	}
	decomp.push_back(shape);
	/*for(int i=0; i<decomp.size();i++) {
		for(int j=0; j<decomp.at(i).size(); j++)
			std::cout << decomp.at(i).at(j) << " ";
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;*/
	return true;
}

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector> shape1, std::vector<Util::Vector> shape2, std::vector<Util::Vector>& simplex) {
	
	std::vector<std::vector<Util::Vector>> decomp1, decomp2;
	convexDecomp(decomp1, shape1);
	convexDecomp(decomp2, shape2);
	
	for(int i=0; i<decomp1.size(); i++) {
		for(int j=0; j<decomp2.size(); j++) {
			/*for(int k=0; k<decomp1.at(i).size(); k++) {
						std::cout << decomp1.at(i).at(k) << " ";
					}
					std::cout << std::endl;
					for(int k=0; k<decomp2.at(j).size(); k++) {
						std::cout << decomp2.at(j).at(k) << " ";
					}
			std::cout << std::endl;*/

			Util::Vector v1, v2, v3;
			Util::Vector arbitraryVector =  Util::Vector(1,0,0);

			//pick point on Minkowski difference
			v1 = getSupport(decomp1.at(i), arbitraryVector) - getSupport(decomp2.at(j), arbitraryVector*-1);
			//std::cout << getSupport(decomp1.at(i), arbitraryVector) << "  " << getSupport(decomp2.at(j), arbitraryVector*-1) << std::endl;
			//std::cout << v1 << std::endl;
			//projection along v1 closest to origin
			arbitraryVector = v1*-1;
			v2 = getSupport(decomp1.at(i), arbitraryVector) - getSupport(decomp2.at(j), arbitraryVector*-1);
			//std::cout << v2 << std::endl;
			if(dotProduct(v2, arbitraryVector) <= 0) { //point added to simplex does not pass the origin, no intersect
				continue;
			}

			//find new vertex for simplex in direction of the origin
			arbitraryVector = tripleProduct(v1-v2, v2*-1, v1-v2);
			//std::cout << "new normal " << arbitraryVector << std::endl;
			if(arbitraryVector == Util::Vector(0,0,0)) { //these points are aligned with the origin
				float tempDot = dotProduct(v2-v1, v1*-1);
				float squaredLength = (v2.x-v1.x)*(v2.x-v1.x) + (v2.z-v1.z)*(v2.z-v1.z);
				if(tempDot < 0 || tempDot > squaredLength) //origin is not between the points, no intersect
					continue;
			}
			v3 = getSupport(decomp1.at(i), arbitraryVector) - getSupport(decomp2.at(j), arbitraryVector*-1);
			//std::cout << v3 << std::endl;
			
			while(true) { //CONDITION?
				//if(fabsf(v2.length() - v3.length()) <= EQUIVALENCERANGE || v1.length() - v3.length() <= EQUIVALENCERANGE) {
				if(v2 == v3 || v1 == v3) { //if the vertex closest to the origin is already in the simplex, no intersect
					break;
				}

				//std::cout << dotProduct(v3, arbitraryVector) << " " << " " << std::endl;
				if(dotProduct(v3, arbitraryVector) <= 0) //new point for simplex does not pass the origin, no intersect
					break;

				float findOrigin1, findOrigin2; //used to determine what partition of the extended edge minkowski difference that the origin is in
				findOrigin1 = dotProduct(v1-v3, v3*-1);
				//std::cout << v1-v3 << std::endl;
				findOrigin2 = dotProduct(v2-v3, v3*-1);
				if(findOrigin1<0 && findOrigin2<0) //origin is outside simplex, no intersect
					break;
				else if(findOrigin1>=0 && findOrigin2>=0) { //origin is in simplex, shapes intersect
					std::vector<Util::Vector> returnSimplex;
					returnSimplex.push_back(v1);
					returnSimplex.push_back(v2);
					returnSimplex.push_back(v3);
					simplex = returnSimplex;
					std::cout << v1 << v2 << v3 << std::endl;
					/*for(int k=0; k<decomp1.at(i).size(); k++) {
						std::cout << decomp1.at(i).at(k) << " ";
					}
					std::cout << std::endl;
					for(int k=0; k<decomp2.at(j).size(); k++) {
						std::cout << decomp2.at(j).at(k) << " ";
					}*/

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
				v3 = getSupport(decomp1.at(i), arbitraryVector) - getSupport(decomp2.at(j), arbitraryVector*-1);

				//std::cout << v1 << v2 << v3 << std::endl;
			}
		}
	}
	return false;
}

bool SteerLib::GJK_EPA::EPA(std::vector<Util::Vector> shape1, std::vector<Util::Vector> shape2, std::vector<Util::Vector> simplex, float& depth, Util::Vector& mtv) {
	
	float DEPTHTOLERANCE = .01f;

	while(true) { //CONDITION?
		int index = simplex.size()-1;
	
		float penDepth;
		Util::Vector addSimplex;
	
		float closest;
		Util::Vector closestVector;
		Util::Vector edge;

		edge = simplex[index] - simplex[0];
		closestVector = tripleProduct(edge, simplex[index], edge);
		//std::cout << closestVector;
		closestVector = closestVector / closestVector.norm(); //normalize vector
		//std::cout << closestVector << std::endl;
		closest = dotProduct(simplex[index], closestVector);

		for(int i=0; i < simplex.size()-1; i++) {
			//std::cout << simplex.size() << std::endl;
			//std::cout << simplex[i] << std::endl;
			Util::Vector testVector;
			float testDepth;
			
			edge = simplex[i+1] - simplex[i];
			testVector = tripleProduct(edge, simplex[i], edge);
			//std::cout << testVector;
			testVector = testVector / testVector.norm(); //normalize vector
			//std::cout << testVector << std::endl;
			testDepth = dotProduct(simplex[i], testVector);

			if(testDepth < closest) {
				closest = testDepth;
				closestVector = testVector;
				index = i;	
			}
		}

		addSimplex = getSupport(shape1, closestVector) - getSupport(shape2, closestVector*-1);
		penDepth = dotProduct(addSimplex, closestVector);

		//std::cout << closestVector << addSimplex << std::endl << std::endl;
		if(penDepth <= DEPTHTOLERANCE) {
			depth = penDepth;
			mtv = closestVector;
			return true;
		}
		

		if(index == simplex.size()-1) {
			if(simplex[simplex.size()-1] == addSimplex || simplex[0] == addSimplex) {
				depth = penDepth;
				mtv = closestVector;
				return true;
			}
			simplex.push_back(addSimplex);
		}
		else {
			std::vector<Util::Vector>::iterator it = simplex.begin();
			for(int i=0; i <= index; i++) {
				it++;
			}
			//std::cout << *it << std::endl;
			if(*it == addSimplex || (*(it-1) == addSimplex)) {
				depth = penDepth;
				mtv = closestVector;
				return true;
			}
			simplex.insert(it, addSimplex);
		}
	}
}

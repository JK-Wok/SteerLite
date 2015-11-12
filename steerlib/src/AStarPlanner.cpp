//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

//PART 1-4 MACROS
#define MANHATTAN false
//0 for no pref, 1 for higher preferred, -1 for lower preferred
#define GFAVOR 0
#define DIAGMOD false
#define HEURISTICWEIGHT 1.0

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}


	double AStarPlanner::calcHeuristic(Util::Point start, Util::Point goal, bool manhattan) {
		if(MANHATTAN) {
			return abs(start.x - goal.x) + abs(start.z - goal.z);
		}	
		else {
			return (goal-start).length();
		}
	}

	class priorityQueue {
	public:
		std::vector<SteerLib::AStarPlannerNode> heap;
		double max = 10000;
	
		void siftUp(int heapIndex) {
			int i = heapIndex;
			while(i != 0) {
				if(heap[i].f > heap[(i-1)/2].f) {
					SteerLib::AStarPlannerNode tempNode = heap[(i-1)/2];
					heap[(i-1)/2] = heap[i];
					heap[i] = tempNode;
					i = (i-1)/2;
				}
				else if(heap[i].f == heap[(i-1)/2].f) {
					int gPref = GFAVOR;
					if(gPref == -1 && heap[i].g < heap[(i-1)/2].g) {
						SteerLib::AStarPlannerNode tempNode = heap[(i-1)/2];
						heap[(i-1)/2] = heap[i];
						heap[i] = tempNode;
						i = (i-1)/2;

					}
					else if(gPref == 1 && heap[i].g > heap[(i-1)/2].g) {
						SteerLib::AStarPlannerNode tempNode = heap[(i-1)/2];
						heap[(i-1)/2] = heap[i];
						heap[i] = tempNode;
						i = (i-1)/2;
					}
					else {
						return;
					}
				}
				else {
					return;
				}
			}
		}

		void siftDown(int heapIndex) {
			int i = heapIndex;
			while(2*i+1 < heap.size()) { //???? TEST THIS
				int swapIndex;
				//1 child
				if(2*i+2 == heap.size()) {
					swapIndex = 2*i+1;
				}
				else {
					if(heap[2*i+1].f > heap[2*i+2].f)
						swapIndex = 2*i+1;
					else
						swapIndex = 2*i+2;
				}

				if(heap[i].f < heap[swapIndex].f) {
					SteerLib::AStarPlannerNode tempNode = heap[swapIndex];
					heap[swapIndex] = heap[i];
					heap[i] = tempNode;
					i = swapIndex;
				}
				else if(heap[i].f == heap[swapIndex].f) {
					int prefG = GFAVOR;
					if(prefG == -1 && heap[swapIndex].g < heap[i].g) {
						SteerLib::AStarPlannerNode tempNode = heap[swapIndex];
						heap[swapIndex] = heap[i];
						heap[i] = tempNode;
						i = swapIndex;
					}
					else if(prefG == 1 && heap[swapIndex].g > heap[i].g) {
						SteerLib::AStarPlannerNode tempNode = heap[swapIndex];
						heap[swapIndex] = heap[i];
						heap[i] = tempNode;
						i = swapIndex;
					}
					else {
						return;
					}
				}
				else {
					return;
				}
			}
		}
	
		void insert(SteerLib::AStarPlannerNode newNode) {
			heap.push_back(newNode);
			siftUp(heap.size()-1);
		}

		SteerLib::AStarPlannerNode pop() {
			SteerLib::AStarPlannerNode returnNode = heap.front();
			heap.front() = heap.back();
			heap.pop_back();
			siftDown(0);
			return returnNode;
		}
		
		void update(int t, SteerLib::AStarPlannerNode parentNode, int xIncr, int zIncr, SteerLib::GridDatabase2D * gSpatialDatabase, Util::Point goal) {
			
			unsigned int parentX, parentZ;
			//if(t<20 && parentNode.g!=0)
			//	std::cout << "PARENT IN UPDATE " << parentNode.parent->point << std::endl;
			gSpatialDatabase->getGridCoordinatesFromIndex(parentNode.gridIndex, parentX, parentZ);
			int newIndex = gSpatialDatabase->getCellIndexFromGridCoords(parentX+xIncr, parentZ+zIncr);
			Util::Point newPoint;
			gSpatialDatabase->getLocationFromIndex(newIndex, newPoint);
			double newG = parentNode.g+1;
			//diagonal movement costs more (root2)
			if(DIAGMOD && zIncr != 0 && xIncr != 0) {
				newG += 0.414; //root2 rounded
			}
			SteerLib::AStarPlannerNode newNode = SteerLib::AStarPlannerNode(newPoint, newG, max-(newG+HEURISTICWEIGHT*calcHeur(newPoint, goal, false)), parentNode.gridIndex);
			newNode.gridIndex = newIndex;
			//if(t<20)
			std::cout << newNode.point << " " << newNode.f << std::endl;
			/*if(t<20){
				std::cout << newNode.point << "   " << (*newNode.parent).point << "  ";
				if(newNode.parent->parent != NULL)
					std::cout << newNode.parent->parent->point;
				std::cout << std::endl;}*/

			int i = 0;
			for(; i < heap.size(); i++) {
				if(heap[i].gridIndex == newNode.gridIndex) {
					break;
				}
			}

			//Insert if not found
			if(i >= heap.size()) {
				insert(newNode);
				return;
			}

			if(newNode.f > heap[i].f) {
				heap[i] = newNode;
				siftUp(i);
			}
		}
		private:
		double calcHeur(Util::Point start, Util::Point goal, bool manhattan) {
		if(MANHATTAN) {
			return abs(start.x - goal.x) + abs(start.z - goal.z);
		}	
		else {
			return (goal-start).length();
		}
	}

	};

	//search set for node with following grid index
	int searchClosed(int t, std::vector<SteerLib::AStarPlannerNode> s, int gIndex, int xIncr, int zIncr, SteerLib::GridDatabase2D * gSpatialDatabase) {
		//if(t<20 && baseNode.parent != NULL)
		//	std::cout << "Y " << baseNode.parent->point << " " << baseNode.point << std::endl;
		unsigned int xGridPos, zGridPos;
		gSpatialDatabase->getGridCoordinatesFromIndex(gIndex, xGridPos, zGridPos);
		int testIndex = gSpatialDatabase->getCellIndexFromGridCoords(xGridPos+xIncr, zGridPos+zIncr);
		//if(t<20 && baseNode.parent != NULL)
		//	std::cout << "Y " << baseNode.parent->point << std::endl;


		for(int i=0; i<s.size(); i++) {
			if(s[i].gridIndex == testIndex)
				return i;
		}
		return -1;
	}

		bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//Build node trees/sets
		priorityQueue openNodes = priorityQueue();
		std::vector<SteerLib::AStarPlannerNode> closedNodes;

		double max = 10000;

		//Build start node
		SteerLib::AStarPlannerNode startNode = SteerLib::AStarPlannerNode(start, 0, max-(0+HEURISTICWEIGHT*calcHeuristic(start, goal, false)), -1);
		startNode.gridIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		openNodes.insert(startNode);
	
		int t = 0;	
		while(!openNodes.heap.empty()) {
			/*if(t<20) {
			std::cout << std::endl << std::endl;
			for(int i=0; i<openNodes.heap.size(); i++)
				std::cout << openNodes.heap[i].f << "  ";
			std::cout << std::endl;}*/
			SteerLib::AStarPlannerNode currNode = openNodes.pop();
			closedNodes.push_back(currNode);
			//if(t<20)
				std::cout << std::endl << currNode.point << "   " << currNode.f << "  " << currNode.parentIndex << std::endl;
			t++;
			
			//Found goal, trace back for path
			if(currNode.gridIndex == gSpatialDatabase->getCellIndexFromLocation(goal)) {
				std::cout << currNode.gridIndex << " FOUND PATH  " << currNode.point << std::endl;
				currNode.point = goal;
				std::cout << currNode.point << "  " << currNode.parentIndex << std::endl;
				int currIndex = currNode.gridIndex;
				std::cout << currIndex << std::endl;
				int tt = 0;
				while(currIndex != gSpatialDatabase->getCellIndexFromLocation(start)) {
					Util::Point currPoint;
					gSpatialDatabase->getLocationFromIndex(currIndex, currPoint);
					std::cout << currPoint << std::endl;
					agent_path.insert(agent_path.begin(), currPoint);
					if(tt < 20)
						std::cout << currPoint << std::endl;
					tt++;
					std::cout << closedNodes[searchClosed(t, closedNodes, currIndex, 0, 0, gSpatialDatabase)].parentIndex << std::endl;
					currIndex = closedNodes[searchClosed(t, closedNodes, currIndex, 0, 0, gSpatialDatabase)].parentIndex;
					//if(currIndex == -1)
					//	return false;
					//std::cout << goal << " " << currIndex << "  " << currPoint << std::endl;
				}	
				agent_path.insert(agent_path.begin(), start);
				for(int i=0; i<agent_path.size();i++) {
					std::cout << agent_path[i] << " ";
				}

				return true;
			}
			//TODO
			//Update neighbors
			unsigned int xIndex, zIndex;
			gSpatialDatabase->getGridCoordinatesFromIndex(currNode.gridIndex, xIndex, zIndex);
			//SteerLib::AStarPlannerNode savedParent = *(currNode.parent);
			if(t<20)
				;//std::cout << savedParent.point << "    " << std::endl;
			//aligned neighbors
			//check if neighbor is valid (out of bounds of grid)
			//left
			if(xIndex-1 >= 0 && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex-1, zIndex))) {
				//Is in closed set?
				if(searchClosed(t, closedNodes, currNode.gridIndex, -1, 0, gSpatialDatabase)==-1) {	
					//Is in open set?
					openNodes.update(t, currNode, -1, 0, gSpatialDatabase, goal);
				}

			}
			//right
			if(xIndex+1 < gSpatialDatabase->getNumCellsX() && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex+1, zIndex))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, 1, 0, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, 1, 0, gSpatialDatabase, goal);
				}
			}
			//top
			if(zIndex-1 >= 0 && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex, zIndex-1))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, 0, -1, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, 0, -1, gSpatialDatabase, goal);
				}
			}
			//bottom
			if(zIndex+1 < gSpatialDatabase->getNumCellsZ() && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex, zIndex+1))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, 0, 1, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, 0, 1, gSpatialDatabase, goal);
				}
			} 
				
			//diagonal neighbors
			//top-left
			if(zIndex-1>=0 && xIndex-1>=0 && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex-1, zIndex-1))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, -1, -1, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, -1, -1, gSpatialDatabase, goal);
				}
			} 
			//top-right
			if(zIndex+-1>=0 && xIndex+1<gSpatialDatabase->getNumCellsX() && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex+1, zIndex-1))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, 1, -1, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, 1, -1, gSpatialDatabase, goal);
				}
			} 
			//bottom-left
			if(zIndex+1<gSpatialDatabase->getNumCellsZ() && xIndex-1>=0 && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex-1, zIndex+1))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, -1, 1, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, -1, 1, gSpatialDatabase, goal);
				}
			} 
			//bottom-right
			if(zIndex+1<gSpatialDatabase->getNumCellsZ() && xIndex+1<gSpatialDatabase->getNumCellsX() && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(xIndex+1, zIndex+1))) {
				if(searchClosed(t, closedNodes, currNode.gridIndex, 1, 1, gSpatialDatabase)==-1) {	
					openNodes.update(t, currNode, 1, 1, gSpatialDatabase, goal);
				}
			} 

		}

		//TODO
		std::cout<<"\nIn A*";

		return false;
	}
}

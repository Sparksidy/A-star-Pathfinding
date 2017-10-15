#include <Stdafx.h>
#include <iostream>
#include <movement.h>

#include "AStar.h"

AStar::AStar()
{
	//Initialize the lists
}

AStar::~AStar()
{
	//Clear the open and closed lists
	_ClosedList.clear();
	_OpenList.clear();
}

bool AStar::FindAStarPath( Node* start,  Node* end, WaypointList& waypointList)
{
	//TODO: If straight line path then dont do pathfinding :SLO


	//Push the starting node first
	 start->List = OPEN_LIST;
	_OpenList.push_back(start);
	

	while (!_OpenList.empty())
	{
		
		//Pop cheapest node off the Open list
		Node* current_node = PopCheapestNode();

	

		if (current_node->xPos == end->xPos && current_node->yPos == end->yPos)
		{
			//Reconstruct Path	
			CalculatePathFromGoal(*start, *current_node, waypointList);

			return true;
		}
		

		int xDX = 0;
		int yDY = 0;
		//Explore all the neighbours of the node
		for (int i = 0; i < dir; i++)
		{
			xDX = current_node->xPos + dx[i];
			yDY = current_node->yPos + dy[i];

			
			//Range Checking or if its not in closed list then use the node
			if (!(xDX < 0 || xDX > (MAP_X_SIZE -1) || yDY < 0 || yDY > (MAP_Y_SIZE -1) || CheckInClosedList(xDX, yDY) || g_terrain.IsWall(yDY, xDX))) 
			{
				
				//Generate a new node if not in open or closed list
				Node node;
		
				node.xParent	=	current_node->xPos;
				node.yParent	=	current_node->yPos;

				node.xPos		=	xDX;
				node.yPos		=	yDY;
				node.TotalCost  =	0;
				node.givenCost  =	0;

				
				//Check for diagnols and ignore them
				if ((dx[i] != 0 && dy[i] != 0))
				{
					node.givenCost = current_node->givenCost + 1.414f;

					if (g_terrain.IsWall(current_node->yPos + dy[i], current_node->xPos) || g_terrain.IsWall(current_node->yPos, current_node->xPos + dx[i]))
					{
						continue;
					}

				}


				else
				{
					node.givenCost = current_node->givenCost + 1.0f;
				}

				//Calculate the H Cost
				float hCost = OctileHCost(node, *end);

				//Final Cost
				node.TotalCost = node.givenCost + (hCost * _hueristicWeight);

				/*Check if the node is already in the open or closed list*/
				Node* open_Node  =  FindInOpenList(xDX, yDY);
				Node* close_Node =  FindInClosedList(xDX, yDY);

				//If the node is already in open list
				if (open_Node)
				{
					// if the new node is cheaper
					
					if (node.TotalCost < open_Node->TotalCost)
					{
						//Take the expensive node in OpenList off the list
						 open_Node->List = NONE;
						_OpenList.remove(open_Node);
						

						//Put the new one in the list
						node.List = OPEN_LIST;
						_OpenList.push_back(new Node(node));
						g_terrain.SetColor(node.yPos, node.xPos, DEBUG_COLOR_BLUE);
						
					}
					

				}
				else if (close_Node)
				{
				
					if (node.TotalCost < close_Node->TotalCost)
					{
						//Take the expensive node in Closed List off the list
						close_Node->List = NONE;
						_ClosedList.remove(close_Node);

						//Put the new one in the list
						node.List = OPEN_LIST;
						_OpenList.push_back(new Node(node));
						g_terrain.SetColor(node.yPos, node.xPos, DEBUG_COLOR_BLUE);
					}
					
				}
				else
				{
					//If not in the open list or closed list then add it to the open list and update the enum value
					 node.List = OPEN_LIST;
					_OpenList.push_back(new Node(node));
					g_terrain.SetColor(node.yPos, node.xPos, DEBUG_COLOR_BLUE);
				}

			}
		}

		//After exploring the neighbours of the current node push it to the closed list
		 current_node->List = CLOSED_LIST;
		_ClosedList.push_back(current_node);
		g_terrain.SetColor(current_node->yPos, current_node->xPos, DEBUG_COLOR_YELLOW);
	

	}

	//If openList empty, then no path possible
	if (_OpenList.empty())
		return false;

}

Node* AStar::PopCheapestNode()
{
	float minCost = INT_MAX;
	Node* node;
	std::list<Node*>::iterator iterator;
	for (iterator = _OpenList.begin(); iterator != _OpenList.end(); ++iterator) {
		if ((*iterator)->TotalCost < minCost)
		{
			minCost = (*iterator)->TotalCost;
			node = (*iterator);
		}
	}
	//Remove it from the openList and use it to explore
	 node->List = NONE;
	_OpenList.remove(node);

	return node;
}



float AStar::OctileHCost(Node& current, Node& goal)
{
	int cur_x = current.xPos;
	int cur_y = current.yPos;

	int goal_x = goal.xPos;
	int goal_y = goal.yPos;

	int xDiff = abs(goal_x - cur_x);
	int yDiff = abs(goal_y - cur_y);

	float a = min(xDiff, yDiff) * sqrt(2);
	float b = max(xDiff, yDiff);
	float c = min(xDiff, yDiff);

	float dist = a + b - c;

	return dist;

}

float AStar::EuclideanHCost(Node * current, Node * goal)
{
	int cur_x = current->xPos;
	int cur_y = current->yPos;

	int goal_x = goal->xPos;
	int goal_y = goal->yPos;

	int xDiff = goal_x - cur_x;
	int yDiff = goal_y - cur_y;

	float dist = sqrt((xDiff * xDiff) + (yDiff * yDiff));

	return dist;
}



Node* AStar::FindInOpenList(int x, int y)
{
	std::list<Node*>::const_iterator iterator;
	Node* node;
	for (iterator = _OpenList.begin(); iterator != _OpenList.end(); ++iterator) {

		//If the nodes are same then return the node from the list
		if ((*iterator)->xPos == x && (*iterator)->yPos == y)
		{
			node = (*iterator);
			return node;
		}
	}

	return nullptr;
}

Node* AStar::FindInClosedList(int x, int y)
{
	if (!_ClosedList.empty())
	{
		std::list<Node*>::iterator iterator;
		Node* node;
		for (iterator = _ClosedList.begin(); iterator != _ClosedList.end(); ++iterator)
		{

			//If the nodes are same return the node from the list
			if ((*iterator)->xPos == x && (*iterator)->yPos == y)
			{
				node = (*iterator);
				return node;
			}
		}
	}
	

	return nullptr;
}

bool AStar::CheckInClosedList(int x, int y)
{
	if (!_ClosedList.empty())
	{
		std::list<Node*>::iterator iterator;
		for (iterator = _ClosedList.begin(); iterator != _ClosedList.end(); ++iterator) {

			//If the nodes are equal
			if ((*iterator)->xPos == x && (*iterator)->yPos == y)
			{
				return true;
			}
		}

		
	}

	return false;
}

bool AStar::CheckInOpenList(int x, int y)
{
	if (!_OpenList.empty())
	{
		std::list<Node*>::iterator iterator;
		for (iterator = _OpenList.begin(); iterator != _OpenList.end(); ++iterator) {

			//If the nodes are equal
			if ((*iterator)->xPos == x && (*iterator)->yPos == y)
			{
				return true;
			}
		}

		
	}
	return false;
}

void AStar::CalculatePathFromGoal(Node& start, Node & end, WaypointList & list)
{
	
	Node current = end;
	while (current.xPos != start.xPos || current.yPos != start.yPos)
	{
			D3DXVECTOR3 spot = g_terrain.GetCoordinates(current.yPos, current.xPos);
			list.push_back(spot);
			//g_terrain.SetColor(current.yPos, current.xPos, DEBUG_COLOR_YELLOW);

			Node* node = FindInClosedList(current.xParent, current.yParent);
			if (node)
			{
				current.xPos = node->xPos;
				current.yPos = node->yPos;

				current.xParent = node->xParent;
				current.yParent = node->yParent;
			}
			else {
				g_terrain.SetColor(current.yPos, current.xPos, DEBUG_COLOR_RED);
			}

		
	}

	list.reverse();

}




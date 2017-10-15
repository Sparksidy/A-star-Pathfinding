#pragma once
#include <list>


#define MAX_NUMBER_OF_NODES 1600
#define MAP_X_SIZE 40
#define MAP_Y_SIZE 40

class Movement;
enum OnList {
	OPEN_LIST,
	CLOSED_LIST,
	NONE
};

struct Node {
	int xParent;
	int yParent;

	
	//Current Position
	int xPos;
	int yPos;

	//Final Cost
	float TotalCost;

	//The G cost
	float givenCost;

	//Which list this node is on? By default its on NONEdgdfg
	OnList List = NONE;

	const Node& operator=(const Node& rhs)
	{
		//Self check: TODO

		this->givenCost = rhs.givenCost;
		this->List = rhs.List;
		this->TotalCost = rhs.TotalCost;
		
		this->xParent = rhs.xParent;
		this->yParent = rhs.yParent;
		
		this->xPos = rhs.xPos;
		this->yPos = rhs.yPos;
		
		return *this;
	}


	//Check if the nodes are equal
	bool operator==(const Node& rhs)
	{
		if ((this->xPos == rhs.xPos) && (this->yPos == rhs.yPos) )
			return true;
		else
			return false;
	}

};

class AStar {

public:
	AStar();
	~AStar();

	bool FindAStarPath(Node* start, Node* end, WaypointList& list);
	
private:
	float OctileHCost(Node& current, Node& goal);
	float EuclideanHCost(Node* current, Node* goal);


	Node* PopCheapestNode();

	Node* FindInOpenList(int x, int y);
	Node* FindInClosedList(int x, int y);
	
	bool CheckInClosedList(int x, int y);
	bool CheckInOpenList(int x, int y);

	void CalculatePathFromGoal(Node& start, Node& end, WaypointList& list);
	
	
private:
	 std::list<Node*> _OpenList;
	 std::list<Node*> _ClosedList;
	 
	 const float _hueristicWeight = 1.01f;

	 int dir = 8;
	 int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
	 int dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

	
};

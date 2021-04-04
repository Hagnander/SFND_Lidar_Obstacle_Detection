/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertSubNodes(std::vector<float> point, int id, int depth, Node** node)
	{
		if(*node == NULL)
			*node = new Node (point,id);
		else
		{
			int currentDepth = depth % 2;
		
			if(point[currentDepth] < ((*node)->point[currentDepth]))
				insertSubNodes(point,id,depth+1,&((*node)->left));
			else
				insertSubNodes(point,id,depth+1,&((*node)->right));
		}
	}
	void insert(std::vector<float> point, int id)
	{
		insertSubNodes(point, id, 0, &root);	
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};





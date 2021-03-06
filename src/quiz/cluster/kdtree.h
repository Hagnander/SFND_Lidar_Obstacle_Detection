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
			int currentDepth = depth % 3;
		
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

	
	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL) 
		{
			if (node->point[0] >= target[0] - distanceTol &&
				node->point[0] <= target[0] + distanceTol &&
				node->point[1] >= target[1] - distanceTol &&
				node->point[1] <= target[1] + distanceTol &&
				node->point[2] >= target[2] - distanceTol &&
				node->point[2] <= target[2] + distanceTol &&
				std::sqrt(std::pow(node->point[0] - target[0], 2) +	
				std::pow(node->point[1] - target[1], 2) +
				std::pow(node->point[2] - target[2], 2)) <= distanceTol) 
			{
				ids.push_back(node->id);
			}
			if ((target[depth % 3] - distanceTol) < node->point[depth % 3])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if ((target[depth % 3] + distanceTol) > node->point[depth % 3])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
    	}
	}
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};





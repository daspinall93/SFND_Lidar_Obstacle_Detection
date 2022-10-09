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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertionHelper(Node *&node, uint depth, std::vector<float> point, int id)
	{
		if (node == nullptr)
			node = new Node(point, id);
		else
		{
			uint dim = depth % 2;

			if (point[dim] < node->point[dim])
				insertionHelper(node->left, depth + 1, point, id);
			else
				insertionHelper(node->right, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertionHelper(root, 0, point, id);

	}

	void searchHelper(Node *node, uint depth, const std::vector<float> &target, float distanceTol, std::vector<int>& ids) const
	{
		if (node == nullptr)
			return;

		// Check if current point is within the box
		float dx = node->point[0] - target[0];
		float dy = node->point[1] - target[1];
		if (sqrt(dx * dx + dy * dy) < distanceTol)
			ids.push_back(node->id);		
		
		uint dim = depth % 2;
		// Check if extreme of box in dimension is less than current node
		if (target[dim] - distanceTol < node->point[dim])
			searchHelper(node->left, depth + 1, target, distanceTol, ids);
		// Check if extreme of box in dimension is greater than current node
		if (target[dim] + distanceTol > node->point[dim])
			searchHelper(node->right, depth + 1, target, distanceTol, ids);
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) const
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
	

};





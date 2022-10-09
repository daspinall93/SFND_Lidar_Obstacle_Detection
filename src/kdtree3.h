/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
template <typename T>
struct Node
{
	T point;
	int id;
	Node* left;
	Node* right;

	Node(T arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template <typename T>
struct KdTree3
{
	Node<T>* root;

	KdTree3()
	: root(NULL)
	{}

	~KdTree3()
	{
		delete root;
	}

	void insertionHelper(Node<T> *&node, uint depth, T point, int id)
	{
		if (node == nullptr)
			node = new Node<T>(point, id);
		else
		{
			uint dim = depth % 3;

			if (point.data[dim] < node->point.data[dim])
				insertionHelper(node->left, depth + 1, point, id);
			else
				insertionHelper(node->right, depth + 1, point, id);
		}
	}

	void insert(T point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertionHelper(root, 0, point, id);

	}

	void searchHelper(Node<T> *node, uint depth, const T &target, float distanceTol, std::vector<int>& ids) const
	{
		if (node == nullptr)
			return;

		// Check if current point is within the box
		float dx = node->point.data[0] - target.data[0];
		float dy = node->point.data[1] - target.data[1];
		float dz = node->point.data[2] - target.data[2];
		if (sqrt(dx * dx + dy * dy + dz * dz) < distanceTol)
			ids.push_back(node->id);		
		
		uint dim = depth % 3;
		// Check if extreme of box in dimension is less than current node
		if (target.data[dim] - distanceTol < node->point.data[dim])
			searchHelper(node->left, depth + 1, target, distanceTol, ids);
		// Check if extreme of box in dimension is greater than current node
		if (target.data[dim] + distanceTol > node->point.data[dim])
			searchHelper(node->right, depth + 1, target, distanceTol, ids);
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(T target, float distanceTol) const
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
	

};





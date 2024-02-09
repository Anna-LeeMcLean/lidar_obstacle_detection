/* \author Anna-Lee McLean */

#include "render/render.h"
#include <cmath>


// Structure to represent 3D node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree3D
{
	Node<PointT>* root;

	KdTree3D()
	: root(NULL)
	{}

	~KdTree3D()
	{
		delete root;
	}

	void insertHelper(Node<PointT>*& tree_node, Node<PointT>* new_node, int depth){

		if(tree_node == NULL){
			tree_node = new_node;
			return;
		}
		if (depth%3==0){
			if (new_node->point.x < tree_node->point.x){
				insertHelper(tree_node->left, new_node, depth+1);
			}
			else{
				insertHelper(tree_node->right, new_node, depth+1);
			}
		}
		else if (depth%3==1){
			if (new_node->point.y < tree_node->point.y){
				insertHelper(tree_node->left, new_node, depth+1);
			}
			else{
				insertHelper(tree_node->right, new_node, depth+1);
			}
		}
		else if (depth%3==2){
			if (new_node->point.z < tree_node->point.z){
				insertHelper(tree_node->left, new_node, depth+1);
			}
			else{
				insertHelper(tree_node->right, new_node, depth+1);
			}
		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		Node<PointT>* node = new Node<PointT>{point, id};
		int depth = 0;
		insertHelper(root, node, depth);
	}

	void searchHelper(PointT target, const Node<PointT>* currentNode, std::vector<int>& ids, float distanceTol, int depth){

		if (currentNode == NULL){
			return;
		}
		/*
		// check if currentNode in box
		float dx = fabs(target.x - currentNode->point.x);
		float dy = fabs(target.y - currentNode->point.y);
		float dz = fabs(target.z - currentNode->point.z);
		if ((dx <= distanceTol) && (dy <= distanceTol) && (dz <= distanceTol)){
			//inside the box. check if euclidean distance is less than tolerance
			float d = sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2));
			if (d <= distanceTol)
				ids.push_back(currentNode->id);
		}
		*/
		bool dx1 = currentNode->point.x <= (target.x + distanceTol);
		bool dx2 = currentNode->point.x >= (target.x - distanceTol);

		bool dy1 = currentNode->point.y <= (target.y + distanceTol);
		bool dy2 = currentNode->point.y >= (target.y - distanceTol);

		bool dz1 = currentNode->point.z <= (target.z + distanceTol);
		bool dz2 = currentNode->point.z >= (target.z - distanceTol);

		if (dx1 && dx2 && dy1 && dy2 && dz1 && dz2){
			//inside the box. check if euclidean distance is less than tolerance
			float dx = target.x - currentNode->point.x;
			float dy = target.y - currentNode->point.y;
			float dz = target.z - currentNode->point.z;
			float d = sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2));
			if (d <= distanceTol)
				ids.push_back(currentNode->id);
		}
		
		if (depth%3==0){
			// split tree using x axis
			if ((target.x-distanceTol) < currentNode->point.x){
				// check left if the left most edge of the box is more left than the current nodes's x. 
				// This means that the node is to the right of the left edge and we need to check
				// if its left node is also to the right of the left most edge.
				searchHelper(target, currentNode->left, ids, distanceTol, depth+1);
			}
			if ((target.x+distanceTol) > currentNode->point.x){
				// check right if the right most edge of the box is further right than the node.
				// We need to check if the node's right node is between the node and the right edge.
				searchHelper(target, currentNode->right, ids, distanceTol, depth+1);
			}
		}
		else if (depth%3==1){
			// split tree using y axis
			if ((target.y-distanceTol) < currentNode->point.y){
				searchHelper(target, currentNode->left, ids, distanceTol, depth+1);
			}
			if ((target.y+distanceTol) > currentNode->point.y){
				searchHelper(target, currentNode->right, ids, distanceTol, depth+1);
			}
		}
		else if (depth%3==2){
			// split tree using z axis
			if ((target.z-distanceTol) < currentNode->point.z){
				searchHelper(target, currentNode->left, ids, distanceTol, depth+1);
			}
			if ((target.z+distanceTol) > currentNode->point.z){
				searchHelper(target, currentNode->right, ids, distanceTol, depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;
		searchHelper(target, root, ids, distanceTol, depth);
		return ids;
	}
};

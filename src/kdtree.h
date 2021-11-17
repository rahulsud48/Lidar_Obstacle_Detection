/* \author Aaron Brown */
// Quiz on implementing kd tree

// #include "render/render.h"
#include<iostream>



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

// void print_vec(std::vector<float> v){
// 	std::cout<<"("<<v[0]<<","<<v[1]<<")"<<"\n";
// }

// void print_node(Node** node){
// 	if(*node == NULL){
// 		std::cout<<"Root: Null" << "\n";
// 	} else {
// 		std::cout << "Root: ";
// 		print_vec((*node)->point);
// 		if ((*node)->left == NULL){
// 			std::cout<<"Left: Null" << "\n";
// 		} else {
// 			std::cout << "Left: ";
// 			print_vec(((*node)->left)->point);
// 		}

// 		if ((*node)->right == NULL){
// 			std::cout<<"Right: Null" <<"\n";
// 		} else {
// 			std::cout << "Right: ";
// 			print_vec(((*node)->right)->point);
// 		}
// 	}
// }

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

	void insertRecursive(Node** node, uint depth, std::vector<float> point, int id){
		// if tree empty
		if(*node==NULL){
			*node = new Node(point, id);
		} 
		else {
			uint cd = depth % 3;
			
			
			if(point[cd] < ((*node)->point[cd])){
				insertRecursive(&((*node)->left), depth+1, point, id);
			}
			else {
				insertRecursive(&((*node)->right), depth+1, point, id);
			}
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertRecursive(&root, 0, point, id);
	}

	bool checkBox(Node* node, std::vector<float> target, float dt){
		float x_n = node->point[0];
		float y_n = node->point[1];
		float z_n = node->point[2];
		float x_t = target[0];
		float y_t = target[1];
		float z_t = target[2];
		return (x_n >= x_t - dt) && (x_n <= x_t + dt) && (y_n >= y_t - dt) && (y_n <= y_t + dt) && (z_n >= z_t - dt) && (z_n <= z_t + dt);
	}

	float clacDistance(Node* node, std::vector<float> target){
		float x_n = node->point[0];
		float y_n = node->point[1];
		float z_n = node->point[2];
		float x_t = target[0];
		float y_t = target[1];
		float z_t = target[2];

		float x_diff = x_n - x_t;
		float y_diff = y_n - y_t;
		float z_diff = z_n - z_t;
		return sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
	}

	void searchRecursive(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!=NULL)
		{

			if (checkBox(node, target, distanceTol))
			{

				float distance = clacDistance(node, target);
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// recursive iterate
			int di = depth % 3;
			if (target[di] - distanceTol  < node->point[di])
				searchRecursive(target, node->left, depth+1, distanceTol, ids);
			if (target[di] + distanceTol  > node->point[di])
				searchRecursive(target, node->right, depth+1, distanceTol, ids);
		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRecursive(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};





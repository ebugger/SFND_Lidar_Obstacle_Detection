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
	//node pointer was actually what root was to begin with
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert_helper(Node** node, uint depth, std::vector<float> point, int id) {
		if(*node == NULL) {
			//hey, root ptr! here is the new addr you should to point to. Ressign node in the tree
			*node = new Node(point, id);
		}else {
			uint cd = depth % 2;

			if(point[cd] < ((*node)->point[cd])) {
				//depth++;
				insert_helper(&((*node)->left), depth + 1, point, id);
			}else {
				//depth++;
				insert_helper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		//really want to abale to assign this node in the tree, so for pass root address and depth 0 for starting resursive
		insert_helper(&root, 0, point, id);

	}

	void search_helper(Node* root, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)  {
		if(root != NULL) {
			//all 3 if are parallel, maybe independent mybe union mybe none
			if((fabs( root->point[0] - target[0] ) < distanceTol) &&  (fabs( root->point[1] - target[1] ) < distanceTol) ) {
				float dis = std::sqrt(std::pow( (root->point[0] - target[0]), 2 ) + std::pow( (root->point[1] - target[1]), 2) );
				if (dis < distanceTol) {
					ids.push_back(root->id);
				}
			}
				uint cd = depth % 2;
			//very left of the box is on the strip left
			if( (target[cd] - distanceTol) < (root->point[cd]) ){
				search_helper(root->left, depth + 1, target, distanceTol, ids);
			}
			//very right of the box is on the strip right
			if( (target[cd] + distanceTol) > (root->point[cd]) ) {
				search_helper(root->right, depth + 1, target, distanceTol, ids);
			}

		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};






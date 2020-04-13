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

	void insertRecursion(Node** node, int depth ,std::vector<float> point, int id)
	{
		if(*node == NULL)
			{
				*node = new Node(point,id);
			}
		else
		{
			int cd = depth % 2;
			
			if( point[cd] < (*node)->point[cd])
			{
				insertRecursion(&((*node)->left),depth +1 , point, id);
			}
			else
			{
				insertRecursion(&((*node)->right),depth +1 , point, id);
			}
			
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
	
		insertRecursion(&root, 0 , point, id);
		
	}
	bool insideBox(std::vector<float> point, std::vector<float> target,float distanceTol)
	{
		return (
			(point[0]>=target[0]-distanceTol) && (point[0]<=target[0]+distanceTol)
			) &&
		(
			(point[1]>=target[1]-distanceTol) && (point[1]<=target[1]+distanceTol)
			);
	
	}
	bool insideCircle(std::vector<float> point, std::vector<float> target,float distanceTol)
	{
		float distance = sqrt(
			(point[0]-target[0])*(point[0]-target[0]) +
			(point[1]-target[1])*(point[1]-target[1])    );// +
		//(point[0]-target[0])*(point[0]-target[0])  );
		return distance <= distanceTol;
	}
	

	void searchRecursion(Node* node,std::vector<int>& ids,int depth, const std::vector<float> target,float distanceTol)
	{
		if(node != NULL)
		{
			if(insideBox(node->point,target,distanceTol))
			{
				if(insideCircle(node->point,target,distanceTol))
					{
						ids.push_back(node->id);
					}
			}
			if((target[depth%2]-distanceTol) < node->point[depth%2])
				{
					searchRecursion(node->left, ids, depth +1 , target, distanceTol);
				}

			if((target[depth%2]+distanceTol) > node->point[depth%2])
				{
					searchRecursion(node->right, ids, depth +1 , target, distanceTol);
				}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchRecursion(root,ids,0, target,distanceTol);


		return ids;
	}
	

};




